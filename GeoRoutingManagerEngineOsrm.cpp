#include "GeoRoutingManagerEngineOsrm.h"
#include "osrm/coordinate.hpp"
#include "osrm/engine/api/flatbuffers/fbresult_generated.h"

#include <QDebug>

GeoRoutingManagerEngineOsrm::GeoRoutingManagerEngineOsrm(const QVariantMap &parameters,
    QGeoServiceProvider::Error *error, QString *errorString) :
    QGeoRoutingManagerEngine(parameters),
#ifdef USE_Thread
    worker_(new WorkerThread(this)),
#endif
    routeReply_(nullptr),
    errorCode_(QGeoRouteReply::NoError)
{
#ifdef USE_Thread
    bool ok = connect(worker_, SIGNAL(finished()), this, SLOT(updateRoutes()));
    assert(ok);
#endif

    if (error)
        *error = QGeoServiceProvider::NoError;
    if (errorString)
        *errorString = QString();

    auto setError = [=](QGeoServiceProvider::Error errorCode, QString errorMsg)
    {
        if (error)
            *error = errorCode;
        if (errorString)
            *errorString = errorMsg;
        qWarning() << errorMsg;
    };

    if (parameters.contains("engineConfig.storage_config")) {
        QString storage {qvariant_cast<QString>(parameters.value("engineConfig.storage_config"))};
        engineConfig.storage_config = boost::filesystem::path{storage.toStdString()};
        if ( ! engineConfig.storage_config.IsValid())
        {
            setError(QGeoServiceProvider::ConnectionError,
                "OSRM Routing plugin: " + storage + " file not found");
            return;
        }
    }
    else
    {
        setError(QGeoServiceProvider::MissingRequiredParameterError,
            "OSRM Routing plugin: storage_config parameter missing");
        return;
    }

    if (parameters.contains("engineConfig.use_shared_memory")) {
        engineConfig.use_shared_memory = qvariant_cast<bool>(parameters.value("engineConfig.use_shared_memory"));
    }

    if (parameters.contains("engineConfig.use_mmap"))
        engineConfig.use_mmap = qvariant_cast<bool>(parameters.value("engineConfig.use_mmap"));
    else
        engineConfig.use_mmap = true;

    if (parameters.contains("engineConfig.algorithm")) {
        auto algorithm = qvariant_cast<QString>(parameters.value("engineConfig.algorithm"));
        if (algorithm == "CH")
            engineConfig.algorithm = osrm::engine::EngineConfig::Algorithm::CH;
        else if (algorithm == "MLD")
            engineConfig.algorithm = osrm::engine::EngineConfig::Algorithm::MLD;
        else
        {
            setError(QGeoServiceProvider::NoError,
                "OSRM Routing plugin: invalid algorithm: " + algorithm);
        }
    }

    if ( ! engineConfig.IsValid())
    {
#if (QT_VERSION >= QT_VERSION_CHECK(5, 12, 1))
        setError(QGeoServiceProvider::LoaderError, "OSRM Routing plugin: invalid engineConfig");
#else
        setError(QGeoServiceProvider::ConnectionError, "OSRM Routing plugin: invalid engineConfig");
#endif
        return;
    }

    osrm = std::make_unique<osrm::OSRM>(engineConfig);

//    setLocale(QLocale (QLocale::German, QLocale::Germany));
    //  TODO: разобраться с setSupported*
    setSupportedFeatureTypes (
                  QGeoRouteRequest::NoFeature
                | QGeoRouteRequest::TollFeature
                | QGeoRouteRequest::HighwayFeature
                | QGeoRouteRequest::PublicTransitFeature
                | QGeoRouteRequest::FerryFeature
                | QGeoRouteRequest::TunnelFeature
                | QGeoRouteRequest::DirtRoadFeature
                | QGeoRouteRequest::ParksFeature
                | QGeoRouteRequest::MotorPoolLaneFeature );
    setSupportedFeatureWeights (
                  QGeoRouteRequest::NeutralFeatureWeight
                | QGeoRouteRequest::PreferFeatureWeight
                | QGeoRouteRequest::RequireFeatureWeight
                | QGeoRouteRequest::AvoidFeatureWeight
                | QGeoRouteRequest::DisallowFeatureWeight );
    setSupportedManeuverDetails (
                  QGeoRouteRequest::NoManeuvers
                | QGeoRouteRequest::BasicManeuvers);
    setSupportedRouteOptimizations (
                  QGeoRouteRequest::ShortestRoute
                | QGeoRouteRequest::FastestRoute
                | QGeoRouteRequest::MostEconomicRoute
                | QGeoRouteRequest::MostScenicRoute);
    setSupportedSegmentDetails (
                  QGeoRouteRequest::NoSegmentData
                | QGeoRouteRequest::BasicSegmentData );
    setSupportedTravelModes (
                  QGeoRouteRequest::CarTravel
                | QGeoRouteRequest::PedestrianTravel
                | QGeoRouteRequest::BicycleTravel
                | QGeoRouteRequest::PublicTransitTravel
                | QGeoRouteRequest::TruckTravel );

//    setSupportsRouteUpdates(true);
}

QGeoRouteReply* GeoRoutingManagerEngineOsrm::calculateRoute(const QGeoRouteRequest& request)
{

    routeReply_ = new RouteReply();
#ifdef USE_Thread
    routeReply_->setFinished(false);
    assert( ! routeReply_->isFinished());
    worker_->request = request;
    worker_->start();
#else
    bool ok = connect(routeReply_, SIGNAL(aborted()), this, SLOT(requestAborted()));
    assert(ok);

    auto result = calcRoutes(request);

    if (std::get<QGeoRouteReply::Error>(result) == QGeoRouteReply::Error::NoError)
    {
        routeReply_->setRoutes(std::get<QList<QGeoRoute>>(result));
        routeReply_->setError(QGeoRouteReply::NoError, "no error");
        routeReply_->setFinished(true);
    }
    else
    {
        routeReply_->setError(std::get<QGeoRouteReply::Error>(result), std::get<QString>(result));
    }
#endif

    return routeReply_;
}

std::tuple<QGeoRouteReply::Error, QString, QList<QGeoRoute>>
GeoRoutingManagerEngineOsrm::calcRoutes(const QGeoRouteRequest& request)const
{
    QGeoRouteReply::Error error = QGeoRouteReply::Error::NoError;
    QString errorString;
    QList<QGeoRoute> routesOut;

    osrm::engine::api::RouteParameters params;

//    params.steps = true;
//    params.alternatives = true;
//    params.annotations = true;
//    params.annotations_type = osrm::engine::api::RouteParameters::AnnotationsType::All;
    params.overview = osrm::engine::api::RouteParameters::OverviewType::Full;
    params.geometries = osrm::engine::api::RouteParameters::GeometriesType::GeoJSON;

    for(auto waypoint : request.waypoints())
    {
        params.coordinates.push_back({
            osrm::FloatLongitude{waypoint.longitude()},
            osrm::FloatLatitude{waypoint.latitude()}
        });
    }

    osrm::engine::api::ResultT result = flatbuffers::FlatBufferBuilder();

    // Execute routing request, this does the heavy lifting
    const auto status = osrm->Route(params, result);

    if (status == osrm::engine::Status::Ok)
    {
        auto &fb_result = result.get<flatbuffers::FlatBufferBuilder>();
        auto result = osrm::engine::api::fbresult::GetFBResult(fb_result.GetBufferPointer());
        assert(result);
        assert( ! result->error());
        assert(result->routes());
        const auto& routes = *result->routes();
        assert(routes.size() > 0);
        for(const auto route : routes)
        {
            assert(route);
            assert(route->coordinates());
            QList<QGeoCoordinate> path;
            for (auto pos : *route->coordinates())
            {
                assert(pos);
                path << QGeoCoordinate(pos->latitude(), pos->longitude());
            }

            QGeoRoute routeOut;
            routeOut.setPath(path);
            routeOut.setDistance(route->distance());
            routeOut.setTravelTime(route->duration());
            routesOut.append(routeOut);
        }

    }
    else if (status == osrm::engine::Status::Error)
    {
        //  обработка ошибок
        error = QGeoRouteReply::Error::UnknownError;
    }

    return {error, errorString, routesOut};
}

void GeoRoutingManagerEngineOsrm::requestAborted()
{
#ifdef USE_Thread
    if (worker_->isRunning())
    {
        worker_->terminate();
        //  This function is dangerous and its use is discouraged.
        //  The thread can be terminated at any point in its code path.
        //  Threads can be terminated while modifying data.
        //  There is no chance for the thread to clean up after itself,
        //  unlock any held mutexes, etc.
        //  In short, use this function only if absolutely necessary.
    }
#endif
}

#ifdef USE_Thread

void GeoRoutingManagerEngineOsrm::updateRoutes()
{
    assert(worker_->isFinished());
    if (worker_->error == QGeoRouteReply::Error::NoError)
    {
        routeReply_->setRoutes(worker_->routes);
        routeReply_->setFinished(true);
        emit finished(routeReply_);
    }
    else
    {
        routeReply_->setError(worker_->error, worker_->errorString);
        emit error(routeReply_, worker_->error, worker_->errorString);
    }
}

void WorkerThread::run()
{
    auto result = owner_->calcRoutes(request);
    error       = std::get<QGeoRouteReply::Error>(result);
    errorString = std::get<QGeoRouteReply::Error>(result);
    routes      = std::get<QList<QGeoRoute>>(result);
};
#endif



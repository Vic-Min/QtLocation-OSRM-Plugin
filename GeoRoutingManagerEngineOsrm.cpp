#include "GeoRoutingManagerEngineOsrm.h"
#include "osrm/coordinate.hpp"
#include "osrm/engine/api/flatbuffers/fbresult_generated.h"

#include <QDebug>

GeoRoutingManagerEngineOsrm::GeoRoutingManagerEngineOsrm(const QVariantMap &parameters,
    QGeoServiceProvider::Error *error, QString *errorString) :
    QGeoRoutingManagerEngine(parameters),
    worker_(new WorkerThread(this)),
    routeReply_(nullptr)
{
    bool ok = connect(worker_, SIGNAL(finished()), this, SLOT(updateRoutes()));
    assert(ok);

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

    try
    {
        osrm = std::make_unique<osrm::OSRM>(engineConfig);
    }
    catch (const std::exception& e)
    {
        setError(
#if (QT_VERSION >= QT_VERSION_CHECK(5, 12, 1))
            QGeoServiceProvider::LoaderError,
#else
            QGeoServiceProvider::ConnectionError,
#endif
            QString("OSRM Routing plugin: ") + e.what());
        return;
    }

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
    auto routeReply = new RouteReply(request, this);
    routeReply_.store(routeReply);

    bool ok = connect(routeReply, SIGNAL(aborted()), this, SLOT(requestAborted()));
    assert(ok);

    worker_->start();

    return routeReply;
}

void GeoRoutingManagerEngineOsrm::calcRoutes()const
{
    // routeReply_ can be changed on another thread while route calculation is
    // in progress, so let's save it in routeReply
    RouteReply* routeReply = routeReply_.load();
    assert(routeReply);
    const QGeoRouteRequest request = routeReply->request();

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

    osrm::engine::api::ResultT resultVariant = flatbuffers::FlatBufferBuilder();

    // Execute routing request, this does the heavy lifting
    const osrm::engine::Status status = osrm->Route(params, resultVariant);
    const auto &fbBuffer = resultVariant.get<flatbuffers::FlatBufferBuilder>();
    const osrm::engine::api::fbresult::FBResult* result = osrm::engine::api::fbresult::GetFBResult(fbBuffer.GetBufferPointer());
    assert(result);

    if (status == osrm::engine::Status::Ok)
    {
        QList<QGeoRoute> routesOut;
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
        routeReply->setRoutes(routesOut);
        routeReply->setFinished(true);
    }
    else
    {
        QString errorString;
        if (result->error())
        {
            const osrm::engine::api::fbresult::Error* errorCode = result->code();
            const flatbuffers::String* code = errorCode->code();
            const flatbuffers::String* message = errorCode->message();
            errorString = QString("code: %1, message: %2").arg(code->c_str()).arg(message->c_str());
        }
        else
        {
            errorString = "UnknownError";
        }
        routeReply->setError(QGeoRouteReply::Error::UnknownError, errorString);
    }
}

void GeoRoutingManagerEngineOsrm::requestAborted()
{
    if (worker_->isRunning())
    {
        if (routeReply == routeReply_.load())
            routeReply_.store(nullptr);
        // unfortunately there is no way to interrupt OSRM calculations,
        // so you have to kill the thread in which OSRM is running 
        worker_->terminate();
        worker_->wait();

        auto routeReply = qobject_cast<RouteReply*>(sender());
        assert(routeReply);
        routeReply->setError(QGeoRouteReply::UnknownError, "aborted");
    }
}

void GeoRoutingManagerEngineOsrm::updateRoutes()
{
    assert(worker_->isFinished());
    RouteReply* routeReply = routeReply_.load();
    assert(routeReply);
    if (routeReply->error() == QGeoRouteReply::Error::NoError)
    {
        emit finished(routeReply);
    }
    else
    {
        emit error(routeReply, routeReply->error(), routeReply->errorString());
    }
}

void WorkerThread::run()
{
    owner_->calcRoutes();
};

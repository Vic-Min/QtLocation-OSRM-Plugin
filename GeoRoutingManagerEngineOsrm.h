#ifndef GeoRoutingManagerEngineOsrm_H
#define GeoRoutingManagerEngineOsrm_H

#include <QGeoRoutingManagerEngine>
#include <QGeoServiceProvider>

//#define USE_Thread

#ifdef USE_Thread
#include <QThread>
#endif

#include "osrm/osrm.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/route_parameters.hpp"


QT_USE_NAMESPACE


class RouteReply :public QGeoRouteReply
{
    Q_OBJECT

public:
    RouteReply(const QGeoRouteRequest& request, QObject *parent = nullptr)
        : QGeoRouteReply (request, parent)
    {}
    using QGeoRouteReply::setError;
    using QGeoRouteReply::setFinished;
    using QGeoRouteReply::setRoutes;
};

#ifdef USE_Thread
class WorkerThread;
#endif

class GeoRoutingManagerEngineOsrm: public QGeoRoutingManagerEngine
{
    Q_OBJECT

public:
    GeoRoutingManagerEngineOsrm(const QVariantMap &parameters,
        QGeoServiceProvider::Error *error, QString *errorString);

    QGeoRouteReply* calculateRoute(const QGeoRouteRequest& request)override;
//    QGeoRouteReply* updateRoute(const QGeoRoute &route, const QGeoCoordinate &position)override;

private slots:
    void requestAborted();
#ifdef USE_Thread
    void updateRoutes();
#endif

private:
    std::tuple<QGeoRouteReply::Error, QString, QList<QGeoRoute>> calcRoutes(const QGeoRouteRequest& request)const;

    osrm::engine::EngineConfig engineConfig;
    osrm::engine::api::RouteParameters routeParameters;
    std::unique_ptr<const osrm::OSRM> osrm;
#ifdef USE_Thread
friend class WorkerThread;
    WorkerThread* worker_;
#endif
    RouteReply* routeReply_;
    QGeoRouteReply::Error errorCode_;
    QString errorString_;
};

#ifdef USE_Thread
class WorkerThread : public QThread
{
    Q_OBJECT
public:
    WorkerThread(GeoRoutingManagerEngineOsrm *parent=nullptr)
        : QThread(parent)
        , owner_(parent)
    {
    };
    QGeoRouteReply::Error error = QGeoRouteReply::Error::NoError;
    QString errorString;
    QList<QGeoRoute> routes;

private:
    void run() override;
    GeoRoutingManagerEngineOsrm* owner_;
};
#endif

#endif

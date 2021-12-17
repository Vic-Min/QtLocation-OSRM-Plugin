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
    RouteReply(QObject *parent=0) :QGeoRouteReply (QGeoRouteRequest(), parent)
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
    osrm::engine::EngineConfig engineConfig;
    osrm::engine::api::RouteParameters routeParameters;
    std::unique_ptr<const osrm::OSRM> osrm;
#ifdef USE_Thread
    WorkerThread* worker_;
#endif

    RouteReply* routeReply_;
    QGeoRouteReply::Error errorCode_;
    QString errorString_;

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

#ifdef USE_Thread
friend class WorkerThread;
#endif
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
    QGeoRouteRequest request;
    QGeoRouteReply::Error error = QGeoRouteReply::Error::NoError;
    QString errorString;
    QList<QGeoRoute> routes;

private:
    void run() override;
    GeoRoutingManagerEngineOsrm* owner_;
};
#endif

#endif

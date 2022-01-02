#ifndef GeoRoutingManagerEngineOsrm_H
#define GeoRoutingManagerEngineOsrm_H

#include <QGeoRoutingManagerEngine>
#include <QGeoServiceProvider>
#include <QThread>

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

class WorkerThread;

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
    void requestFinished();
    void requestError(QGeoRouteReply::Error error, const QString &errorString);

private:
    void calcRoutes()const;

    osrm::engine::EngineConfig engineConfig;
    osrm::engine::api::RouteParameters routeParameters;
    std::unique_ptr<const osrm::OSRM> osrm;
friend class WorkerThread;
    WorkerThread* worker_;
    QAtomicPointer<RouteReply> routeReply_;
};

class WorkerThread : public QThread
{
    Q_OBJECT
public:
    WorkerThread(GeoRoutingManagerEngineOsrm *parent=nullptr)
        : QThread(parent)
        , owner_(parent)
    {
    };

private:
    void run() override;
    GeoRoutingManagerEngineOsrm* owner_;
};

#endif

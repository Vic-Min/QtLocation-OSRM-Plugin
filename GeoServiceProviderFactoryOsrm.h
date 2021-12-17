#ifndef GeoServiceProviderFactoryOsrm_H
#define GeoServiceProviderFactoryOsrm_H

#include <qgeoserviceproviderfactory.h>
#include <QObject>

QT_USE_NAMESPACE

class GeoServiceProviderFactoryOsrm: public QObject, public QGeoServiceProviderFactory
{
    Q_OBJECT
    Q_INTERFACES(QGeoServiceProviderFactory)
    Q_PLUGIN_METADATA(IID "org.qt-project.qt.geoservice.serviceproviderfactory/5.0"
                      FILE "osrm_plugin.json")

public:
    GeoServiceProviderFactoryOsrm();
    ~GeoServiceProviderFactoryOsrm();

    QGeoRoutingManagerEngine* createRoutingManagerEngine(
                const QVariantMap &parameters,
                QGeoServiceProvider::Error *error, QString *errorString ) const;
};

#endif



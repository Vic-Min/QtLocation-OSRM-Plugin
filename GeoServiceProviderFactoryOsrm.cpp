#include "GeoServiceProviderFactoryOsrm.h"
#include "GeoRoutingManagerEngineOsrm.h"

#include <QtPlugin>


GeoServiceProviderFactoryOsrm::GeoServiceProviderFactoryOsrm()
{
}

GeoServiceProviderFactoryOsrm::~GeoServiceProviderFactoryOsrm()
{
}

QGeoRoutingManagerEngine* GeoServiceProviderFactoryOsrm::createRoutingManagerEngine(
            const QVariantMap& parameters,
            QGeoServiceProvider::Error* error, QString* errorString) const
{
    return new GeoRoutingManagerEngineOsrm(parameters, error, errorString);
}

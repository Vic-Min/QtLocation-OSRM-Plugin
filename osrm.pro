TARGET = qtgeoservices_osrm
QT += location positioning-private
CONFIG += exceptions
PLUGIN_TYPE = geoservices
PLUGIN_CLASS_NAME = GeoServiceProviderFactoryOsrm
PLUGIN_EXTENDS = -
load(qt_plugin)

QMAKE_CXXFLAGS += -D__EXCEPTIONS -g

INCLUDEPATH += \
	../../../../include/QtLocation/5.11.1 \
	/usr/local/include/osrm \

HEADERS += \
	GeoServiceProviderFactoryOsrm.h \
	GeoRoutingManagerEngineOsrm.h

SOURCES += \
	GeoServiceProviderFactoryOsrm.cpp \
	GeoRoutingManagerEngineOsrm.cpp

LIBS += \
    -L/usr/local/lib \
    -losrm \
    -lrt \
    -lboost_filesystem \
    -lboost_iostreams \
    -lboost_thread \
    -lboost_system \


OTHER_FILES +=

DISTFILES += \
	osrm_plugin.json

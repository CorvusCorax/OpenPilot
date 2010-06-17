
TEMPLATE = lib 
TARGET = NotifyPlugin 
 
include(../../openpilotgcsplugin.pri) 
include(../../plugins/coreplugin/coreplugin.pri) 
include(notifyplugin_dependencies.pri)

QT        += phonon

HEADERS += notifyplugin.h \  
    notifypluginoptionspage.h \
    notifypluginconfiguration.h
SOURCES += notifyplugin.cpp \  
    notifypluginoptionspage.cpp \
    notifypluginconfiguration.cpp
 
OTHER_FILES += notifyplugin.pluginspec

FORMS += \
    notifypluginoptionspage.ui

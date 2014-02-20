#-------------------------------------------------
#
# Project created by QtCreator 2013-04-13T18:48:44
#
#-------------------------------------------------

QT       += core gui xml

TARGET = control_panel
TEMPLATE = app


SOURCES += src/main.cpp\
        src/ControlPanel.cpp \
    src/RobotWidgetsDock.cpp \
    src/RobotWorkspace.cpp \
    src/ROSInterface.cpp \
    src/ControlPanelPlugin.cpp \
    src/RobotTab.cpp

HEADERS  += include/control_panel/ControlPanel.h \
    include/control_panel/RobotWidgetsDock.h \
    include/control_panel/RobotWorkspace.h \
    include/control_panel/ROSInterface.h \
    include/control_panel/ControlPanelPlugin.h \
    include/control_panel/RobotTab.h

FORMS    += ui/ControlPanel.ui

INCLUDEPATH  += include

DESTDIR = bin
OBJECTS_DIR = build/obj
MOC_DIR = build/moc
RCC_DIR = build/rcc
UI_DIR = build/ui

QT += core gui opengl widgets openglwidgets

CONFIG += c++17

INCLUDEPATH += include

DISTFILES += \
    README.md \
    Markdown.md


SOURCES += \
    src/DemoScenario.cpp \
    src/main.cpp \
    src/widget.cpp \
    src/window.cpp \
    src/RRTPlanner.cpp \
    src/VelocityIntegrator.cpp \



HEADERS += \
    include/DemoScenario.h \
    include/IPlanner.h \
    include/widget.h \
    include/window.h \
    include/Config.h \
    include/RRTPlanner.h \
    include/VelocityIntegrator.h \

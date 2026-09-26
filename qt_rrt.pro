QT += core gui opengl widgets openglwidgets

CONFIG += c++17


# para incluir PQP
# PQP_ROOT = /home/angelgl/PQP
# LIBS += $$PQP_ROOT/lib/libPQP.a
# PRE_TARGETDEPS += $$PQP_ROOT/lib/libPQP.a
# INCLUDEPATH += $$PQP_ROOT/src

# RUTA RELATIVA (REQUIERE /third_party/PQP )
PQP_ROOT = third_party/PQP
INCLUDEPATH += $$PQP_ROOT
SOURCES += \
    $$PQP_ROOT/Build.cpp \
    $$PQP_ROOT/BV.cpp \
    $$PQP_ROOT/PQP.cpp \
    $$PQP_ROOT/TriDist.cpp

DISTFILES += \
    README.md \
    Markdown.md \
    data/escenario1.in \
    data/obstaculos1.in \


INCLUDEPATH += include
SOURCES += \
    src/DemoScenario.cpp \
    src/PQPCollision.cpp \
    src/PolygonGeometry.cpp \
    src/main.cpp \
    src/widget.cpp \
    src/window.cpp \
    src/RRTPlanner.cpp \
    src/VelocityIntegrator.cpp \



HEADERS += \
    include/DemoScenario.h \
    include/IPlanner.h \
    include/Obstacle.h \
    include/PQPCollision.h \
    include/PolygonGeometry.h \
    include/widget.h \
    include/window.h \
    include/Config.h \
    include/RRTPlanner.h \
    include/VelocityIntegrator.h

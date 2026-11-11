#include "pruebas_rapido.h"
#include "VO.h"

#include "logger.h"
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <QString>
#include <QStringList>

#include "DemoScenario.h" // para encotnrar ruta

PruebasRapido::PruebasRapido(QWidget* parent) : QOpenGLWidget(parent) {

    const std::string path_ = encontrarPath("data/dataprueba/poligonos.in");


    const QString path = QString::fromUtf8(path_);
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        LOG_ERROR("NO SE ABRE las pruebas rapido input");
        return;
    }

    LOG_INFO("Se abrio el archivo");


    QTextStream in(&file);
    int count = 0;
    in >> count;
    for (int i = 0; i < count && in.status() == QTextStream::Ok; ++i) {
        PolygonObstacle polygon;
        QString polygonName;
        int vertices = 0;
        in >> polygonName >> vertices;
        polygon.name = polygonName.toStdString();
        if (vertices < 3) {
            polygons_.clear();
            return;
        }
        polygon.vertices.resize(static_cast<std::size_t>(vertices));
        for (auto& point : polygon.vertices)
            in >> point.x >> point.y;
        if (in.status() != QTextStream::Ok) {
            polygons_.clear();
            return;
        }
        polygons_.push_back(std::move(polygon));
    }
    if (polygons_.size() == 2)
        polygons_.push_back(minkowskiSum(polygons_[0], polygons_[1]));
    else
        polygons_.clear();
}

void PruebasRapido::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.97f, 0.98f, 0.99f, 1.0f);
}

void PruebasRapido::resizeGL(int width, int height) {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, 500.0, 0.0, 500.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
}

void PruebasRapido::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    const float colors[3][4] = {
        {0.15f, 0.45f, 0.85f, 0.75f},
        {0.90f, 0.35f, 0.20f, 0.75f},
        {0.20f, 0.65f, 0.35f, 0.45f}
    };
    for (std::size_t i = 0; i < polygons_.size(); ++i) {
        const auto& polygon = polygons_[i];
        glColor4fv(colors[i]);
        glBegin(GL_POLYGON);
        for (const auto& point : polygon.vertices)
            glVertex2d(point.x, point.y);
        glEnd();
        glColor4f(0.08f, 0.10f, 0.14f, 1.0f);
        glBegin(GL_LINE_LOOP);
        for (const auto& point : polygon.vertices)
            glVertex2d(point.x, point.y);
        glEnd();
    }
}

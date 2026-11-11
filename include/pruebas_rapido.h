#pragma once

#include "Obstacle.h"

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <vector>

class PruebasRapido : public QOpenGLWidget, protected QOpenGLFunctions {
public:
    explicit PruebasRapido(QWidget* parent = nullptr);

protected:
    void initializeGL() override;
    void resizeGL(int width, int height) override;
    void paintGL() override;

private:
    std::vector<PolygonObstacle> polygons_;
};

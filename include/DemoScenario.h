#pragma once
#include <fstream>
#include <Obstacle.h>

#include <string>
#include <vector>
#include <iostream>
#include <sstream>

#include <exception>
#include <fstream>
#include <utility>

class RRTWidget;

struct SceneData {
    int numRobots = 0;
    std::vector<float> xi, yi, thi;
    std::vector<float> xf, yf, thf;
    std::vector<float> Vxr, Vyr, Vangr;
};

bool loadPolygonObstacles(
    const std::string& path,
    std::vector<PolygonObstacle>& output);

bool loadScene(
    const std::string& path,
    SceneData& scene);

void configureDemoScenario(
    RRTWidget* widget,
    const std::string& ruta = "../../data/escenario1.in",
    const std::string& rutaObstaculos = "../../data/obstaculos1.in");

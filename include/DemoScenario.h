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

namespace DemoScenarioDefaults {
inline constexpr float goalTolerance = 30.0f;
inline constexpr float maxExtensionStep = 40.0f;
inline constexpr int maxTreeNodes = 1000000;
inline constexpr int planningIntervalMs = 10;
inline constexpr int eulerIntervalMs = 2000;
}

class RRTWidget;

struct SceneData {
    int numRobots = 0;
    std::vector<float> xi, yi, thi;
    std::vector<float> xf, yf, thf;
    std::vector<float> Vxr, Vyr, Vangr;
    std::vector<float> radii;
};

std::string encontrarPath(const std::string& path, int max_niveles = 20);

bool loadPolygonObstacles(
    const std::string& path,
    std::vector<PolygonObstacle>& output);

bool loadScene(
    const std::string& path,
    SceneData& scene);

void configureDemoScenario(
    RRTWidget* widget,
    const std::string& ruta = "data/escenario1.in",
    const std::string& rutaObstaculos = "data/obstaculos1.in");

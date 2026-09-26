#pragma once

#include <array>
#include <string>
#include <vector>

struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

using Triangle2D = std::array<Point2D, 3>;

struct PolygonObstacle {
    std::string name;
    std::vector<Point2D> vertices;
    std::vector<Triangle2D> triangles;
};
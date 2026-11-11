#pragma once

#include "Obstacle.h"

// Suma de Minkowski para poligonos convexos.
PolygonObstacle minkowskiSum(const PolygonObstacle& a,
                             const PolygonObstacle& b);

#pragma once

#include "Obstacle.h"

// Valida y triangula
// Lanza std::invalid_argument si no es obstaculo valido
// basicamente checa que las triangulaciones si sean correctas
// Ignora cualquier triangulacion anterior y la reemplaza.
void preparePolygon(PolygonObstacle& polygon);
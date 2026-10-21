#pragma once

#include <vector>
#include <iostream>
#include <string>


//mediante esta estructura se define un nodo en el arbol del RRT
inline constexpr int kRobotShapeSegments = 32;

struct Config {
    float x, y, theta;//Pose del robot
    float radius = 5.0f; // Huella del robot; se conserva en sus distintas poses.
};

// Margen posicional más radio físico: radio aceptado para el centro del robot.
inline float goalAcceptanceRadius(const Config& goal, float margin)
{
    return goal.radius + margin;
}

struct Velocities {
    float Vx, Vy, Wang;//velocidades robots
};//new

//mediante esta estructura se define un nodo en el arbol del RRT
struct PlanNode {
    std::vector<Config> q;//coordenadas del nodo
    int parent;//indice del nodo padre

    PlanNode(const std::vector<Config>& q_, int p_)
        : q(q_), parent(p_)
    {
    }
};

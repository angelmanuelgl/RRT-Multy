#pragma once

#include <vector>
#include <iostream>
#include <string>


//mediante esta estructura se define un nodo en el arbol del RRT
inline constexpr int kRobotShapeSegments = 32;
inline constexpr float kDefaultRobotRadius = 5.0f;
inline constexpr float kCollisionSafetyMargin = 1.0f;
inline constexpr float kWorldWidth = 500.0f;
inline constexpr float kWorldHeight = 500.0f;
inline constexpr float kWorldGridSpacing = 50.0f;

struct Config {
    float x, y, theta;//Pose del robot
    float radius = kDefaultRobotRadius; // Huella del robot; se conserva en sus distintas poses.
};

// Margen posicional más radio físico: radio aceptado para el centro del robot.
inline float goalAcceptanceRadius(const Config& goal, float margin)
{
    return goal.radius + margin;
}

struct Velocities {
    float Vx, Vy, Wang;//velocidades robots
};//new

struct MotionPhase {
    float duration = 0.0f;
    std::vector<Velocities> velocities;
};

struct EdgeMotion {
    std::vector<MotionPhase> phases;
};

//mediante esta estructura se define un nodo en el arbol del RRT
struct PlanNode {
    std::vector<Config> q;//coordenadas del nodo
    int parent;//indice del nodo padre
    EdgeMotion incomingMotion;

    PlanNode(const std::vector<Config>& q_, int p_,
             const EdgeMotion& motion = {})
        : q(q_), parent(p_), incomingMotion(motion)
    {
    }
};

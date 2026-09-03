#pragma once

#include <vector>

//mediante esta estructura se define un nodo en el arbol del RRT

struct Config {
    float x, y, theta;//Pose del robot
};

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

struct PlanSnapshot {
    std::vector<PlanNode> tree;
    std::vector<int> path;
    std::vector<Config> goal;
    std::vector<Config> origin;
    int numRobots = 0;
    bool goalReached = false;
};

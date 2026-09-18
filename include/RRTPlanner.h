#pragma once

#include "IPlanner.h"
#include "PolygonGeometry.h"
#include "PQPCollision.h"
#include <cstdint>
#include <vector>

class RRTPlanner : public IPlanner {
public:
    RRTPlanner();

    void setStart(const std::vector<Config>& q, float radius) override;
    void setGoal(const std::vector<Config>& q, float radius) override;
    void setNumRobots(int n) override;
    void setObstacles( const std::vector<PolygonObstacle>& obstacles) override;

    PQPCollisionChecker::Statistics getCollisionStatistics() const;

    void setStepSize(float step) override;
    void setMaxNodes(int maxNodes) override;
    void setGoalTolerance(float distance) override;

    void reset() override;
    bool step() override;
    bool isDone() const override;

    const std::vector<PlanNode>& getTree() const override;
    const std::vector<int>& getPath() const override;
    const std::vector<Config>& getGoal() const override;
    const std::vector<Config>& getOrigin() const override;
    int getNumRobots() const override;

    int getNodeCount() const override;
    int getPathNodeCount() const override;
private:
    float randFloat(float min, float max);
    float configDistance(const std::vector<Config>& a,
                         const std::vector<Config>& b);
    std::vector<Config> steer(const std::vector<Config>& a,
                              const std::vector<Config>& b,
                              float step);
    int getNearest(const std::vector<Config>& qRand);

    // colissiones
    PQPCollisionChecker collisionChecker_;

    double collisionCheckResolution_ = 2.5;
    bool problemChecked_ = false;

    // limite de intentos de muestras
    std::uint64_t attempts_ = 0;
    std::uint64_t maxAttempts_ = 200000;

    bool isConfigurationValid(const std::vector<Config>& q) const;
    bool isEdgeValid(const std::vector<Config>& from,
                     const std::vector<Config>& to) const;

    // el arbol
    std::vector<PlanNode> tree_;//conjunto de nodos actuales (Cada nodo guarda (x,y) y parent)
    std::vector<int> finalPath_;  // indices de los nodos en el camino final
    std::vector<Config> goalQ_;//meta a llegar
    std::vector<Config> originQ_; //origen

    float goalRadius_ = 5.0f;
    float originRadius_ = 5.0f;
    float stepSize_ = 15.0f;//tamaño de paso
    float distToGoal_ = 50.0f;
    float diamRobot_ = 5.0f;

    int maxNodes_ = 2000;//maximo de nodos para detener la expansión del arbol
    int numRobots_ = 1;
    int nNodesPath_ = 0; //numero de nodos en el final path
    int nNodesRRT_ = 0;

    bool done_ = false;
};

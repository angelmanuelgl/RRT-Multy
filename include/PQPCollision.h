#pragma once

#include "Config.h"
#include "Obstacle.h"

#include <cstdint>
#include <memory>
#include <vector>

class PQPCollisionChecker {
public:
    struct Statistics {
        std::uint64_t queries = 0;
        std::uint64_t bvTests = 0;
        std::uint64_t triangleTests = 0;
    };

    PQPCollisionChecker();
    ~PQPCollisionChecker();

    PQPCollisionChecker(const PQPCollisionChecker&) = delete;
    PQPCollisionChecker& operator=(const PQPCollisionChecker&) = delete;

    void setRobotRadius(double radius);
    void setObstacles(const std::vector<PolygonObstacle>& obstacles);

    bool configurationInCollision(
        const std::vector<Config>& q) const;

    bool edgeInCollision(
        const std::vector<Config>& from,
        const std::vector<Config>& to,
        double resolution) const;

    Statistics statistics() const;
    void resetStatistics();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
#pragma once

#include "Config.h"

#include <cstddef>
#include <vector>

bool interpolateEdgeMotion(const std::vector<Config>& start,
                           const std::vector<Config>& target,
                           const EdgeMotion& motion, float elapsed,
                           std::vector<Config>& state);

class AnimMov {
public:
    bool setPath(const std::vector<PlanNode>& tree,
                 const std::vector<int>& path);
    bool start();
    void reset();
    void clear();
    bool advance(float deltaTime);

    const std::vector<Config>& state() const;
    bool hasPath() const;
    bool isActive() const;
    bool isFinished() const;

private:
    struct Segment {
        std::vector<Config> start;
        std::vector<Config> target;
        EdgeMotion motion;
        float duration = 0.0f;
    };

    std::vector<Segment> segments_;
    std::vector<Config> state_;
    std::size_t segmentIndex_ = 0;
    float segmentElapsed_ = 0.0f;
    bool pathReady_ = false;
    bool started_ = false;
    bool finished_ = false;
};

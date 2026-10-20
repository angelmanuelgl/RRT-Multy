#pragma once

#include "Config.h"
#include "Obstacle.h"

#include <vector>

namespace auxDraw {

struct Color {
    float red;
    float green;
    float blue;
    float alpha;
};

// Draws reusable world-space primitives. All functions require an active OpenGL context.
void drawGrid();
void drawObstacles(const std::vector<PolygonObstacle>& obstacles);
void drawCircle(const Config& center, float radius, const Color& fill,
                const Color& outline, int segments = 36);
void drawRobot(const Config& pose, const Color& color,
               bool showHeading = false);
void drawTree(const std::vector<PlanNode>& tree, bool drawNodes,
              float edgeAlpha = 0.48f, int highlightedRobot = -1);
void drawPath(const std::vector<PlanNode>& tree,
              const std::vector<int>& path, bool drawPathNodes,
              int highlightedRobot = -1);
void drawEulerSegment(const std::vector<Config>& previous,
                      const std::vector<Config>& next);
void drawGoalRegions(const std::vector<Config>& goals, float radius);
void drawOriginRegions(const std::vector<Config>& origins, float radius);

} // namespace auxDraw

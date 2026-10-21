#include "auxDraw.h"
#include "logger.h"
#include <QOpenGLFunctions>

#include <algorithm>
#include <cmath>
#include <tuple>

namespace auxDraw {
namespace {
constexpr int kMarkerSegments = kRobotShapeSegments;
constexpr float kPi = 3.14159265358979323846f;

std::tuple<float, float, float> colorForRobot(int robot) {
    float red = ((robot * 97) % 255) / 255.0f;
    float green = ((robot * 150) % 255) / 255.0f;
    float blue = ((robot * 19) % 255) / 255.0f;
    constexpr float minimum = 0.2f;
    return {std::max(red, minimum), std::max(green, minimum),
            std::max(blue, minimum)};
}

void drawOutline(const Config& center, float radius, int segments) {
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < segments; ++i) {
        const float angle = i * 2.0f * kPi / segments;
        glVertex2f(center.x + radius * std::cos(angle),
                   center.y + radius * std::sin(angle));
    }
    glEnd();
}
} // namespace

void drawGrid() {
    glColor4f(0.45f, 0.52f, 0.62f, 0.16f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for (int coord = 0; coord <= kWorldWidth; coord += kWorldGridSpacing) {
        glVertex2f(static_cast<float>(coord), 0.0f);
        glVertex2f(static_cast<float>(coord), kWorldHeight);
        glVertex2f(0.0f, static_cast<float>(coord));
        glVertex2f(kWorldWidth, static_cast<float>(coord));
    }
    glEnd();
}

void drawObstacles(const std::vector<PolygonObstacle>& obstacles) {
    glColor4f(0.48f, 0.52f, 0.58f, 0.58f);
    glBegin(GL_TRIANGLES);
    for (const auto& obstacle : obstacles)
        for (const auto& triangle : obstacle.triangles)
            for (const auto& point : triangle)
                glVertex2d(point.x, point.y);
    glEnd();

    glColor4f(0.16f, 0.19f, 0.24f, 0.90f);
    glLineWidth(2.0f);
    for (const auto& obstacle : obstacles) {
        glBegin(GL_LINE_LOOP);
        for (const auto& point : obstacle.vertices)
            glVertex2d(point.x, point.y);
        glEnd();
    }
    glLineWidth(1.0f);
}

void drawCircle(const Config& center, float radius, const Color& fill,
                const Color& outline, int segments) {
    if (!std::isfinite(radius) || radius <= 0.0f || segments < 3)
        return;
    glColor4f(fill.red, fill.green, fill.blue, fill.alpha);
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(center.x, center.y);
    for (int i = 0; i <= segments; ++i) {
        const float angle = i * 2.0f * kPi / segments;
        glVertex2f(center.x + radius * std::cos(angle),
                   center.y + radius * std::sin(angle));
    }
    glEnd();
    glColor4f(outline.red, outline.green, outline.blue, outline.alpha);
    drawOutline(center, radius, segments);
}

// void drawRobot(const Config& pose, float radius, const Color& color,
//                bool showHeading) {
//     glColor4f(color.red, color.green, color.blue, color.alpha);
//     drawOutline(pose, radius, kMarkerSegments);
//     if (showHeading) {
//         glBegin(GL_LINES);
//         glVertex2f(pose.x, pose.y);
//         glVertex2f(pose.x + radius * std::cos(pose.theta),
//                    pose.y + radius * std::sin(pose.theta));
//         glEnd();
//     }
// }
void drawRobot(const Config& pose, const Color& color, bool showHeading) {
    const float radius = pose.radius;
    const float boundaryRadius = radius / std::cos(kPi / kMarkerSegments);
    // AMGL // misma aproximacion circunscrita y segmentos que el modelo PQP.
    glColor4f(color.red, color.green, color.blue, color.alpha * 0.3f);
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(pose.x, pose.y);
    for (int i = 0; i <= kMarkerSegments; ++i) {
        const float angle = i * 2.0f * kPi / kMarkerSegments;
        glVertex2f(pose.x + boundaryRadius * std::cos(angle),
                   pose.y + boundaryRadius * std::sin(angle));
    }
    glEnd();

    // borde
    glColor4f(color.red, color.green, color.blue, color.alpha);
    glLineWidth(1.5f);
    drawOutline(pose, boundaryRadius, kMarkerSegments);

    // circulo interior
    drawOutline(pose, radius * 0.55f, kMarkerSegments);

    // centro
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    // Eje horizontal de la cruz
    glVertex2f(pose.x - radius * 0.25f, pose.y);
    glVertex2f(pose.x + radius * 0.25f, pose.y);
    // Eje vertical de la cruz
    glVertex2f(pose.x, pose.y - radius * 0.25f);
    glVertex2f(pose.x, pose.y + radius * 0.25f);
    glEnd();

    glLineWidth(1.0f); // Restaurar grosor por defecto
}

void drawRobotState(const std::vector<Config>& state, int highlightedRobot) {
    const bool validHighlight = highlightedRobot >= 0
        && highlightedRobot < static_cast<int>(state.size());
    for (std::size_t robot = 0; robot < state.size(); ++robot) {
        const auto [red, green, blue] = colorForRobot(static_cast<int>(robot));
        const float alpha = validHighlight && static_cast<int>(robot) != highlightedRobot
            ? 0.18f : 1.0f;
        drawRobot(state[robot], {red, green, blue, alpha}, true);
    }
}



void drawTree(const std::vector<PlanNode>& tree, bool drawNodes,
              float edgeAlpha, int highlightedRobot, int highlightedNode) {
    if (tree.empty())
        return;
    const int robotCount = static_cast<int>(tree.front().q.size());
    // AMGL // seleccionar resaltado: dibujar primero los demas y dejar el elegido encima.
    const bool validHighlight = highlightedRobot >= 0
        && highlightedRobot < robotCount;

    for (int pass = 0; pass < robotCount; ++pass){
        const int robot = validHighlight
            ? (pass == robotCount - 1 ? highlightedRobot
               : (pass >= highlightedRobot ? pass + 1 : pass))
            : pass;
        const auto [red, green, blue] = colorForRobot(robot);
        const bool highlighted = validHighlight && robot == highlightedRobot;
        const float alpha = highlighted ? 0.98f
            : (validHighlight ? edgeAlpha * 0.18f : edgeAlpha);

        glColor4f(red, green, blue, alpha);
        glLineWidth(highlighted ? 2.5f : 1.0f);
        glBegin(GL_LINES);
        for (std::size_t index = 0; index < tree.size(); ++index) {
            const auto& node = tree[index];
            if (node.parent < 0 || node.parent >= static_cast<int>(tree.size())
                || robot >= static_cast<int>(node.q.size()))
                continue;
            const auto& parent = tree[node.parent].q;
            if (robot >= static_cast<int>(parent.size()))
                continue;
            glVertex2f(node.q[robot].x, node.q[robot].y);
            glVertex2f(parent[robot].x, parent[robot].y);
        }
        glEnd();
        if (drawNodes) {
            const float nodeAlpha = highlighted ? 0.95f
                : (validHighlight ? 0.03f : 0.1f);
            for (const auto& node : tree) {
                if (robot < static_cast<int>(node.q.size()))
                    drawRobot(node.q[robot], {red, green, blue, nodeAlpha});
            }
        }
    }

    // AMGL // resaltar nodo: dibujar la rama padre-origen y marcar la configuracion elegida.
    if (highlightedNode >= 0
        && highlightedNode < static_cast<int>(tree.size())) {
        std::vector<int> selectedBranch;
        int index = highlightedNode;
        for (std::size_t count = 0;
             index >= 0 && index < static_cast<int>(tree.size())
                 && count < tree.size();
             ++count) {
            selectedBranch.push_back(index);
            const int parent = tree[index].parent;
            if (parent < 0 || parent >= index)
                break;
            index = parent;
        }
        std::reverse(selectedBranch.begin(), selectedBranch.end());
        drawPath(tree, selectedBranch, false, highlightedRobot);

        const auto& selectedConfig = tree[highlightedNode].q;
        for (std::size_t robot = 0; robot < selectedConfig.size(); ++robot) {
            if (highlightedRobot >= 0
                && static_cast<int>(robot) != highlightedRobot)
                continue;
            const Config& pose = selectedConfig[robot];
            drawCircle(pose, std::max(4.0f, pose.radius * 1.8f),
                       {1.0f, 0.72f, 0.05f, 0.10f},
                       {1.0f, 0.62f, 0.02f, 1.0f});
            drawRobot(pose, {1.0f, 0.72f, 0.05f, 1.0f});
        }
    }

    glLineWidth(1.0f);
}

void drawPath(const std::vector<PlanNode>& tree,
              const std::vector<int>& path, bool drawPathNodes,
              int highlightedRobot, float pathAlpha, float nodeAlpha) {
    if (path.empty())
        return;
    const int first = path.front();
    if (first < 0 || first >= static_cast<int>(tree.size()))
        return;
    const int robotCount = static_cast<int>(tree[first].q.size());
    // AMGL // seleccionar resaltado: atenuar las rutas restantes y dibujar al elegido al final
    const bool validHighlight = highlightedRobot >= 0
        && highlightedRobot < robotCount;
    for (int pass = 0; pass < robotCount; ++pass) {
        const int robot = validHighlight
            ? (pass == robotCount - 1 ? highlightedRobot
               : (pass >= highlightedRobot ? pass + 1 : pass))
            : pass;
        const auto [red, green, blue] = colorForRobot(robot);
        const bool highlighted = validHighlight && robot == highlightedRobot;
        glColor4f(red, green, blue,
                  highlighted ? pathAlpha : (validHighlight ? 0.16f : pathAlpha));
        glLineWidth(highlighted ? 7.0f : (validHighlight ? 3.0f : 5.0f));
        glBegin(GL_LINE_STRIP);
        for (int index : path) {
            if (index >= 0 && index < static_cast<int>(tree.size())
                && robot < static_cast<int>(tree[index].q.size()))
                glVertex2f(tree[index].q[robot].x, tree[index].q[robot].y);
        }
        glEnd();
        if (drawPathNodes) {
            const float alpha = highlighted ? nodeAlpha
                : (validHighlight ? 0.18f : nodeAlpha);
            for (int index : path) {
                if (index >= 0 && index < static_cast<int>(tree.size())
                    && robot < static_cast<int>(tree[index].q.size()))
                    drawRobot(tree[index].q[robot], {red, green, blue, alpha});
            }
        }
    }
    glLineWidth(1.0f);
}

void drawEulerSegment(const std::vector<Config>& previous,
                      const std::vector<Config>& next) {
    const std::size_t count = std::min(previous.size(), next.size());
    for (std::size_t robot = 0; robot < count; ++robot) {
        const auto [red, green, blue] = colorForRobot(static_cast<int>(robot));
        glColor4f(red, green, blue, 1.0f);
        glLineWidth(4.0f);
        glBegin(GL_LINES);
        glVertex2f(previous[robot].x, previous[robot].y);
        glVertex2f(next[robot].x, next[robot].y);
        glEnd();
        glLineWidth(1.0f);
        drawRobot(next[robot], {red, green, blue, 1.0f});
    }
}

void drawGoalRegions(const std::vector<Config>& goals, float margin) {
    for (std::size_t robot = 0; robot < goals.size(); ++robot) {
        const auto [red, green, blue] = colorForRobot(static_cast<int>(robot));
        drawCircle(goals[robot], goalAcceptanceRadius(goals[robot], margin),
                   {1.0f, 0.0f, 0.0f, 0.10f},
                   {red, green, blue, 0.95f});
    }
}

void drawOriginRegions(const std::vector<Config>& origins, float radius) {
    // for (const auto& origin : origins)
    //     drawCircle(origin, radius, {0.0f, 1.0f, 0.0f, 0.85f},
    //                {0.0f, 0.45f, 0.0f, 1.0f});
    for (std::size_t i = 0; i < origins.size(); ++i) {
        // sacar color
        const auto [red, green, blue] = colorForRobot(static_cast<int>(i));

        Config robotPose = origins[i];
        if (robotPose.radius <= 0.0f)
            robotPose.radius = radius;
        // circulo  alrededor del robot
        drawCircle(robotPose, robotPose.radius * 2.0f,
                   {0.0f, 1.0f, 0.0f, 0.25f},
                   {0.0f, 0.45f, 0.0f, 0.5f}, kMarkerSegments);
        drawRobot(robotPose, {red, green, blue, 1.0f}, true);
    }
}

} // namespace auxDraw

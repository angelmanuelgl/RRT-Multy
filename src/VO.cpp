#include "VO.h"

#include <algorithm>
#include <vector>

namespace {

double cross(const Point2D& o, const Point2D& a, const Point2D& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

} // namespace

PolygonObstacle minkowskiSum(const PolygonObstacle& a,
                             const PolygonObstacle& b) {
    std::vector<Point2D> sums;
    sums.reserve(a.vertices.size() * b.vertices.size());
    for (const auto& pa : a.vertices)
        for (const auto& pb : b.vertices)
            sums.push_back({pa.x + pb.x, pa.y + pb.y});

    std::sort(sums.begin(), sums.end(), [](const Point2D& p, const Point2D& q) {
        return p.x < q.x || (p.x == q.x && p.y < q.y);
    });
    sums.erase(std::unique(sums.begin(), sums.end(), [](const Point2D& p, const Point2D& q) {
        return p.x == q.x && p.y == q.y;
    }), sums.end());

    PolygonObstacle result;
    result.name = a.name + "_mas_" + b.name;
    if (sums.size() < 3) {
        result.vertices = std::move(sums);
        return result;
    }

    std::vector<Point2D> hull;
    for (const auto& point : sums) {
        while (hull.size() >= 2
               && cross(hull[hull.size() - 2], hull.back(), point) <= 0.0)
            hull.pop_back();
        hull.push_back(point);
    }
    const std::size_t lowerSize = hull.size();
    for (auto it = sums.rbegin() + 1; it != sums.rend(); ++it) {
        while (hull.size() > lowerSize
               && cross(hull[hull.size() - 2], hull.back(), *it) <= 0.0)
            hull.pop_back();
        hull.push_back(*it);
    }
    hull.pop_back();
    result.vertices = std::move(hull);
    return result;
}

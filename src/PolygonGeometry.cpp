#include "PolygonGeometry.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace {

constexpr double lengthEps = 1e-9;
constexpr double areaEps = 1e-9;

double cross(const Point2D& a,
             const Point2D& b,
             const Point2D& c)
{
    return (b.x - a.x) * (c.y - a.y)
    - (b.y - a.y) * (c.x - a.x);
}

bool samePoint(const Point2D& a, const Point2D& b)
{
    return std::hypot(a.x - b.x, a.y - b.y) <= lengthEps;
}

int sign(double value)
{
    if (value > areaEps) return 1;
    if (value < -areaEps) return -1;
    return 0;
}

bool onSegment(const Point2D& a,
               const Point2D& b,
               const Point2D& p)
{
    return std::abs(cross(a, b, p)) <= areaEps
           && p.x >= std::min(a.x, b.x) - lengthEps
           && p.x <= std::max(a.x, b.x) + lengthEps
           && p.y >= std::min(a.y, b.y) - lengthEps
           && p.y <= std::max(a.y, b.y) + lengthEps;
}

bool segmentsIntersect(const Point2D& a,
                       const Point2D& b,
                       const Point2D& c,
                       const Point2D& d)
{
    const int s1 = sign(cross(a, b, c));
    const int s2 = sign(cross(a, b, d));
    const int s3 = sign(cross(c, d, a));
    const int s4 = sign(cross(c, d, b));

    if (s1 * s2 < 0 && s3 * s4 < 0)
        return true;

    return (s1 == 0 && onSegment(a, b, c))
           || (s2 == 0 && onSegment(a, b, d))
           || (s3 == 0 && onSegment(c, d, a))
           || (s4 == 0 && onSegment(c, d, b));
}

// tirnauglo orientado antihorariamente.
// Los puntos sobre el borde también cuentan como contenidos.
bool insideTriangle(const Point2D& p,
                    const Point2D& a,
                    const Point2D& b,
                    const Point2D& c)
{
    return cross(a, b, p) >= -areaEps
           && cross(b, c, p) >= -areaEps
           && cross(c, a, p) >= -areaEps;
}

} // namespace

void preparePolygon(PolygonObstacle& polygon)
{
    // trabajamos sobre copia: no modificar el objeto por si falla
    auto vertices = polygon.vertices;
    const std::size_t n = vertices.size();

    // CASOS DEGENERADOS

    if (n < 3)
        throw std::invalid_argument("Se requieren al menos 3 vertices");

     for( const auto& p : vertices  ){
        if (!std::isfinite(p.x) || !std::isfinite(p.y))
            throw std::invalid_argument("Coordenada no finita");
    }

     for( std::size_t i = 0; i < n; ++i  ){
         for( std::size_t j = i + 1; j < n; ++j  ){
            if (samePoint(vertices[i], vertices[j]))
                throw std::invalid_argument("Vertices repetidos");
        }

        const auto& a = vertices[(i + n - 1) % n];
        const auto& b = vertices[i];
        const auto& c = vertices[(i + 1) % n];

        if (std::abs(cross(a, b, c)) <= areaEps  ){
            throw std::invalid_argument(
                "Vertices consecutivos colineales: se selimina el redundante");
        }
    }

    // comparar aristas no adyacentes
     for( std::size_t i = 0; i < n; ++i  ){
        const std::size_t iNext = (i + 1) % n;

         for( std::size_t j = i + 1; j < n; ++j  ){
            const std::size_t jNext = (j + 1) % n;

            if (iNext == j || jNext == i)
                continue;

            if (segmentsIntersect(vertices[i], vertices[iNext],
                                  vertices[j], vertices[jNext])  ){
                throw std::invalid_argument(
                    "Poligono con autointersecciones");
            }
        }
    }

    double twiceArea = 0.0;
     for( std::size_t i = 0; i < n; i++ ){
        const auto& a = vertices[i];
        const auto& b = vertices[(i + 1) % n];
        twiceArea += a.x * b.y - a.y * b.x;
    }

    if (std::abs(twiceArea) <= areaEps)
        throw std::invalid_argument("Poligono con area cero");

    if (twiceArea < 0.0)
        std::reverse(vertices.begin(), vertices.end());

    // AHORA SI

    std::vector<std::size_t> indices(n);
    std::iota(indices.begin(), indices.end(), std::size_t{0});

    std::vector<Triangle2D> triangles;
    triangles.reserve(n - 2);

    while( indices.size() > 3  ){
        bool clipped = false;
        const std::size_t count = indices.size();

         for( std::size_t k = 0; k < count; ++k  ){
            const auto ia = indices[(k + count - 1) % count];
            const auto ib = indices[k];
            const auto ic = indices[(k + 1) % count];

            const auto& a = vertices[ia];
            const auto& b = vertices[ib];
            const auto& c = vertices[ic];

            if( cross(a, b, c) <= areaEps )
                continue;

            bool containsVertex = false;

             for( const auto index : indices  ){
                if (index == ia || index == ib || index == ic)
                    continue;

                if (insideTriangle(vertices[index], a, b, c)  ){
                    containsVertex = true;
                    break;
                }
            }

            if (containsVertex)
                continue;

            triangles.push_back(Triangle2D{a, b, c});
            indices.erase(indices.begin() + k);
            clipped = true;
            break;
        }

        if (!clipped  ){
            throw std::invalid_argument(
                "Triangulacion fallida: geometria degenerada "
                "o precision insuficiente");
        }
    }

    const auto& a = vertices[indices[0]];
    const auto& b = vertices[indices[1]];
    const auto& c = vertices[indices[2]];

    if (cross(a, b, c) <= areaEps)
        throw std::invalid_argument("Triangulo final degenerado");

    triangles.push_back(Triangle2D{a, b, c});

    polygon.vertices = std::move(vertices);
    polygon.triangles = std::move(triangles);
}
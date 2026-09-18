#include "PQPCollision.h"
#include "PolygonGeometry.h"

// TODO
#include "PQP.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

constexpr double pi = 3.14159265358979323846;
constexpr int circleSegments = 32;

void checkPQP(int code, const char* operation)
{
    if (code != PQP_OK) {
        throw std::runtime_error(
            std::string(operation) + ": error PQP "
            + std::to_string(code));
    }
}

void identity(PQP_REAL R[3][3])
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = (i == j) ? PQP_REAL(1) : PQP_REAL(0);
}

std::unique_ptr<PQP_Model> makeDisk(double radius)
{
    auto model = std::make_unique<PQP_Model>();
    checkPQP(model->BeginModel(circleSegments), "BeginModel(robot)");

    // Polígono circunscrito al disco.
    const double outerRadius = radius / std::cos(pi / circleSegments);

    PQP_REAL center[3] = {0, 0, 0};

    for (int i = 0; i < circleSegments; ++i) {
        const double a = 2.0 * pi * i / circleSegments;
        const double b = 2.0 * pi * (i + 1) / circleSegments;

        PQP_REAL p[3] = {
            static_cast<PQP_REAL>(outerRadius * std::cos(a)),
            static_cast<PQP_REAL>(outerRadius * std::sin(a)),
            0
        };

        PQP_REAL q[3] = {
            static_cast<PQP_REAL>(outerRadius * std::cos(b)),
            static_cast<PQP_REAL>(outerRadius * std::sin(b)),
            0
        };

        checkPQP(model->AddTri(center, p, q, i), "AddTri(robot)");
    }

    checkPQP(model->EndModel(), "EndModel(robot)");
    return model;
}

std::unique_ptr<PQP_Model> makeObstacle(
    const PolygonObstacle& input)
{
    // No confiar en una triangulación suministrada por otro llamador.
    auto polygon = input;
    preparePolygon(polygon);

    if (polygon.triangles.size()
        > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("Demasiados triangulos");
    }

    auto model = std::make_unique<PQP_Model>();

    checkPQP(
        model->BeginModel(static_cast<int>(polygon.triangles.size())),
        "BeginModel(obstaculo)");

    int id = 0;

    for (const auto& triangle : polygon.triangles) {
        PQP_REAL points[3][3];

        for (int k = 0; k < 3; ++k) {
            points[k][0] = static_cast<PQP_REAL>(triangle[k].x);
            points[k][1] = static_cast<PQP_REAL>(triangle[k].y);
            points[k][2] = 0;
        }

        checkPQP(
            model->AddTri(points[0], points[1], points[2], id++),
            "AddTri(obstaculo)");
    }

    checkPQP(model->EndModel(), "EndModel(obstaculo)");
    return model;
}

bool finiteConfig(const Config& q)
{
    return std::isfinite(q.x)
    && std::isfinite(q.y)
        && std::isfinite(q.theta);
}

} // namespace

struct PQPCollisionChecker::Impl {
    std::unique_ptr<PQP_Model> robot = makeDisk(5.0);
    std::vector<std::unique_ptr<PQP_Model>> obstacles;
    mutable Statistics stats;

    bool collides(PQP_Model* a, double ax, double ay,
                  PQP_Model* b, double bx, double by) const
    {
        PQP_REAL Ra[3][3], Rb[3][3];
        identity(Ra);
        identity(Rb);

        PQP_REAL Ta[3] = {
            static_cast<PQP_REAL>(ax),
            static_cast<PQP_REAL>(ay), 0
        };

        PQP_REAL Tb[3] = {
            static_cast<PQP_REAL>(bx),
            static_cast<PQP_REAL>(by), 0
        };

        PQP_CollideResult result;

        checkPQP(
            PQP_Collide(&result,
                        Ra, Ta, a,
                        Rb, Tb, b,
                        PQP_FIRST_CONTACT),
            "PQP_Collide");

        ++stats.queries;
        stats.bvTests += result.NumBVTests();
        stats.triangleTests += result.NumTriTests();

        return result.Colliding() != 0;
    }
};

PQPCollisionChecker::PQPCollisionChecker()
    : impl_(std::make_unique<Impl>())
{
}

PQPCollisionChecker::~PQPCollisionChecker() = default;

void PQPCollisionChecker::setRobotRadius(double radius)
{
    if (!std::isfinite(radius) || radius <= 0.0)
        throw std::invalid_argument("Radio invalido");

    auto replacement = makeDisk(radius);
    impl_->robot = std::move(replacement);
}

void PQPCollisionChecker::setObstacles(
    const std::vector<PolygonObstacle>& obstacles)
{
    std::vector<std::unique_ptr<PQP_Model>> replacement;
    replacement.reserve(obstacles.size());

    for (const auto& obstacle : obstacles)
        replacement.push_back(makeObstacle(obstacle));

    impl_->obstacles = std::move(replacement);
}

bool PQPCollisionChecker::configurationInCollision(
    const std::vector<Config>& q) const
{
    if (q.empty())
        throw std::invalid_argument("Configuracion vacia");

    for (const auto& robot : q) {
        if (!finiteConfig(robot))
            throw std::invalid_argument("Configuracion no finita");

        for (const auto& obstacle : impl_->obstacles) {
            if (impl_->collides(
                    impl_->robot.get(), robot.x, robot.y,
                    obstacle.get(), 0.0, 0.0)) {
                return true;
            }
        }
    }

    // Colisiones robot-robot. Quita este bloque si deseas
    // habilitar únicamente robot-obstáculo.
    for (std::size_t i = 0; i < q.size(); ++i) {
        for (std::size_t j = i + 1; j < q.size(); ++j) {
            if (impl_->collides(
                    impl_->robot.get(), q[i].x, q[i].y,
                    impl_->robot.get(), q[j].x, q[j].y)) {
                return true;
            }
        }
    }

    return false;
}

bool PQPCollisionChecker::edgeInCollision(
    const std::vector<Config>& from,
    const std::vector<Config>& to,
    double resolution) const
{
    if (from.empty() || from.size() != to.size())
        throw std::invalid_argument("Dimensiones de arista invalidas");

    if (!std::isfinite(resolution) || resolution <= 0.0)
        throw std::invalid_argument("Resolucion invalida");

    double maxDistance = 0.0;

    for (std::size_t i = 0; i < from.size(); ++i) {
        if (!finiteConfig(from[i]) || !finiteConfig(to[i]))
            throw std::invalid_argument("Extremo de arista no finito");

        const double dx = double(to[i].x) - from[i].x;
        const double dy = double(to[i].y) - from[i].y;
        maxDistance = std::max(maxDistance, std::hypot(dx, dy));
    }

    const double required = std::ceil(maxDistance / resolution);

    // Evitar bucles enormes por parámetros incorrectos.
    if (!std::isfinite(required) || required > 1000000.0)
        throw std::invalid_argument("Demasiadas muestras por arista");

    const int steps = std::max(1, static_cast<int>(required));
    std::vector<Config> sample(from.size());

    for (int k = 0; k <= steps; ++k) {
        const double t = double(k) / steps;

        for (std::size_t i = 0; i < from.size(); ++i) {
            sample[i].x = static_cast<float>(
                (1.0 - t) * from[i].x + t * to[i].x);

            sample[i].y = static_cast<float>(
                (1.0 - t) * from[i].y + t * to[i].y);

            // La orientación no afecta al disco usado aquí.
            sample[i].theta = from[i].theta;
        }

        if (configurationInCollision(sample))
            return true;
    }

    return false;
}

PQPCollisionChecker::Statistics
PQPCollisionChecker::statistics() const
{
    return impl_->stats;
}

void PQPCollisionChecker::resetStatistics()
{
    impl_->stats = {};
}
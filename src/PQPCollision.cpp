#include "PQPCollision.h"
#include "PolygonGeometry.h"
#include "logger.h"

// TODO
#include "PQP.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>



// AUXILIARES GENERALES QUE USAN PQQ
// Y NOS SERIVRAN MAS ADELATE CUANDO CHECEMOS COLISONES
namespace {

constexpr double pi = 3.14159265358979323846;
constexpr int circleSegments = 32;

// PARA DEBUG FACIL
void checkPQP(int code, const char* operation)
{
    if(  code != PQP_OK ){
        LOG_FATAL("Fallo interno de PQP en ", operation, ": codigo=", code);
        throw std::runtime_error(
            std::string(operation) + ": error PQP "
            + std::to_string(code));
    }
}

// I 3x3
void identity(PQP_REAL R[3][3])
{
    for( int i = 0; i < 3; ++i)
        for( int j = 0; j < 3; ++j)
            R[i][j] = (i == j) ? PQP_REAL(1) : PQP_REAL(0);
}


// PARA LOS ROBOT
// poligono circunscrito al disco (32 puntos_
std::unique_ptr<PQP_Model> makeDisk(double radius)
{
    LOG_DEBUG("Construyendo modelo PQP del robot: radio=", radius,
              ", segmentos=", circleSegments);
    auto model = std::make_unique<PQP_Model>();
    checkPQP(model->BeginModel(circleSegments), "BeginModel(robot)");


    const double radioExterior = radius / std::cos(pi / circleSegments);

    PQP_REAL center[3] = {0, 0, 0};

    for( int i = 0; i < circleSegments;  i++){
        const double a = 2.0 * pi * i / circleSegments;
        const double b = 2.0 * pi * (i + 1) / circleSegments;

        PQP_REAL p[3] = {
            static_cast<PQP_REAL>(radioExterior * std::cos(a)),
            static_cast<PQP_REAL>(radioExterior * std::sin(a)),
            0
        };

        PQP_REAL q[3] = {
            static_cast<PQP_REAL>(radioExterior * std::cos(b)),
            static_cast<PQP_REAL>(radioExterior * std::sin(b)),
            0
        };

        checkPQP(model->AddTri(center, p, q, i), "AddTri(robot)");
    }

    checkPQP(model->EndModel(), "EndModel(robot)");
    LOG_SUCCESS("Modelo PQP del robot construido: triangulos=", circleSegments);
    return model;
}

std::unique_ptr<PQP_Model> makeObstacle(
    const PolygonObstacle& input)
{
    LOG_DEBUG("Construyendo modelo PQP de obstaculo: nombre=", input.name,
              ", vertices=", input.vertices.size());
    // revisamos que el poligono si cumpla las condicones de poligono
    auto polygon = input;
    preparePolygon(polygon);

    // precaucion
    if(  polygon.triangles.size()
        > static_cast<std::size_t>(std::numeric_limits<int>::max()) ){
        throw std::invalid_argument("Demasiados triangulos");
    }

    auto model = std::make_unique<PQP_Model>();

    checkPQP(
        model->BeginModel(static_cast<int>(polygon.triangles.size())),
        "BeginModel(obstaculo)"  );

    int id = 0;

    for( const auto& triangle : polygon.triangles ){
        PQP_REAL points[3][3];

        for( int k = 0; k < 3; ++k ){
            points[k][0] = static_cast<PQP_REAL>(triangle[k].x);
            points[k][1] = static_cast<PQP_REAL>(triangle[k].y);
            points[k][2] = 0;
        }

        checkPQP(
            model->AddTri(points[0], points[1], points[2], id++),
            "AddTri(obstaculo)");
    }

    checkPQP(model->EndModel(), "EndModel(obstaculo)");
    LOG_DEBUG("Modelo PQP de obstaculo listo: nombre=", input.name,
              ", triangulos=", polygon.triangles.size());
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

        if (result.Colliding() != 0)
            LOG_TRACE("PQP detecto colision: A=(", ax, ",", ay,
                      "), B=(", bx, ",", by, ")");
        return result.Colliding() != 0;
    }
};


// LA ESTRCUTURA QUE NOS REVISARA COLISIONES

PQPCollisionChecker::PQPCollisionChecker()
    : impl_(std::make_unique<Impl>())
{
    LOG_SUCCESS("Verificador de colisiones PQP inicializado");
}

PQPCollisionChecker::~PQPCollisionChecker() = default;


// seters
void PQPCollisionChecker::setRobotRadius(double radius)
{
    if(  !std::isfinite(radius) || radius <= 0.0) {
        LOG_ERROR("Radio de robot invalido para PQP: ", radius);
        throw std::invalid_argument("Radio invalido");
    }

    auto replacement = makeDisk(radius);
    impl_->robot = std::move(replacement);
    LOG_SUCCESS("Radio del modelo de robot actualizado: ", radius);
}

void PQPCollisionChecker::setObstacles(
    const std::vector<PolygonObstacle>& obstacles)
{
    LOG_INFO("Preparando modelos PQP de obstaculos: total=", obstacles.size());
    std::vector<std::unique_ptr<PQP_Model>> replacement;
    replacement.reserve(obstacles.size());

    for( const auto& obstacle : obstacles)
        replacement.push_back(makeObstacle(obstacle));

    impl_->obstacles = std::move(replacement);
    LOG_SUCCESS("Modelos PQP de obstaculos configurados: total=",
                impl_->obstacles.size());
}


// para verificar si es un nodo valido
// es decir los robots en la configuracion de este nodo no chocan
// con obstaculos ni entre si
bool PQPCollisionChecker::configurationInCollision(
    const std::vector<Config>& q) const
{
    if(  q.empty()) {
        LOG_ERROR("No se puede comprobar una configuracion vacia");
        throw std::invalid_argument("Configuracion vacia");
    }

    // colisiones robot - obstaculo
    for( const auto& robot : q ){
        if(  !finiteConfig(robot)) {
            LOG_ERROR("Robot con estado no finito: (", robot.x, ",",
                      robot.y, ",", robot.theta, ")");
            throw std::invalid_argument("Configuracion no finita");
        }


        for( const auto& obstacle : impl_->obstacles ){
            if(  impl_->collides(
                    impl_->robot.get(), robot.x, robot.y,
                    obstacle.get(), 0.0, 0.0) ){
                LOG_TRACE("Colision robot-obstaculo: robot=(", robot.x,
                          ",", robot.y, ")");
                return true;
            }
        }
    }

    // colisiones robot-robot
    // para ver que robots no choquen tambien
    for( std::size_t i = 0; i < q.size();  i++){
        for( std::size_t j = i + 1; j < q.size(); ++j ){
            if(  impl_->collides(
                    impl_->robot.get(), q[i].x, q[i].y,
                    impl_->robot.get(), q[j].x, q[j].y) ){
                LOG_TRACE("Colision robot-robot: indices=", i, ",", j);
                return true;
            }
        }
    }

    return false;
}

// para ver que las aristas no vayan a chocar
// ToDo: SI YA NO SON OMNIDERECCIONALE HAY QUE HACER CAMBIOS AQUI
bool PQPCollisionChecker::edgeInCollision(
    const std::vector<Config>& from,
    const std::vector<Config>& to,
    double resolution) const
{
    if(  from.empty() || from.size() != to.size()) {
        LOG_ERROR("Arista con dimensiones invalidas: origen=", from.size(),
                  ", destino=", to.size());
        throw std::invalid_argument("Dimensiones de arista invalidas");
    }

    if(  !std::isfinite(resolution) || resolution <= 0.0) {
        LOG_ERROR("Resolucion de colision invalida: ", resolution);
        throw std::invalid_argument("Resolucion invalida");
    }

    double maxDistance = 0.0;

    for( std::size_t i = 0; i < from.size();  i++){
        if(  !finiteConfig(from[i]) || !finiteConfig(to[i])) {
            LOG_ERROR("Extremo no finito en arista para robot ", i);
            throw std::invalid_argument("Extremo de arista no finito");
        }

        const double dx = double(to[i].x) - from[i].x;
        const double dy = double(to[i].y) - from[i].y;
        maxDistance = std::max(maxDistance, std::hypot(dx, dy));
    }

    const double required = std::ceil(maxDistance / resolution);

    // para no hacer tantas muestras, y que sea tan tardadp
    if(  !std::isfinite(required) || required > 1000000.0) {
        LOG_ERROR("Muestreo de arista fuera de rango: muestras=", required,
                  ", resolucion=", resolution);
        throw std::invalid_argument("Demasiadas muestras por arista");
    }

    const int steps = std::max(1, static_cast<int>(required));
    std::vector<Config> sample(from.size());
    LOG_TRACE("Comprobando arista: robots=", from.size(),
              ", muestras=", steps + 1);

    for( int k = 0; k <= steps; ++k ){
        const double t = double(k) / steps;

        for( std::size_t i = 0; i < from.size();  i++){
            sample[i].x = static_cast<float>(
                (1.0 - t) * from[i].x + t * to[i].x);

            sample[i].y = static_cast<float>(
                (1.0 - t) * from[i].y + t * to[i].y);

            // La orientación no afecta al disco usado aquí.
            sample[i].theta = from[i].theta;
        }

        if(  configurationInCollision(sample)) {
            LOG_TRACE("Colision en arista: muestra=", k, "/", steps);
            return true;
        }
    }

    LOG_TRACE("Arista libre de colisiones: muestras=", steps + 1);
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
    LOG_DEBUG("Estadisticas de colision reiniciadas");
}

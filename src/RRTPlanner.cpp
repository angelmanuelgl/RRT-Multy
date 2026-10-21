#include "RRTPlanner.h"


#include "logger.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <utility>

namespace {

EdgeMotion makeEdgeMotion(const std::vector<Config>& start,
                          const std::vector<Config>& target,
                          float duration)
{
    EdgeMotion motion;
    MotionPhase phase;
    phase.duration = duration;
    phase.velocities.resize(start.size());
    for (std::size_t i = 0; i < start.size(); ++i) {
        const float deltaTheta = std::atan2(
            std::sin(target[i].theta - start[i].theta),
            std::cos(target[i].theta - start[i].theta));
        phase.velocities[i] = {
            (target[i].x - start[i].x) / duration,
            (target[i].y - start[i].y) / duration,
            deltaTheta / duration
        };
    }
    motion.phases.push_back(std::move(phase));
    return motion;
}

} // namespace

RRTPlanner::RRTPlanner(const RRTPlannerParameters& parameters)
    : parameters_(parameters),
      goalQ_({{1000.0f, 600.0f, 0.0f}}),
      originQ_({{200.0f, 140.0f, 0.0f}})
{
    if (!std::isfinite(parameters_.stepSize) || parameters_.stepSize <= 0.0f
        || !std::isfinite(parameters_.maxStepSize)
        || parameters_.maxStepSize < parameters_.stepSize
        || !std::isfinite(parameters_.goalTolerance)
        || parameters_.goalTolerance < 0.0f
        || !std::isfinite(parameters_.goalBias)
        || parameters_.goalBias < 0.0f || parameters_.goalBias > 1.0f
        || parameters_.maxNodes < 2 || parameters_.maxAttempts == 0
        || !std::isfinite(parameters_.workspaceWidth)
        || !std::isfinite(parameters_.workspaceHeight)
        || parameters_.workspaceWidth < parameters_.stepSize
        || parameters_.workspaceHeight < parameters_.stepSize
        || !std::isfinite(parameters_.minimumCollisionResolution)
        || parameters_.minimumCollisionResolution <= 0.0
        || !std::isfinite(parameters_.collisionResolutionRadiusFactor)
        || parameters_.collisionResolutionRadiusFactor <= 0.0) {
        parameters_ = RRTPlannerParameters{};
        LOG_WARN("Parametros RRT invalidos; se aplicaron los valores predeterminados");
    }

    std::srand(static_cast<unsigned>(std::time(nullptr)));//Semilla para generacion aleatoria

    //**opcional porque asi se inicializa con un robot********
    //verificar que la configuración de origen tenga al menos un robot
    if( originQ_.empty() ){
        originQ_.push_back({200.f, 140.f, 0.f});
    }
    //**opcional porque asi se inicializa con un robot********

    reset();
    LOG_SUCCESS("RRTPlanner inicializado con configuracion predeterminada");
}

void RRTPlanner::setStart(const std::vector<Config>& q, float radius)
{
    const float fallbackRadius = std::isfinite(radius) && radius > 0.0f
        ? radius : kDefaultRobotRadius;
    originQ_ = q;
    float smallestRobotRadius = fallbackRadius;
    for (auto& robot : originQ_) {
        if (!std::isfinite(robot.radius) || robot.radius <= 0.0f)
            robot.radius = fallbackRadius;
        smallestRobotRadius = std::min(smallestRobotRadius, robot.radius);
        collisionChecker_.setRobotRadius(robot.radius);
    }
    collisionCheckResolution_ = std::max(
        parameters_.minimumCollisionResolution,
        (smallestRobotRadius + kCollisionSafetyMargin)
            * parameters_.collisionResolutionRadiusFactor);
    if (goalQ_.size() == originQ_.size()) {
        for (std::size_t i = 0; i < goalQ_.size(); ++i)
            goalQ_[i].radius = originQ_[i].radius;
    }
    numRobots_ = static_cast<int>(originQ_.size());
    reset();
    LOG_INFO("Origen RRT configurado: robots=", numRobots_,
             ", radio_fallback=", fallbackRadius,
             ", holgura_colision=", kCollisionSafetyMargin,
             ", resolucion_colision=", collisionCheckResolution_);
    for (std::size_t i = 0; i < originQ_.size(); ++i)
        LOG_DEBUG("Origen robot ", i, "=(", originQ_[i].x, ",",
                  originQ_[i].y, ",", originQ_[i].theta, ")");
}

void RRTPlanner::setGoal(const std::vector<Config>& q, float radius)
{
    const float fallbackRadius = std::isfinite(radius) && radius > 0.0f
        ? radius : kDefaultRobotRadius;
    goalQ_ = q;
    for (std::size_t i = 0; i < goalQ_.size(); ++i) {
        if (i < originQ_.size()) {
            // El radio pertenece al robot i y se conserva al cambiar su pose.
            goalQ_[i].radius = originQ_[i].radius;
        } else if (!std::isfinite(goalQ_[i].radius)
                   || goalQ_[i].radius <= 0.0f) {
            goalQ_[i].radius = fallbackRadius;
        }
    }
    numRobots_ = static_cast<int>(goalQ_.size());
    reset(); // AMGL //
    LOG_INFO("Meta RRT configurada: robots=", numRobots_);
    for (std::size_t i = 0; i < goalQ_.size(); ++i)
        LOG_DEBUG("Meta robot ", i, "=(", goalQ_[i].x, ",",
                  goalQ_[i].y, ",", goalQ_[i].theta, ")");
}

void RRTPlanner::setNumRobots(int n)
{
    if( n > 0 ){
        numRobots_ = n;
        LOG_DEBUG("Numero de robots configurado: ", numRobots_);
    } else {
        LOG_WARN("Se ignoro un numero de robots no positivo: ", n);
    }
}

void RRTPlanner::setStepSize(float step)
{
    if (!std::isfinite(step) || step <= 0.0f
        || step > parameters_.maxStepSize) {
        LOG_WARN("Se ignoro un paso RRT invalido: ", step,
                 ", maximo=", parameters_.maxStepSize);
        return;
    }
    parameters_.stepSize = step;
    LOG_DEBUG("Paso del RRT configurado: ", parameters_.stepSize);
}

float RRTPlanner::getStepSize() const
{
    return parameters_.stepSize;
}

void RRTPlanner::setMaxNodes(int maxNodes)
{
    if (maxNodes < 2) {
        LOG_WARN("Se ignoro un limite de nodos invalido: ", maxNodes);
        return;
    }
    parameters_.maxNodes = maxNodes;
    LOG_DEBUG("Limite de nodos configurado: ", parameters_.maxNodes);
}

int RRTPlanner::getMaxNodes() const
{
    return parameters_.maxNodes;
}

void RRTPlanner::setGoalTolerance(float distance)
{
    if (!std::isfinite(distance) || distance < 0.0f) {
        LOG_WARN("Se ignoro un margen de llegada invalido: ", distance);
        return;
    }
    parameters_.goalTolerance = distance;
    LOG_DEBUG("Margen posicional de llegada configurado: ",
              parameters_.goalTolerance);
}

float RRTPlanner::getGoalTolerance() const
{
    return parameters_.goalTolerance;
}

void RRTPlanner::reset()
{
    tree_.clear();
    finalPath_.clear();
    nNodesPath_ = 0;
    nNodesRRT_ = 0;
    attempts_ = 0;
    problemChecked_ = false;
    done_ = false;

    if( !originQ_.empty() ){
        tree_.emplace_back(originQ_, -1);//insertamos el nodo inicial al origen del arbol
        //tree_.emplace_back(originQ_, -1);//Agregamos el nodo de configuracion inicial de los robots al arbol
    }
    LOG_DEBUG("RRT reiniciado: nodos=", tree_.size(),
              ", robots=", originQ_.size());
}


/* --- --- ACTUALMENTE HACEMOS: --- ---

    q_rand <- RANDOM_STATE()
    q_nea <-  NEAREST_NEIGHBOR(q_rand, T)
    u <-  SELECT_INPUT_RECTA(q_rand, q_near)
    q_new <-  FINAL_DE_LA_RECTA(q_near, u, \delta t)

    T.ADD_VERTEX(q_new)
    T.ADD_EDGE(q_near, q_new, u)
*/


/* --- --- POSIBLE PASO INTERMEDIO: --- --- 

    qNear: RRT-Multy propone una meta local con su recta
                        |
                        V
    ORCA intenta conectar qNear con esa meta sin colisiones
                        |
                        V
    Euler simula esa conexion \delta t
                        |
                        V
    el estado realmente alcanzado es qNew
                        |
                        V
    RRT agrega qNew y guarda la trayectoria ORCA


    PSUDOCODIGO:
    q_rand <- RANDOM_STATE()
    q_near <- NEAREST_NEIGHBOR(q_rand, T)

    u <- ORCA(q_near, dirección hacia q_rand)
    q_new <- EULER(q_near, u, \delta t)

    T.ADD_VERTEX(q_new)
    T.ADD_EDGE(q_near, q_new, u
*/


/* --- --- META: --- --- 
    q_rand <- RANDOM_STATE()
    q_near <- NEAREST_NEIGHBOR(q_rand, T)

    estado <- q_near
    trayectoria <- [estado]

    repetir durante T:

        velocidades_preferidas <- DIRECCION_HACIA(q_rand, estado)

        velocidades_seguras <- ORCA(
            estado,
            velocidades_preferidas,
            vecinos,
            obstaculos
        )

        estado <- EULER(
            estado,
            velocidades_seguras,
            delta_t
        )

        trayectoria.agregar(estado)

    q_new <- estado

    si la trayectoria es valida:
        T.ADD_VERTEX(q_new)
        T.ADD_EDGE(q_near, q_new, trayectoria)

*/

/*
    step(): devuelve true si existe sol
    isDone(): devuelve true si termino, con solucion o por fallo
*/
bool RRTPlanner::step()
{
    if( done_ ){
        LOG_TRACE("Paso RRT omitido: planificador finalizado, ruta=",
                  !finalPath_.empty());
        return !finalPath_.empty(); // AMGL//
        return true;
    }


    // --- --- PARA DEBUG MAS CLARO --- ---
    // AMGL // auxiliar en caso de error
    auto fallo = [this](const char* message ){
        LOG_ERROR(message);
        finalPath_.clear();
        nNodesPath_ = 0;
        done_ = true;
        return false;
    };

    if( !problemChecked_ ){

        // comprovaciones anteriores
        if( numRobots_ <= 0 || tree_.empty()
            || originQ_.size() != static_cast<std::size_t>(numRobots_)
            || goalQ_.size() != static_cast<std::size_t>(numRobots_) ){

            return fallo("Cantidades de Robots en inicio y final no coiciden");
        }

        // comprovacion para mas seguridad
        if( !std::isfinite(parameters_.stepSize)
            || parameters_.stepSize <= 0.0f || parameters_.stepSize > parameters_.maxStepSize
            || !std::isfinite(parameters_.goalTolerance) || parameters_.goalTolerance < 0.0f
            || parameters_.maxNodes < 2 ){
            return fallo("Parametros del RRT invalidos");
        }

        if( !isConfigurationValid(originQ_))
            return fallo("NO hay configuracion inicial en colision");

        if( !isConfigurationValid(goalQ_))
            return fallo("NO hay configuracion final en colision");

        problemChecked_ = true;
        LOG_INFO("Inicio de planificacion RRT: robots=", numRobots_,
                 ", paso=", parameters_.stepSize, ", max_nodos=", parameters_.maxNodes,
                 ", margen_meta_por_robot=", parameters_.goalTolerance);

        // AMGL // criterio de llegada: el origen puede empezar dentro de la region aceptada.
        if (isWithinGoalTolerance(originQ_)) {
            finalPath_ = {0};
            nNodesPath_ = 1;
            done_ = true;
            LOG_SUCCESS("Inicio ya dentro de las regiones de meta");
            return true;
        }
    }

    if( tree_.size() >= static_cast<std::size_t>(parameters_.maxNodes))
        return fallo("Limite de nodos alcanzado sin solucion");

    if( attempts_ >= parameters_.maxAttempts)
        return fallo("Limite de intentos alcanzado sin solucion");


    // AMGL// esto es lo que estaba anteriormente //ya no deberia ser util
    // if( numRobots_ <= 0 || tree_.empty() || goalQ_.empty()
    //     || static_cast<int>(tree_.size()) >= parameters_.maxNodes ){
    //     return false;
    // }

    // if( static_cast<int>(goalQ_.size()) < numRobots_
    //     || static_cast<int>(tree_.front().q.size()) < numRobots_ ){
    //     return false;
    // }


    // Contar también las muestras que después se rechazan por colisión.
    ++attempts_;

    // --- --- --- ---  q_rand <- RANDOM_STATE() --- --- --- ---
    // AMGL // Dirección preferencial según parameters_.goalBias
    // como muestra en el 10% de los intentos; el resto sigue siendo aleatorio.
    const bool sampleGoal = randFloat(0.0f, 1.0f) < parameters_.goalBias;
    std::vector<Config> qRand(numRobots_);
    if (sampleGoal) {
        qRand = goalQ_; // AMGL // tomar la configuracion final conjunta como objetivo
        LOG_TRACE("Muestra dirigida a la meta (intento ", attempts_, ")");
    } else {
        for (int i = 0; i < numRobots_; ++i) {
            qRand[i].x = randFloat(parameters_.stepSize, parameters_.workspaceWidth); //width());
            qRand[i].y = randFloat(parameters_.stepSize, parameters_.workspaceHeight);// height());
            qRand[i].theta = 0.0f; // el criterio de llegada es solo posicional
            qRand[i].radius = i < static_cast<int>(originQ_.size())
                ? originQ_[i].radius : kDefaultRobotRadius;
        }
    }
    if (!qRand.empty())
        LOG_TRACE("q_rand: robot0=(", qRand[0].x, ",", qRand[0].y,
                  "), robots=", numRobots_);

    // EAR
    //encontramos el nodo mas cercano del arbol a qrand
    const int nearest = getNearest(qRand);
    const auto& qNear = tree_[nearest].q;
    LOG_TRACE("q_near: indice=", nearest, ", nodos=", tree_.size());

    //*****************13/11/2025*******************************************************
    //Steer: mover de q_near hacia q_rand con paso stepSize
    const auto qNew = steer(qNear, qRand, parameters_.stepSize);
    if (!qNew.empty())
        LOG_TRACE("q_new: robot0=(", qNew[0].x, ",", qNew[0].y,
                  "), robots=", qNew.size());


    // realizamos las comprobacio antes de insertar qNew
    if (!isEdgeValid(qNear, qNew)) {
        LOG_TRACE("Arista RRT rechazada por colision: padre=", nearest);
        return false;
    }

    //Insertar nuevo nodo
    const int newIndex = static_cast<int>(tree_.size());
    constexpr float edgeDuration = 1.0f;
    const EdgeMotion motion = makeEdgeMotion(qNear, qNew, edgeDuration);
    tree_.emplace_back(qNew, nearest, motion);
    ++nNodesRRT_;
    LOG_TRACE("Vertice agregado: indice=", newIndex,
              ", padre=", nearest, ", total=", tree_.size(),
              ", duracion_arista=", edgeDuration);

    if (!isWithinGoalTolerance(qNew))
        return false;

    LOG_DEBUG("Todos los robots dentro de sus regiones de llegada: indice=",
              newIndex, ", radio_aceptacion_por_robot=radio_robot+", parameters_.goalTolerance);

    // AMGL // la region es la meta; no se exige una arista extra hasta los centros exactos.
    finalPath_.clear();
    nNodesPath_ = 0;
    int index = newIndex;
    while (index != -1 ){
        finalPath_.push_back(index);
        index = tree_[index].parent;
        ++nNodesPath_;
    }
    std::reverse(finalPath_.begin(), finalPath_.end());

    done_ = true;
    const auto stats = collisionChecker_.statistics();
    LOG_SUCCESS("Ruta RRT encontrada dentro de las regiones de meta: nodos_arbol=", tree_.size(),
                ", nodos_ruta=", finalPath_.size(),
                ", consultas_colision=", stats.queries);
    return true;

     // AMGL// esto es lo que estaba anteriormente //ya no deberia ser util
    // if( !qNew.empty() ){
    //     const float distanceToGoal = configDistance(qNew, goalQ_);//Calculamos la distancia des qrand a la meta
    //     if( distanceToGoal <= parameters_.goalTolerance ){
    //         // Conectamos exactamente al goal del robot 0
    //         tree_.emplace_back(goalQ_, static_cast<int>(tree_.size()) - 1);//conectamos la meta con el ultimo nodo agregado

    //         // Backtracking del camino
    //         finalPath_.clear();
    //         nNodesPath_ = 0;
    //         int index = static_cast<int>(tree_.size()) - 1;
    //         while (index != -1 ){
    //             finalPath_.push_back(index);
    //             index = tree_[index].parent;
    //             ++nNodesPath_;
    //         }
    //         std::reverse(finalPath_.begin(), finalPath_.end());

    //         //TimerStop=1;
    //         done_ = true;
    //         return true;
    //     }
    // }

    // //update();//refrescamos pantalla
    // return false;
}

bool RRTPlanner::isDone() const
{
    return done_;
}

const std::vector<PlanNode>& RRTPlanner::getTree() const
{
    return tree_;
}

const std::vector<int>& RRTPlanner::getPath() const
{
    return finalPath_;
}

const std::vector<Config>& RRTPlanner::getGoal() const
{
    return goalQ_;
}

const std::vector<Config>& RRTPlanner::getOrigin() const
{
    return originQ_;
}

int RRTPlanner::getNumRobots() const
{
    return numRobots_;
}

int RRTPlanner::getNodeCount() const
{
    return nNodesRRT_;
}

int RRTPlanner::getPathNodeCount() const
{
    return nNodesPath_;
}

//devuelve un valor aleatorio en el rango min-max
float RRTPlanner::randFloat(float min, float max)
{
    return min + (max - min)
        * (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX));
    //return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

//+++++++++++++++++++++++++++++++12/11/2025+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

float RRTPlanner::configDistance(const std::vector<Config>& a,
                                 const std::vector<Config>& b)
{
    float sum = 0.0f;
    for (int i = 0; i < numRobots_; ++i ){
        const float dx = a[i].x - b[i].x;
        const float dy = a[i].y - b[i].y;

        //Intercambiar sum y descomentar float dth para contemplar orientacion
        //float dth= atan2(sin(b[i].theta-a[i].theta),cos(b[i].theta-a[i].theta));
        //sum += dx * dx + dy * dy + wth * dth * dth;
        sum += dx * dx + dy * dy;
    }

    return std::sqrt(sum);
}

std::vector<Config> RRTPlanner::steer(const std::vector<Config>& a,
                                      const std::vector<Config>& b,
                                      float step)
{
    std::vector<Config> qNew = a;
    const float distance = configDistance(a, b);
    //float factor =stepSize / dist; //(dist > stepSize) ? (stepSize / dist) : 1.0f;
    const float factor = (distance > step) ? (step / distance) : 1.0f;

    for (int i = 0; i < numRobots_; ++i ){
        qNew[i].x = a[i].x + factor * (b[i].x - a[i].x);//     dx[i] = b[i].x - a[i].x
        qNew[i].y = a[i].y + factor * (b[i].y - a[i].y);//     dy[i] = b[i].y - a[i].y
        //descomentar para contemlplar orientación, tener en cuebta que //dth[i]= atan2(sin(b[i].theta-a[i].theta),cos(b[i].theta-a[i].theta));
        //qNew[i].theta = a[i].theta + step * dth[i] / distance;
    }

    return qNew;
}

//Se recorre el arbol para encontrar el indice del nodo mas cercano
//con base en la distancia euclidiana.
int RRTPlanner::getNearest(const std::vector<Config>& qRand)
{
    int index = 0;
    float minDistance = 1e9f;

    for (int i = 0; i < static_cast<int>(tree_.size()); ++i ){
        const float distance = configDistance(qRand, tree_[i].q);//Calculamos la distancia de qrand a cada configuración

        if( distance < minDistance ){
            minDistance = distance;
            index = i;
        }
    }

    LOG_TRACE("Vecino mas cercano seleccionado: indice=", index,
              ", distancia=", minDistance);
    return index;
}


// OBSTACULOS

void RRTPlanner::setObstacles(
    const std::vector<PolygonObstacle>& obstacles)
{
    LOG_INFO("Configurando obstaculos del RRT: total=", obstacles.size());
    collisionChecker_.setObstacles(obstacles);
    reset();
    LOG_SUCCESS("Obstaculos del RRT configurados: total=", obstacles.size());
}

bool RRTPlanner::isConfigurationValid(
    const std::vector<Config>& q) const
{
    // return true;
    if( q.size() != static_cast<std::size_t>(numRobots_)) {
        LOG_ERROR("Configuracion con dimension invalida: recibidos=", q.size(),
                  ", esperados=", numRobots_);
        return false;
    }
    for (std::size_t i = 0; i < q.size(); ++i) {
        if (!std::isfinite(q[i].x) || !std::isfinite(q[i].y)
            || !std::isfinite(q[i].radius) || q[i].radius <= 0.0f) {
            LOG_ERROR("Pose o radio invalido para robot ", i,
                      ": x=", q[i].x, ", y=", q[i].y,
                      ", radio=", q[i].radius);
            return false;
        }
    }

    const bool valid = !collisionChecker_.configurationInCollision(q);
    LOG_TRACE("Validacion de configuracion: robots=", q.size(),
              ", valida=", valid);
    return valid;
}

bool RRTPlanner::isWithinGoalTolerance(const std::vector<Config>& q) const
{
    if (q.size() != goalQ_.size())
        return false;

    for (std::size_t i = 0; i < q.size(); ++i) {
        const float dx = q[i].x - goalQ_[i].x;
        const float dy = q[i].y - goalQ_[i].y;
        const float acceptedCenterDistance = goalAcceptanceRadius(goalQ_[i], parameters_.goalTolerance);
        if (!std::isfinite(dx) || !std::isfinite(dy)
            || !std::isfinite(acceptedCenterDistance)
            || std::hypot(dx, dy) > acceptedCenterDistance)
            return false;
    }
    return true;
}

bool RRTPlanner::isEdgeValid(
    const std::vector<Config>& from,
    const std::vector<Config>& to) const
{
    // return true;
    const bool valid = !collisionChecker_.edgeInCollision(
        from, to, collisionCheckResolution_);
    LOG_TRACE("Validacion de arista: robots=", from.size(),
              ", resolucion=", collisionCheckResolution_,
              ", valida=", valid);
    return valid;
}

PQPCollisionChecker::Statistics
RRTPlanner::getCollisionStatistics() const
{
    return collisionChecker_.statistics();
}

#include "RRTPlanner.h"


#include "logger.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>

RRTPlanner::RRTPlanner()
    : goalQ_({{1000.0f, 600.0f, 0.0f}}),
      originQ_({{200.0f, 140.0f, 0.0f}})
{
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
    collisionChecker_.setRobotRadius(radius); // AMGL //
    collisionCheckResolution_ = radius / 2.0;
    originQ_ = q;
    originRadius_ = radius;
    numRobots_ = static_cast<int>(originQ_.size());
    reset();
    LOG_INFO("Origen RRT configurado: robots=", numRobots_,
             ", radio=", radius,
             ", resolucion_colision=", collisionCheckResolution_);
    for (std::size_t i = 0; i < originQ_.size(); ++i)
        LOG_DEBUG("Origen robot ", i, "=(", originQ_[i].x, ",",
                  originQ_[i].y, ",", originQ_[i].theta, ")");
}

void RRTPlanner::setGoal(const std::vector<Config>& q, float radius)
{
    goalQ_ = q;
    goalRadius_ = radius;
    numRobots_ = static_cast<int>(goalQ_.size());
    reset(); // AMGL //
    LOG_INFO("Meta RRT configurada: robots=", numRobots_, ", radio=", radius);
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
    stepSize_ = step;
    LOG_DEBUG("Paso del RRT configurado: ", stepSize_);
}

void RRTPlanner::setMaxNodes(int maxNodes)
{
    maxNodes_ = maxNodes;
    LOG_DEBUG("Limite de nodos configurado: ", maxNodes_);
}

void RRTPlanner::setGoalTolerance(float distance)
{
    distToGoal_ = distance;
    LOG_DEBUG("Tolerancia de llegada configurada: ", distToGoal_);
}

void RRTPlanner::reset()
{
    tree_.clear();
    finalPath_.clear();
    nNodesPath_ = 0;
    nNodesRRT_ = 0;
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
        if( !std::isfinite(stepSize_)
            || stepSize_ <= 0.0f || stepSize_ > 500.0f
            || !std::isfinite(distToGoal_) || distToGoal_ < 0.0f
            || maxNodes_ < 2 ){
            return fallo("Parametros del RRT invalidos");
        }

        if( !isConfigurationValid(originQ_))
            return fallo("NO hay configuracion inicial en colision");

        if( !isConfigurationValid(goalQ_))
            return fallo("NO hay configuracion final en colision");

        problemChecked_ = true;
        LOG_INFO("Inicio de planificacion RRT: robots=", numRobots_,
                 ", paso=", stepSize_, ", max_nodos=", maxNodes_,
                 ", tolerancia_meta=", distToGoal_);
    }

    if( tree_.size() >= static_cast<std::size_t>(maxNodes_))
        return fallo("Limite de nodos alcanzado sin solucion");

    if( attempts_ >= maxAttempts_)
        return fallo("Limite de intentos alcanzado sin solucion");


    // AMGL// esto es lo que estaba anteriormente //ya no deberia ser util
    // if( numRobots_ <= 0 || tree_.empty() || goalQ_.empty()
    //     || static_cast<int>(tree_.size()) >= maxNodes_ ){
    //     return false;
    // }

    // if( static_cast<int>(goalQ_.size()) < numRobots_
    //     || static_cast<int>(tree_.front().q.size()) < numRobots_ ){
    //     return false;
    // }


    //generamos un conf aleatoria de cada robot
    std::vector<Config> qRand(numRobots_);
    for (int i = 0; i < numRobots_; ++i ){
        qRand[i].x = randFloat(stepSize_, 500.0f); //width());
        qRand[i].y = randFloat(stepSize_, 500.0f);// height());
        //descomentar para contemlplar orientación
        //qRand[i].theta = randFloat(-M_PI, M_PI); // por ahora de 0 a PI rad
        if (i == 0)
            LOG_TRACE("q_rand: robot0=(", qRand[i].x, ",", qRand[i].y,
                      "), robots=", numRobots_);
    }

    //encontramos el nodo mas cercano del arbol a qrand
    const int nearest = getNearest(qRand);
    const auto& qNear = tree_[nearest].q;
    LOG_TRACE("q_near: indice=", nearest, ", nodos=", tree_.size());

    //*****************13/11/2025*******************************************************
    //Steer: mover de q_near hacia q_rand con paso stepSize
    const auto qNew = steer(qNear, qRand, stepSize_);
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
    tree_.emplace_back(qNew, nearest);
    ++nNodesRRT_;
    LOG_TRACE("Vertice agregado: indice=", newIndex,
              ", padre=", nearest, ", total=", tree_.size());

    const float distanceToGoal = configDistance(qNew, goalQ_);
    LOG_TRACE("Distancia de q_new a meta: ", distanceToGoal);
    if( distanceToGoal > distToGoal_)
        return false;

    LOG_DEBUG("Nodo dentro de tolerancia de meta: indice=", newIndex,
              ", distancia=", distanceToGoal);

    if( tree_.size() >= static_cast<std::size_t>(maxNodes_))
        return false;


    // comprobar la conexion final
    if( !isEdgeValid(qNew, goalQ_) ) {
        LOG_TRACE("Conexion final rechazada por colision: nodo=", newIndex);
        return false;
    }

    // Conectamos exactamente al goal del robot 0
    tree_.emplace_back(goalQ_, newIndex); //conectamos la meta con el ultimo nodo agregado

    // Backtracking del camino
    finalPath_.clear();
    nNodesPath_ = 0;
    int index = static_cast<int>(tree_.size()) - 1;
    while (index != -1 ){
        finalPath_.push_back(index);
        index = tree_[index].parent;
        ++nNodesPath_;
    }
    std::reverse(finalPath_.begin(), finalPath_.end());

    done_ = true;
    const auto stats = collisionChecker_.statistics();
    LOG_SUCCESS("Ruta RRT encontrada: nodos_arbol=", tree_.size(),
                ", nodos_ruta=", finalPath_.size(),
                ", consultas_colision=", stats.queries);
    return true;

     // AMGL// esto es lo que estaba anteriormente //ya no deberia ser util
    // if( !qNew.empty() ){
    //     const float distanceToGoal = configDistance(qNew, goalQ_);//Calculamos la distancia des qrand a la meta
    //     if( distanceToGoal <= distToGoal_ ){
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
    [[maybe_unused]] const float wth=diamRobot_*diamRobot_;//peso angular

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

    const bool valid = !collisionChecker_.configurationInCollision(q);
    LOG_TRACE("Validacion de configuracion: robots=", q.size(),
              ", valida=", valid);
    return valid;
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

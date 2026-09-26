#pragma once

#include "Obstacle.h"
#include "Config.h"

#include <vector>


// Creamos un planeador generico para poder reciclar la parte grafica
// para usarlo con RTT o cualquier otro algoritmo heredando
// de esta clase vease RRTPlanner.h
class IPlanner {
public:
    virtual ~IPlanner() = default;

    // --- Configuracion del problema ---
    virtual void setStart(const std::vector<Config>& q, float radius) = 0;
    virtual void setGoal(const std::vector<Config>& q, float radius) = 0;
    virtual void setNumRobots(int n) = 0;
    virtual void setObstacles( const std::vector<PolygonObstacle>& obstacles) = 0;

    // --- Parametros del algoritmo ---
    virtual void setStepSize(float step) = 0;
    virtual void setMaxNodes(int maxNodes) = 0;
    virtual void setGoalTolerance(float distance) = 0;

    // --- Ciclo de vida de planificacion ---
    virtual void reset() = 0;
    virtual bool step() = 0;
    virtual bool isDone() const = 0;
    /*
        step(): devuelve true si existe sol
        isDone(): devuelve true si termino, con solucion o por fallo
    */

    // --- Lectura de resultados ---
    virtual const std::vector<PlanNode>& getTree() const = 0;
    virtual const std::vector<int>& getPath() const = 0;
    virtual const std::vector<Config>& getGoal() const = 0;
    virtual const std::vector<Config>& getOrigin() const = 0;
    virtual int getNumRobots() const = 0;


    // --- obstaculos ---
    void SetObstacles(const std::vector<PolygonObstacle>& obstacles);
    void StopPlanning();

    // por si queremos graficar visualmente
    // --- Telemetria opcional ---
    virtual int getNodeCount() const = 0;
    virtual int getPathNodeCount() const = 0;


};
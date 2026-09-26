#pragma once
#include <QOpenGLWidget>
#include <QTimer>
#include "IPlanner.h" //clase general // aqui esta el RTT
#include "VelocityIntegrator.h"
#include "Obstacle.h"
#include <memory>


//RRTWidget hereda QOpenGLWidget
class RRTWidget : public QOpenGLWidget {
    Q_OBJECT
public:
    RRTWidget(QWidget *parent = nullptr);//constructor, inicializa los graficos, el arbol y el temporizador
    void setPlanner(std::shared_ptr<IPlanner> planner);
    //QPointF ReadOriginTreePos() const {return QPointF(originX, originY, originTH);}
    //QPointF ReadGoalTreePos() const {return QPointF(goalX, goalY, goalTH);}
    int ReadTimeGrow() const{return timeGrowMs_;}

    // obstaculos
    void SetObstacles(const std::vector<PolygonObstacle>& obstacles);
    void StopPlanning();

    bool ActiveEuler=true;//new

public slots:
    void OriginTree(std::vector<float>& x, std::vector<float>& y, std::vector<float>& th,int Nrobots, float radio);
    void GoalTree(std::vector<float>& x, std::vector<float>& y, std::vector<float>& th,int Nrobots, float radio);
    void computeVelocities(float Tau, int Nrobots);
    void VelocitiesRobots(std::vector<float>& Vx, std::vector<float>& Vy, std::vector<float>& Wang, int Nrobots);//new
    void DrawMyNodes(bool All, bool FinalPath);
    void DistanceToTheGoal(float distance);
    void ParamsTreeRRT(float Step, int MaximalNodes);
    void SetTimeGrow(int Tgrow);
    void EulerMult(float DeltaT);//new

protected:
    void initializeGL() override;//se llama una sola vez para configurar el opengl
    void paintGL() override;//se dibuja todo (arbol, meta origen y camino final)
    void resizeGL(int w, int h) override;//ajuste de vista al cambiar el tamaño de ventana

private slots:
    void growTree();// se llama cada 30ms para controlar el crecimiento del arbol RRT

private:
    //AMGL// el estado del RTT es guardado aparte para separar  algoritmo de visualizacion
    std::shared_ptr<IPlanner> planner_; //RTT_planner // en general IPlaner

    // la idea es que este modulo podremos elegir que tipo es
    // avanzar en lineas rectas // metodo de euler // o
    VelocityIntegrator velocityIntegrator_;

    // obstaculos
    std::vector<PolygonObstacle> obstacles_;
    void drawObstacles();

    QTimer timer_; //temporizador que activa el arbol
    bool drawAllNodes_=true;
    bool drawFinalPath_=true;

    int timeGrowMs_=30; //milisegundos
    float goalRadius_ = 5.0f;//radio de la meta
    float originRadius_ = 5.0f;//radio del origen
};

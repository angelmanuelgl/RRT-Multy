#pragma once
#include <QOpenGLWidget>
#include <QElapsedTimer>
#include <QPoint>
#include <QTimer>
#include "IPlanner.h" //clase general // aqui esta el RTT
#include "VelocityIntegrator.h"
#include "Obstacle.h"
#include <memory>

class QFrame;
class QLabel;
class QKeyEvent;
class QMouseEvent;
class QPushButton;
class QResizeEvent;
class QWheelEvent;


//RRTWidget hereda QOpenGLWidget
class RRTWidget : public QOpenGLWidget {
    Q_OBJECT
public:
    RRTWidget(QWidget *parent = nullptr);//constructor, inicializa los graficos, el arbol y el temporizador
    void setPlanner(std::shared_ptr<IPlanner> planner);
    //QPointF ReadOriginTreePos() const {return QPointF(originX, originY, originTH);}
    //QPointF ReadGoalTreePos() const {return QPointF(goalX, goalY, goalTH);}
    int ReadTimeGrow() const{return timeGrowMs_;}

    // amgl // configurar los obstaculos sin dejar un estado visual parcial.
    void SetObstacles(const std::vector<PolygonObstacle>& obstacles);


    // amgl // visual // Detiene la planificacion y congela su telemetria.
    void StopPlanning();

    bool ActiveEuler=false;

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

    // amgl // visual
    // ajusta el tam d el panel cuando cambia el tamano del widget
    void resizeEvent(QResizeEvent *event) override;
    // zoom mediante la rueda del raton.
    void wheelEvent(QWheelEvent *event) override;
    // atajos de teclado para pausa, paso y zoom.
    void keyPressEvent(QKeyEvent *event) override;
    // inicia el desplazamiento de camara mediante arrastre
    void mousePressEvent(QMouseEvent *event) override;
    // acumula el desplazamiento de camara durante el arrastre
    void mouseMoveEvent(QMouseEvent *event) override;
    // finaliza el desplazamiento de camara mediante arrastre
    void mouseReleaseEvent(QMouseEvent *event) override;

private slots:
    void growTree();// se llama cada 30ms para controlar el crecimiento del arbol RRT

    // amgl // visual
    // cambiar entre pausada y activa.
    void togglePause();
    // ejecuta un unico paso // pone tambien la simulacion pausada
    void stepOnce();
    // aumenta escala
    void zoomIn();
    // reduce escala
    void zoomOut();
    // resetear escala
    void resetZoom();
    // aqui actualizamos las descricpiones de cada paso del RTT
    void advanceConceptualPhase();

private:
    // amgl // visual
    enum class VisualState {
        Preparing,
        Running,
        Paused,
        ManualStep,
        GoalReached,
        NodeLimitReached,
        PlanningFailed,
        InvalidConfiguration
    };

    // amgl // visual // Construye el panel HUD y conecta sus controles.
    void createOverlayPanel();
    // amgl // visual // Sincroniza textos, contadores y botones del HUD.
    void refreshOverlay();
    // amgl // visual // Coloca el HUD sobre el area OpenGL.
    void positionOverlay();
    // amgl // visual // Ejecuta un ciclo de simulacion compartido por timer y paso manual.
    void executeSimulationTick(bool manualStep);
    // amgl // visual // Ejecuta una iteracion del planner y actualiza telemetria visual.
    bool executePlannerIteration();
    // amgl // visual // Inicia o reanuda la medicion de tiempo activo.
    void startElapsedTime();
    // amgl // visual // Congela y acumula la medicion de tiempo activo.
    void stopElapsedTime();
    // amgl // visual // Devuelve el tiempo activo acumulado en milisegundos.
    qint64 elapsedMilliseconds() const;
    // amgl // visual // Cambia el estado global mostrado por el HUD.
    void setVisualState(VisualState state);
    // amgl // visual // Aplica la proyeccion respetando aspecto, centro y zoom.
    void applyProjection(int w, int h);
    // amgl // visual // Dibuja una cuadricula de referencia en el escenario.
    void drawGrid() const;
    // amgl // visual // Ajusta el zoom dentro de limites seguros.
    void setZoomFactor(float factor);
    // amgl // visual // Desplaza el centro de la camara en coordenadas del mundo.
    void panCamera(float dx, float dy);
    // amgl // visual // Comprueba la configuracion observable antes de ejecutar el planner.
    bool hasValidConfiguration() const;

    //AMGL// el estado del RTT es guardado aparte para separar  algoritmo de visualizacion
    std::shared_ptr<IPlanner> planner_; //RTT_planner // en general IPlaner

    // la idea es que este modulo podremos elegir que tipo es
    // avanzar en lineas rectas // metodo de euler // o
    VelocityIntegrator velocityIntegrator_;

    // obstaculos
    std::vector<PolygonObstacle> obstacles_;
    // dibuja todos los obstaculos
    void drawObstacles();

    QTimer timer_; //temporizador que activa el arbol
    QTimer phaseTimer_; // amgl // visual
    bool drawAllNodes_=true;
    bool drawFinalPath_=true;

    // amgl // visual
    QFrame *statusPanel_ = nullptr;
    QFrame *telemetryPanel_ = nullptr;
    QFrame *executionPanel_ = nullptr;
    QFrame *cameraPanel_ = nullptr;
    QLabel *stateLabel_ = nullptr;
    QLabel *phaseLabel_ = nullptr;
    QLabel *stepsLabel_ = nullptr;
    QLabel *nodesLabel_ = nullptr;
    QLabel *elapsedLabel_ = nullptr;
    QLabel *zoomLabel_ = nullptr;
    QPushButton *pauseButton_ = nullptr;
    QPushButton *stepButton_ = nullptr;

    // amgl // visual
    VisualState visualState_ = VisualState::Preparing;
    int conceptualPhase_ = 0;
    quint64 executedSteps_ = 0;
    int configuredMaxNodes_ = 0;
    bool paused_ = false;
    float zoomFactor_ = 1.0f;
    float cameraCenterX_ = 250.0f;
    float cameraCenterY_ = 250.0f;
    bool panning_ = false;
    QPoint lastPanPosition_;
    QElapsedTimer activeClock_;
    qint64 accumulatedElapsedMs_ = 0;

    int timeGrowMs_=30; //milisegundos
    float goalRadius_ = 5.0f;//radio de la meta
    float originRadius_ = 5.0f;//radio del origen
};

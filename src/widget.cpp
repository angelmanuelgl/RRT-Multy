#include "widget.h"
#include "RRTPlanner.h"

#include "logger.h"

#include <QOpenGLFunctions>
#include <QFrame>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QMouseEvent>
#include <QPushButton>
#include <QResizeEvent>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QtMath>
#include <cmath>
#include <QPainter>
#include <algorithm>
#include <cstddef>
#include <exception>
#include <QTextStream>
#include <tuple>
#include <utility>
//Se inicializa el widget
RRTWidget::RRTWidget(QWidget *parent)
    : QOpenGLWidget(parent),
      planner_(std::make_shared<RRTPlanner>())
{
    //**opcional porque asi se inicializa con un robot********
    //verificar que la configuración de origen tenga al menos un robot
    const auto &originQ = planner_->getOrigin();
    //**opcional porque asi se inicializa con un robot********



    const int numRobots = static_cast<int>(originQ.size());//revisamos el numero de robots de la configuracion inicial
    (void)numRobots;

    velocityIntegrator_.setState(originQ);

    // amgl // visual
    setFocusPolicy(Qt::StrongFocus);
    createOverlayPanel();
    phaseTimer_.setInterval(450);
    connect(&phaseTimer_, &QTimer::timeout,
            this, &RRTWidget::advanceConceptualPhase);

    //cada timeGrow=30 ms nos conectamos al arbol llamando a growtree
    //para expandir el arbol
    connect(&timer_, &QTimer::timeout, this, &RRTWidget::growTree);
    // La ejecucion inicia cuando el escenario termina de configurarse. // amgl // visual
    refreshOverlay(); // amgl // visual
    LOG_SUCCESS("Widget de visualizacion inicializado: robots_predeterminados=",
                numRobots);
}

void RRTWidget::setPlanner(std::shared_ptr<IPlanner> planner)
{
    if (!planner) {
        LOG_FATAL("No se puede instalar un planner nulo en RRTWidget");
        qWarning("setPlanner: planner nulo");
        return;
    }

    // AMGL // conservar obstaculos al intercambiar la implementacion del planner
    planner->setObstacles(obstacles_);
    planner_ = std::move(planner);
    velocityIntegrator_.setState(planner_->getOrigin());
    // amgl // visual
    executedSteps_ = 0;
    conceptualPhase_ = 0;
    accumulatedElapsedMs_ = 0;
    paused_ = false;
    setVisualState(VisualState::Preparing);
    startElapsedTime();
    refreshOverlay();
    update();
    LOG_SUCCESS("Planner instalado en RRTWidget: obstaculos=", obstacles_.size(),
                ", robots=", planner_->getNumRobots());
}

//aqui inicializamos el opengl (solo se llama una vez) y se define el color de fonde (blanco en este caso)
void RRTWidget::initializeGL() {
    // amgl // visual
    glClearColor(0.965f, 0.975f, 0.985f, 1.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    LOG_SUCCESS("Contexto OpenGL inicializado para el visualizador RRT");
}

void RRTWidget::OriginTree(std::vector<float>& x, std::vector<float>& y,
                           std::vector<float>& th, int Nrobots, float radio)
{
    if (Nrobots <= 0)
    {
        LOG_ERROR("Origen invalido: numero de robots=", Nrobots);
        qWarning("OriginTree: NRobots <= 0");
        return;
    }
    if (x.size() < static_cast<std::size_t>(Nrobots)
        || y.size() < static_cast<std::size_t>(Nrobots)
        || th.size() < static_cast<std::size_t>(Nrobots)) {
        LOG_ERROR("Datos de origen insuficientes: robots=", Nrobots,
                  ", x=", x.size(), ", y=", y.size(), ", theta=", th.size());
        qWarning("OriginTree: datos insuficientes");
        setVisualState(VisualState::InvalidConfiguration);
        return;
    }


    std::vector<Config> origin(Nrobots);//Actualizar numero de robots

    //AMGL// Next y Curr estan en VelocityIntegrator

    for(int i=0;i<Nrobots; i++)
    {
        origin[i]={x[i],y[i],th[i]};//Tomando coordenadas de robots en la configuración origen
    }

    originRadius_=radio;//radio del robot


    velocityIntegrator_.setState(origin);
    planner_->setNumRobots(Nrobots);
    planner_->setStart(origin, radio);

    // amgl // visual
    executedSteps_ = 0;
    conceptualPhase_ = 0;
    accumulatedElapsedMs_ = 0;
    paused_ = false;
    setVisualState(VisualState::Preparing);
    startElapsedTime();
    phaseTimer_.start();

    //Reset para la nueva posición de inicio (de otro modo lee la default)
    timer_.stop();
    timer_.start(timeGrowMs_); // mismo periodo que se usa ahora
    refreshOverlay(); // amgl // visual
    update();
    LOG_INFO("Origen enviado al planner: robots=", Nrobots, ", radio=", radio);
}

void RRTWidget::GoalTree(std::vector<float>& x, std::vector<float>& y,
                         std::vector<float>& th, int Nrobots, float radio)
{
    if (Nrobots <= 0) {
        LOG_ERROR("Meta invalida: numero de robots=", Nrobots);
        qWarning("GoalTree: NRobots <= 0");
        return;
    }
    if (x.size() < static_cast<std::size_t>(Nrobots)
        || y.size() < static_cast<std::size_t>(Nrobots)
        || th.size() < static_cast<std::size_t>(Nrobots)) {
        LOG_ERROR("Datos de meta insuficientes: robots=", Nrobots,
                  ", x=", x.size(), ", y=", y.size(), ", theta=", th.size());
        qWarning("GoalTree: datos insuficientes");
        setVisualState(VisualState::InvalidConfiguration);
        return;
    }


    std::vector<Config> goal(Nrobots);
    for(int i=0;i<Nrobots; i++)
    {
        goal[i]={x[i],y[i],th[i]};
    }
    goalRadius_=radio;

    planner_->setNumRobots(Nrobots);
    planner_->setGoal(goal, radio);

    // amgl // visual
    setVisualState(hasValidConfiguration()
        ? VisualState::Running
        : VisualState::InvalidConfiguration);
    refreshOverlay();
    LOG_INFO("Meta enviada al planner: robots=", Nrobots, ", radio=", radio);
}

void RRTWidget::computeVelocities(float Tau, int Nrobots)
{
    (void)Nrobots;
    LOG_DEBUG("Solicitud de calculo de velocidades: tau=", Tau,
              ", robots=", Nrobots);
    velocityIntegrator_.computeVelocities(Tau);
}


void RRTWidget::VelocitiesRobots(std::vector<float>& Vx,
                                 std::vector<float>& Vy,
                                 std::vector<float>& Wang,
                                 int Nrobots)
{
    if (Nrobots <= 0
        || Vx.size() < static_cast<std::size_t>(Nrobots)
        || Vy.size() < static_cast<std::size_t>(Nrobots)
        || Wang.size() < static_cast<std::size_t>(Nrobots)) {
        LOG_ERROR("Datos de velocidad insuficientes: robots=", Nrobots,
                  ", Vx=", Vx.size(), ", Vy=", Vy.size(),
                  ", Wang=", Wang.size());
        qWarning("VelocitiesRobots: datos insuficientes");
        setVisualState(VisualState::InvalidConfiguration);
        return;
    }

    std::vector<Velocities> velocities(Nrobots);
    for(int i=0;i<Nrobots; i++)
    {
        velocities[i]={Vx[i],Vy[i],Wang[i]};
    }
    velocityIntegrator_.setVelocities(velocities);
    LOG_DEBUG("Velocidades enviadas al integrador: robots=", Nrobots);
}

void RRTWidget::DrawMyNodes(bool All, bool FinalPath)
{
    drawAllNodes_=All;
    drawFinalPath_=FinalPath;
    LOG_DEBUG("Opciones de dibujo: arbol_completo=", All,
              ", ruta_final=", FinalPath);
}


void RRTWidget::DistanceToTheGoal(float distance)//Definimos la distancia a la meta que consideramos suficiente para conectarla al arbol
{
    planner_->setGoalTolerance(distance);
    LOG_DEBUG("Distancia de llegada configurada desde widget: ", distance);
}

void RRTWidget::ParamsTreeRRT(float Step, int MaximalNodes){//Definimos el tamaño de paso (step) y el numero maximo de nodos en el arbol
    planner_->setStepSize(Step);
    planner_->setMaxNodes(MaximalNodes);
    configuredMaxNodes_ = MaximalNodes; // amgl // visual
    refreshOverlay(); // amgl // visual
    LOG_INFO("Parametros RRT configurados desde widget: paso=", Step,
             ", max_nodos=", MaximalNodes);
}

void RRTWidget::SetTimeGrow(int Tgrow){//Tiempo de crecimiento en cada iteracion del arbol
    timeGrowMs_=std::max(1, Tgrow); // amgl // visual
    timer_.start(timeGrowMs_);
    if (Tgrow < 1)
        LOG_WARN("Periodo ajustado a 1 ms; solicitado=", Tgrow);
    LOG_INFO("Temporizador de planificacion iniciado: periodo_ms=", timeGrowMs_);
}


//define el area de visualización y sistema de coordenadas
/*void RRTWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, w, h, 0, -1, 1);  // coordenadas origen arriba-izquierda
    glMatrixMode(GL_MODELVIEW);
}*/

void RRTWidget::resizeGL(int w, int h)// con escalado de acuerdo al aspecto
{
    // amgl // visual
    applyProjection(w, h);
}

// amgl // visual // Reubica el panel superpuesto cuando cambia el tamano del widget.
void RRTWidget::resizeEvent(QResizeEvent *event)
{
    QOpenGLWidget::resizeEvent(event);
    positionOverlay();
}

// amgl // visual // Controla el zoom mediante la rueda del raton.
void RRTWidget::wheelEvent(QWheelEvent *event)
{
    if (event->angleDelta().y() > 0)
        zoomIn();
    else if (event->angleDelta().y() < 0)
        zoomOut();
    event->accept();
}

// amgl // visual // Ofrece atajos de teclado para pausa, paso y zoom.
void RRTWidget::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Space:
        togglePause();
        return;
    case Qt::Key_Return:
    case Qt::Key_Enter:
        stepOnce();
        return;
    case Qt::Key_Left:
        panCamera(-25.0f / zoomFactor_, 0.0f);
        return;
    case Qt::Key_Right:
        panCamera(25.0f / zoomFactor_, 0.0f);
        return;
    case Qt::Key_Up:
        panCamera(0.0f, -25.0f / zoomFactor_);
        return;
    case Qt::Key_Down:
        panCamera(0.0f, 25.0f / zoomFactor_);
        return;
    case Qt::Key_Plus:
    case Qt::Key_Equal:
        zoomIn();
        return;
    case Qt::Key_Minus:
        zoomOut();
        return;
    case Qt::Key_0:
        resetZoom();
        return;
    default:
        QOpenGLWidget::keyPressEvent(event);
    }
}

// amgl // visual // Inicia el desplazamiento de camara mediante arrastre.
void RRTWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton
        || event->button() == Qt::MiddleButton
        || event->button() == Qt::RightButton) {
        panning_ = true;
        lastPanPosition_ = event->position().toPoint();
        setCursor(Qt::ClosedHandCursor);
        setFocus(Qt::MouseFocusReason);
        event->accept();
        return;
    }
    QOpenGLWidget::mousePressEvent(event);
}

// amgl // visual // Acumula el desplazamiento de camara durante el arrastre.
void RRTWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (!panning_ || width() <= 0 || height() <= 0) {
        QOpenGLWidget::mouseMoveEvent(event);
        return;
    }

    const QPoint currentPosition = event->position().toPoint();
    const QPoint pixelDelta = currentPosition - lastPanPosition_;
    lastPanPosition_ = currentPosition;

    const float baseVisibleWorld = 500.0f / zoomFactor_;
    const float aspect = static_cast<float>(width()) / static_cast<float>(height());
    const float visibleW = aspect > 1.0f
        ? baseVisibleWorld * aspect : baseVisibleWorld;
    const float visibleH = aspect < 1.0f
        ? baseVisibleWorld / aspect : baseVisibleWorld;

    panCamera(-pixelDelta.x() * visibleW / static_cast<float>(width()),
              -pixelDelta.y() * visibleH / static_cast<float>(height()));
    event->accept();
}

// amgl // visual // Finaliza el desplazamiento de camara mediante arrastre.
void RRTWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (panning_ && (event->button() == Qt::LeftButton
                     || event->button() == Qt::MiddleButton
                     || event->button() == Qt::RightButton)) {
        panning_ = false;
        unsetCursor();
        event->accept();
        return;
    }
    QOpenGLWidget::mouseReleaseEvent(event);
}

// amgl // visual // Construye el panel HUD y conecta sus controles.
void RRTWidget::createOverlayPanel()
{
    // amgl // visual
    const QString panelStyle = QStringLiteral(
        "QFrame[rrtPanel=\"true\"] { background: rgba(20, 29, 43, 205);"
        " border: 1px solid rgba(255,255,255,40); border-radius: 9px; }"
        "QLabel { color: #eaf2ff; background: transparent; font-size: 11px; }"
        "QLabel#rrtPhase { color: #8fd3ff; font-family: monospace; font-size: 10px; }"
        "QPushButton { color: #f7fbff; background: rgba(69,111,166,215);"
        " border: 1px solid rgba(255,255,255,40); border-radius: 6px;"
        " min-height: 24px; padding: 3px 9px; font-size: 11px; font-weight: 600; }"
        "QPushButton:hover { background: rgba(83,139,207,235); }"
        "QPushButton:pressed { background: rgba(46,83,132,240); }"
        "QPushButton:disabled { color: #8390a3; background: rgba(45,55,70,180); }");

    auto createPanel = [this, &panelStyle]() {
        auto *panel = new QFrame(this);
        panel->setProperty("rrtPanel", true);
        panel->setStyleSheet(panelStyle);
        return panel;
    };

    statusPanel_ = createPanel();
    telemetryPanel_ = createPanel();
    executionPanel_ = createPanel();
    cameraPanel_ = createPanel();

    auto *statusLayout = new QVBoxLayout(statusPanel_);
    statusLayout->setContentsMargins(10, 7, 10, 7);
    statusLayout->setSpacing(3);
    stateLabel_ = new QLabel(statusPanel_);
    phaseLabel_ = new QLabel(statusPanel_);
    phaseLabel_->setObjectName(QStringLiteral("rrtPhase"));
    phaseLabel_->setWordWrap(true);
    statusLayout->addWidget(stateLabel_);
    statusLayout->addWidget(phaseLabel_);

    auto *telemetryLayout = new QVBoxLayout(telemetryPanel_);
    telemetryLayout->setContentsMargins(10, 7, 10, 7);
    telemetryLayout->setSpacing(3);
    auto *metrics = new QHBoxLayout;
    stepsLabel_ = new QLabel(telemetryPanel_);
    nodesLabel_ = new QLabel(telemetryPanel_);
    elapsedLabel_ = new QLabel(telemetryPanel_);
    metrics->addWidget(stepsLabel_);
    metrics->addSpacing(10);
    metrics->addWidget(nodesLabel_);
    telemetryLayout->addLayout(metrics);
    telemetryLayout->addWidget(elapsedLabel_, 0, Qt::AlignRight);

    auto *executionLayout = new QHBoxLayout(executionPanel_);
    executionLayout->setContentsMargins(8, 6, 8, 6);
    executionLayout->setSpacing(5);
    pauseButton_ = new QPushButton(QStringLiteral("Pausar"), executionPanel_);
    stepButton_ = new QPushButton(QStringLiteral("Paso"), executionPanel_);
    executionLayout->addWidget(pauseButton_);
    executionLayout->addWidget(stepButton_);

    auto *cameraLayout = new QHBoxLayout(cameraPanel_);
    cameraLayout->setContentsMargins(8, 6, 8, 6);
    cameraLayout->setSpacing(5);
    auto *zoomOutButton = new QPushButton(QStringLiteral("Zoom -"), cameraPanel_);
    auto *zoomInButton = new QPushButton(QStringLiteral("Zoom +"), cameraPanel_);
    auto *zoomResetButton = new QPushButton(QStringLiteral("Restablecer"), cameraPanel_);
    zoomLabel_ = new QLabel(cameraPanel_);
    zoomLabel_->setMinimumWidth(36);
    zoomLabel_->setAlignment(Qt::AlignCenter);
    cameraLayout->addWidget(zoomOutButton);
    cameraLayout->addWidget(zoomInButton);
    cameraLayout->addWidget(zoomResetButton);
    cameraLayout->addWidget(zoomLabel_);

    connect(pauseButton_, &QPushButton::clicked, this, &RRTWidget::togglePause);
    connect(stepButton_, &QPushButton::clicked, this, &RRTWidget::stepOnce);
    connect(zoomOutButton, &QPushButton::clicked, this, &RRTWidget::zoomOut);
    connect(zoomInButton, &QPushButton::clicked, this, &RRTWidget::zoomIn);
    connect(zoomResetButton, &QPushButton::clicked, this, &RRTWidget::resetZoom);

    statusPanel_->raise();
    telemetryPanel_->raise();
    executionPanel_->raise();
    cameraPanel_->raise();
    positionOverlay();
}

// amgl // visual // Sincroniza textos, contadores y botones del HUD.
void RRTWidget::refreshOverlay()
{
    if (!statusPanel_ || !telemetryPanel_
        || !executionPanel_ || !cameraPanel_)
        return;

    QString stateText;
    switch (visualState_) {
    case VisualState::Preparing: stateText = QStringLiteral("Preparando escenario"); break;
    case VisualState::Running: stateText = QStringLiteral("Ejecutando"); break;
    case VisualState::Paused: stateText = QStringLiteral("Pausado"); break;
    case VisualState::ManualStep: stateText = QStringLiteral("Ejecutando paso manual"); break;
    case VisualState::GoalReached: stateText = QStringLiteral("Meta alcanzada"); break;
    case VisualState::NodeLimitReached: stateText = QStringLiteral("Límite de nodos alcanzado"); break;
    case VisualState::PlanningFailed: stateText = QStringLiteral("Búsqueda finalizada sin solución"); break;
    case VisualState::InvalidConfiguration: stateText = QStringLiteral("Configuración inválida"); break;
    }

    static const char *const phases[] = {
        "Muestreando: q_rand <- RANDOM_STATE()",
        "Vecino más cercano: q_near <- NEAREST_NEIGHBOR(q_rand, T)",
        "Selección de entrada: u <- SELECT_INPUT_RECTA(q_rand, q_near)",
        "Integración/Steer: q_new <- FINAL_DE_LA_RECTA(q_near, u, dt)",
        "Actualización de árbol: T.ADD_VERTEX(q_new) & T.ADD_EDGE(q_near, q_new, u)"
    };

    stateLabel_->setText(QStringLiteral("Estado: %1").arg(stateText));
    phaseLabel_->setText(QString::fromUtf8(phases[conceptualPhase_]));
    stepsLabel_->setText(QStringLiteral("Pasos: %1").arg(executedSteps_));

    const int nodes = planner_ ? static_cast<int>(planner_->getTree().size()) : 0;
    nodesLabel_->setText(configuredMaxNodes_ > 0
        ? QStringLiteral("Nodos: %1 / %2").arg(nodes).arg(configuredMaxNodes_)
        : QStringLiteral("Nodos: %1").arg(nodes));

    const qint64 elapsed = elapsedMilliseconds();
    const qint64 minutes = elapsed / 60000;
    const qint64 seconds = (elapsed / 1000) % 60;
    const qint64 centiseconds = (elapsed / 10) % 100;
    elapsedLabel_->setText(QStringLiteral("Tiempo: %1:%2.%3")
        .arg(minutes, 2, 10, QLatin1Char('0'))
        .arg(seconds, 2, 10, QLatin1Char('0'))
        .arg(centiseconds, 2, 10, QLatin1Char('0')));

    zoomLabel_->setText(QStringLiteral("%1%").arg(qRound(zoomFactor_ * 100.0f)));
    pauseButton_->setText(paused_ ? QStringLiteral("Continuar") : QStringLiteral("Pausar"));
    const bool finished = visualState_ == VisualState::GoalReached
        || visualState_ == VisualState::NodeLimitReached
        || visualState_ == VisualState::PlanningFailed
        || visualState_ == VisualState::InvalidConfiguration;
    const bool presentingManualStep = visualState_ == VisualState::ManualStep;
    pauseButton_->setEnabled(!finished && !presentingManualStep);
    stepButton_->setEnabled(!finished && !presentingManualStep);
}

// amgl // visual // Coloca el HUD sobre el area OpenGL.
void RRTWidget::positionOverlay()
{
    if (!statusPanel_ || !telemetryPanel_
        || !executionPanel_ || !cameraPanel_)
        return;

    // amgl // visual
    const int margin = 10;
    const int gap = 10;
    const int availableWidth = std::max(260, width() - 2 * margin);
    const int cornerWidth = std::max(125, (availableWidth - gap) / 2);

    statusPanel_->setFixedWidth(cornerWidth);
    telemetryPanel_->setMaximumWidth(cornerWidth);
    statusPanel_->adjustSize();
    telemetryPanel_->adjustSize();
    executionPanel_->adjustSize();
    cameraPanel_->adjustSize();

    statusPanel_->move(margin, margin);
    telemetryPanel_->move(width() - margin - telemetryPanel_->width(), margin);
    executionPanel_->move(margin,
                          height() - margin - executionPanel_->height());
    cameraPanel_->move(width() - margin - cameraPanel_->width(),
                       height() - margin - cameraPanel_->height());

    statusPanel_->raise();
    telemetryPanel_->raise();
    executionPanel_->raise();
    cameraPanel_->raise();
}

// amgl // visual // Inicia o reanuda la medicion de tiempo activo.
void RRTWidget::startElapsedTime()
{
    if (!activeClock_.isValid())
        activeClock_.start();
}

// amgl // visual // Congela y acumula la medicion de tiempo activo.
void RRTWidget::stopElapsedTime()
{
    if (activeClock_.isValid()) {
        accumulatedElapsedMs_ += activeClock_.elapsed();
        activeClock_.invalidate();
    }
}

// amgl // visual // Devuelve el tiempo activo acumulado en milisegundos.
qint64 RRTWidget::elapsedMilliseconds() const
{
    return accumulatedElapsedMs_
        + (activeClock_.isValid() ? activeClock_.elapsed() : 0);
}

// amgl // visual // Cambia el estado global mostrado por el HUD.
void RRTWidget::setVisualState(VisualState state)
{
    visualState_ = state;
    refreshOverlay();
}

// amgl // visual // Aplica la proyeccion respetando aspecto, centro y zoom.
void RRTWidget::applyProjection(int w, int h)
{
    if (w <= 0 || h <= 0)
        return;

    glViewport(0, 0, w, h);
    const float baseWorld = 500.0f;
    const float visibleWorld = baseWorld / zoomFactor_;
    const float aspect = static_cast<float>(w) / static_cast<float>(h);
    float visibleW = visibleWorld;
    float visibleH = visibleWorld;
    if (aspect > 1.0f)
        visibleW *= aspect;
    else
        visibleH /= aspect;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(cameraCenterX_ - visibleW * 0.5f,
            cameraCenterX_ + visibleW * 0.5f,
            cameraCenterY_ + visibleH * 0.5f,
            cameraCenterY_ - visibleH * 0.5f, -1, 1);
    glMatrixMode(GL_MODELVIEW);
}

// amgl // visual // Dibuja una cuadricula de referencia en el escenario.
void RRTWidget::drawGrid() const
{
    glColor4f(0.45f, 0.52f, 0.62f, 0.16f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for (int coordinate = 0; coordinate <= 500; coordinate += 50) {
        glVertex2f(static_cast<float>(coordinate), 0.0f);
        glVertex2f(static_cast<float>(coordinate), 500.0f);
        glVertex2f(0.0f, static_cast<float>(coordinate));
        glVertex2f(500.0f, static_cast<float>(coordinate));
    }
    glEnd();
}

// amgl // visual // Ajusta el zoom dentro de limites seguros.
void RRTWidget::setZoomFactor(float factor)
{
    const float previousZoom = zoomFactor_;
    zoomFactor_ = std::clamp(factor, 0.35f, 5.0f);
    if (zoomFactor_ != factor)
        LOG_WARN("Zoom ajustado al limite permitido: solicitado=", factor,
                 ", aplicado=", zoomFactor_);
    LOG_DEBUG("Zoom actualizado: anterior=", previousZoom,
              ", actual=", zoomFactor_);
    refreshOverlay();
    update();
}

// amgl // visual // Aumenta la escala visual del escenario.
void RRTWidget::zoomIn()
{
    setZoomFactor(zoomFactor_ * 1.2f);
}

// amgl // visual // Reduce la escala visual del escenario.
void RRTWidget::zoomOut()
{
    setZoomFactor(zoomFactor_ / 1.2f);
}

// amgl // visual // Restaura la escala visual predeterminada.
void RRTWidget::resetZoom()
{
    // amgl // visual
    cameraCenterX_ = 250.0f;
    cameraCenterY_ = 250.0f;
    LOG_INFO("Camara restablecida al centro y zoom predeterminado");
    setZoomFactor(1.0f);
}

// amgl // visual // Desplaza el centro de la camara en coordenadas del mundo.
void RRTWidget::panCamera(float dx, float dy)
{
    cameraCenterX_ += dx;
    cameraCenterY_ += dy;
    LOG_TRACE("Desplazamiento de camara: dx=", dx, ", dy=", dy,
              ", centro=(", cameraCenterX_, ",", cameraCenterY_, ")");
    update();
}

// amgl // visual // Avanza la descripcion conceptual de las etapas del RRT.
void RRTWidget::advanceConceptualPhase()
{
    // amgl // visual
    if (visualState_ == VisualState::ManualStep) {
        if (conceptualPhase_ < 4) {
            ++conceptualPhase_;
            refreshOverlay();
            return;
        }

        phaseTimer_.stop();
        startElapsedTime();
        executeSimulationTick(true);
        stopElapsedTime();
        phaseTimer_.setInterval(450);
        if (visualState_ != VisualState::GoalReached
            && visualState_ != VisualState::NodeLimitReached
            && visualState_ != VisualState::PlanningFailed
            && visualState_ != VisualState::InvalidConfiguration) {
            setVisualState(VisualState::Paused);
        }
        update();
        return;
    }

    if (!paused_ && visualState_ == VisualState::Running) {
        conceptualPhase_ = (conceptualPhase_ + 1) % 5;
        refreshOverlay();
    }
}

// amgl // visual // Comprueba la configuracion observable antes de ejecutar el planner.
bool RRTWidget::hasValidConfiguration() const
{
    if (!planner_ || planner_->getNumRobots() <= 0)
        return false;
    const auto &origin = planner_->getOrigin();
    const auto &goal = planner_->getGoal();
    return !origin.empty() && origin.size() == goal.size()
        && static_cast<int>(origin.size()) == planner_->getNumRobots();
}



//Aqui se redibuja el arbol
void RRTWidget::paintGL()
{   

    // amgl // visual
    applyProjection(width(), height());
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    drawGrid();
    drawObstacles(); // AMGL // obstaculos bajo el arbol y los robots
    refreshOverlay();

    // ---+++ Funcion que determina los colores por robot +++---
    auto colorForRobot = [](int r) {
        // Paleta simple pero estable (0..N-1)
        float cr = ((r * 97) % 255) / 255.0f;
        float cg = ((r * 150) % 255) / 255.0f;
        float cb = ((r * 19) % 255) / 255.0f;
        // Evitar colores muy oscuros
        const float minc = 0.2f;
        cr = std::max(cr, minc);
        cg = std::max(cg, minc);
        cb = std::max(cb, minc);
        return std::tuple<float,float,float>(cr,cg,cb);
    };

    if(!ActiveEuler)
    {
        const auto &tree = planner_->getTree();
        const auto &finalPath = planner_->getPath();
        const auto &goalQ = planner_->getGoal();
        const auto &originQ = planner_->getOrigin();

        // ---++++++++++++++++++++++++++++---

        // --- 1) Árbol completo: líneas por robot ---
        if (!tree.empty()) {
            // Asumimos que el tamaño de q en cada nodo = numRobots
            const int R = static_cast<int>(tree.front().q.size());
            for (int r = 0; r < R; ++r)// para cada robot en el nodo
            {
                auto [cr, cg, cb] = colorForRobot(r);//asignamos color al robot de cada nodo
                glColor4f(cr, cg, cb, 0.48f); // amgl // visual
                glLineWidth(1.0f);
                glBegin(GL_LINES);
                for (const auto& node : tree) {
                    if (node.parent != -1) {
                        const auto &q_child  = node.q;
                        const auto &q_parent = tree[node.parent].q;
                        if (r < (int)q_child.size() && r < (int)q_parent.size()) {
                            glVertex2f(q_child[r].x,  q_child[r].y);
                            glVertex2f(q_parent[r].x, q_parent[r].y);


                        }
                    }
                }
                glEnd();

                // amgl // visual
                if (drawAllNodes_) {
                    const float rn = 2.5f;
                    for (const auto &node : tree) {
                        if (r >= static_cast<int>(node.q.size()))
                            continue;
                        glBegin(GL_LINE_LOOP);
                        for (int i = 0; i < 24; ++i) {
                            const float angle = i * 2.0f * M_PI / 24.0f;
                            glVertex2f(node.q[r].x + rn * std::cos(angle),
                                       node.q[r].y + rn * std::sin(angle));
                        }
                        glEnd();
                    }
                }
            }
        }



        // --- 2) Caminos finales por robot (si existe finalPath) ---
        if (!finalPath.empty()) {
            // robots presentes en el primer nodo del camino
            const int R = static_cast<int>(tree[finalPath.front()].q.size());
            for (int r = 0; r < R; ++r) {
                auto [cr, cg, cb] = colorForRobot(r);
                glColor4f(cr, cg, cb, 0.95f); // amgl // visual
                glLineWidth(5.0f); // amgl // visual
                glBegin(GL_LINE_STRIP);
                for (int idx : finalPath) {
                    const auto &q = tree[idx].q;
                    if (r < (int)q.size()) {
                        glVertex2f(q[r].x, q[r].y);
                    }
                }
                glEnd();

                // amgl // visual
                if (drawFinalPath_) {
                    const float pathNodeRadius = 3.5f;
                    for (int idx : finalPath) {
                        const auto &q = tree[idx].q;
                        if (r >= static_cast<int>(q.size()))
                            continue;
                        glBegin(GL_LINE_LOOP);
                        for (int i = 0; i < 24; ++i) {
                            const float angle = i * 2.0f * M_PI / 24.0f;
                            glVertex2f(q[r].x + pathNodeRadius * std::cos(angle),
                                       q[r].y + pathNodeRadius * std::sin(angle));
                        }
                        glEnd();
                    }
                }
            }
            glLineWidth(1.0f);

           // QTextStream out(stdout);
           // out << "Numero de nodos en el path: " << NNodesPath << Qt::endl;
           // out << "Numero de nodos en el path: " << (NNodesRRT+2) << Qt::endl;
        }

        // --- 3) Metas (círculos rojos con contorno) ---
        for (const auto &g : goalQ) {
            const float r = goalRadius_;
            // relleno
            glColor4f(1.0f, 0.0f, 0.0f, 0.85f);
            glBegin(GL_TRIANGLE_FAN);
            glVertex2f(g.x, g.y);
            for (int i = 0; i <= 36; ++i) {
                float ang = i * 2.0f * M_PI / 36.0f;
                glVertex2f(g.x + r * std::cos(ang), g.y + r * std::sin(ang));
            }
            glEnd();
            // contorno
            glColor3f(0.6f, 0.0f, 0.0f);
            glBegin(GL_LINE_LOOP);
            for (int i = 0; i < 36; ++i) {
                float ang = i * 2.0f * M_PI / 36.0f;
                glVertex2f(g.x + r * std::cos(ang), g.y + r * std::sin(ang));
            }
            glEnd();
        }

        // --- 4) Orígenes (círculos verdes con contorno) ---
        for (const auto &o : originQ) {
            const float ro = originRadius_;
            // relleno
            glColor4f(0.0f, 1.0f, 0.0f, 0.85f);
            glBegin(GL_TRIANGLE_FAN);
            glVertex2f(o.x, o.y);
            for (int i = 0; i <= 36; ++i) {
                float ang = i * 2.0f * M_PI / 36.0f;
                glVertex2f(o.x + ro * std::cos(ang), o.y + ro * std::sin(ang));
            }
            glEnd();
            // contorno
            glColor3f(0.0f, 0.45f, 0.0f);
            glBegin(GL_LINE_LOOP);
            for (int i = 0; i < 36; ++i) {
                float ang = i * 2.0f * M_PI / 36.0f;
                glVertex2f(o.x + ro * std::cos(ang), o.y + ro * std::sin(ang));
            }
            glEnd();
        }
    }
    else
    {
        const auto &tree = planner_->getTree();
        const auto &goalQ = planner_->getGoal();
        const auto &originQ = planner_->getOrigin();
        const auto &Next = velocityIntegrator_.getNext();
        const auto &Prev = velocityIntegrator_.getPrevious();

        // ---++++++++++++++++++++++++++++---

        // --- 1) Árbol completo: líneas por robot ---
        if (!tree.empty()) {
            // Asumimos que el tamaño de q en cada nodo = numRobots
            const int R = static_cast<int>(tree.front().q.size());
            for (int r = 0; r < R; ++r)// para cada robot en el nodo
            {
                auto [cr, cg, cb] = colorForRobot(r);//asignamos color al robot de cada nodo
                glColor4f(cr, cg, cb, 0.45f); // amgl // visual
                glLineWidth(1.0f);
                glBegin(GL_LINES);
                for (const auto& node : tree) {
                    if (node.parent != -1) {
                        const auto &q_child  = node.q;
                        const auto &q_parent = tree[node.parent].q;
                        if (r < static_cast<int>(q_child.size())
                            && r < static_cast<int>(q_parent.size())) {
                            glVertex2f(q_child[r].x, q_child[r].y);
                            glVertex2f(q_parent[r].x, q_parent[r].y);
                        }
                    }
                }
                glEnd();

                // amgl // visual
                if (r < static_cast<int>(Prev.size())
                    && r < static_cast<int>(Next.size())) {
                    glColor4f(cr, cg, cb, 1.0f);
                    glLineWidth(4.0f);
                    glBegin(GL_LINES);
                    glVertex2f(Prev[r].x, Prev[r].y);
                    glVertex2f(Next[r].x, Next[r].y);
                    glEnd();

                    const float movingNodeRadius = 4.0f;
                    glBegin(GL_LINE_LOOP);
                    for (int i = 0; i < 24; ++i) {
                        const float angle = i * 2.0f * M_PI / 24.0f;
                        glVertex2f(Next[r].x + movingNodeRadius * std::cos(angle),
                                   Next[r].y + movingNodeRadius * std::sin(angle));
                    }
                    glEnd();
                    glLineWidth(1.0f);
                }
            }
        }

        // --- 3) Metas (círculos rojos con contorno) ---
        for (const auto &g : goalQ) {
            const float r = goalRadius_;
            // relleno
            glColor4f(1.0f, 0.0f, 0.0f, 0.85f);
            glBegin(GL_TRIANGLE_FAN);
            glVertex2f(g.x, g.y);
            for (int i = 0; i <= 36; ++i) {
                float ang = i * 2.0f * M_PI / 36.0f;
                glVertex2f(g.x + r * std::cos(ang), g.y + r * std::sin(ang));
            }
            glEnd();
            // contorno
            glColor3f(0.6f, 0.0f, 0.0f);
            glBegin(GL_LINE_LOOP);
            for (int i = 0; i < 36; ++i) {
                float ang = i * 2.0f * M_PI / 36.0f;
                glVertex2f(g.x + r * std::cos(ang), g.y + r * std::sin(ang));
            }
            glEnd();
        }

        // --- 4) Orígenes (círculos verdes con contorno) ---
        for (const auto &o : originQ) {
            const float ro =2; //originRadius;
            // relleno
            glColor4f(0.0f, 1.0f, 0.0f, 0.85f);
            glBegin(GL_TRIANGLE_FAN);
            glVertex2f(o.x, o.y);
            for (int i = 0; i <= 36; ++i) {
                float ang = i * 2.0f * M_PI / 36.0f;
                glVertex2f(o.x + ro * std::cos(ang), o.y + ro * std::sin(ang));
            }
            glEnd();
            // contorno
            glColor3f(0.0f, 0.45f, 0.0f);
            glBegin(GL_LINE_LOOP);
            for (int i = 0; i < 36; ++i) {
                float ang = i * 2.0f * M_PI / 36.0f;
                glVertex2f(o.x + ro * std::cos(ang), o.y + ro * std::sin(ang));
            }
            glEnd();
        }
    }

    // amgl // visual
    statusPanel_->raise();
    telemetryPanel_->raise();
    executionPanel_->raise();
    cameraPanel_->raise();
}


void RRTWidget::EulerMult(float DeltaT)
{
    velocityIntegrator_.advance(DeltaT);
}

// amgl // visual // Alterna la ejecucion entre pausada y activa.
void RRTWidget::togglePause()
{
    const bool finished = visualState_ == VisualState::GoalReached
        || visualState_ == VisualState::NodeLimitReached
        || visualState_ == VisualState::PlanningFailed
        || visualState_ == VisualState::InvalidConfiguration;
    if (finished)
        return;

    paused_ = !paused_;
    if (paused_) {
        timer_.stop();
        phaseTimer_.stop();
        stopElapsedTime();
        setVisualState(VisualState::Paused);
        LOG_INFO("Planificacion pausada: pasos=", executedSteps_,
                 ", nodos=", planner_->getTree().size());
    } else {
        timer_.start(timeGrowMs_);
        phaseTimer_.start();
        startElapsedTime();
        setVisualState(VisualState::Running);
        LOG_INFO("Planificacion reanudada: periodo_ms=", timeGrowMs_);
    }
    update();
}

// amgl // visual // Ejecuta un unico avance manteniendo la simulacion pausada.
void RRTWidget::stepOnce()
{
    const bool finished = visualState_ == VisualState::GoalReached
        || visualState_ == VisualState::NodeLimitReached
        || visualState_ == VisualState::PlanningFailed
        || visualState_ == VisualState::InvalidConfiguration;
    if (finished)
        return;

    paused_ = true;
    timer_.stop();
    phaseTimer_.stop();
    stopElapsedTime();
    conceptualPhase_ = 0;
    setVisualState(VisualState::ManualStep);
    phaseTimer_.setInterval(180);
    phaseTimer_.start();
    update();
    LOG_DEBUG("Paso manual solicitado: siguiente_paso=", executedSteps_ + 1);
}

// amgl // visual // Ejecuta una iteracion del planner y actualiza telemetria visual.
bool RRTWidget::executePlannerIteration()
{
    if (!hasValidConfiguration()) {
        timer_.stop();
        phaseTimer_.stop();
        stopElapsedTime();
        setVisualState(VisualState::InvalidConfiguration);
        LOG_ERROR("No se puede ejecutar el planner: configuracion observable invalida");
        return false;
    }

    if (configuredMaxNodes_ > 0
        && static_cast<int>(planner_->getTree().size()) >= configuredMaxNodes_) {
        timer_.stop();
        phaseTimer_.stop();
        stopElapsedTime();
        setVisualState(VisualState::NodeLimitReached);
        LOG_ERROR("Limite de nodos alcanzado sin ruta: nodos=",
                  planner_->getTree().size(), ", limite=", configuredMaxNodes_);
        return false;
    }

    ++executedSteps_;
    bool reachedGoal = false;
    try {
        reachedGoal = planner_->step();
    } catch (const std::exception& ex) {
        timer_.stop();
        phaseTimer_.stop();
        stopElapsedTime();
        setVisualState(VisualState::PlanningFailed);
        LOG_ERROR("planificacion interrumpida: ", ex.what());
        return false;
    }

    if (reachedGoal) {
        timer_.stop();
        phaseTimer_.stop();
        stopElapsedTime();
        setVisualState(VisualState::GoalReached);
        LOG_SUCCESS("Meta alcanzada por el widget: pasos=", executedSteps_,
                    ", nodos=", planner_->getTree().size());
        return true;
    }

    // Un intento rechazado por colision puede no insertar un nodo y no es un error.
    if (planner_->isDone()) {
        timer_.stop();
        phaseTimer_.stop();
        stopElapsedTime();
        const bool hitNodeLimit = configuredMaxNodes_ > 0
            && static_cast<int>(planner_->getTree().size()) >= configuredMaxNodes_;
        setVisualState(hitNodeLimit
            ? VisualState::NodeLimitReached
            : VisualState::PlanningFailed);
        LOG_ERROR("Planificacion finalizada sin ruta: pasos=", executedSteps_,
                  ", nodos=", planner_->getTree().size(),
                  ", limite_nodos=", hitNodeLimit);
        return false;
    }

    refreshOverlay();
    return false;
}

// amgl // visual // Ejecuta un ciclo de simulacion compartido por timer y paso manual.
void RRTWidget::executeSimulationTick(bool manualStep)
{
    setVisualState(manualStep ? VisualState::ManualStep : VisualState::Running);

    if (!ActiveEuler) {
        executePlannerIteration();
        update();
        return;
    }

    if (velocityIntegrator_.needsSegment()) {
        const std::size_t previousTreeSize = planner_->getTree().size();
        const bool reachedGoal = executePlannerIteration();
        const auto &tree = planner_->getTree();

        if (!reachedGoal && tree.size() > previousTreeSize) {
            const auto &newNode = tree[previousTreeSize];
            if (newNode.parent >= 0
                && newNode.parent < static_cast<int>(tree.size())) {
                const auto &qNear = tree[newNode.parent].q;
                const auto &qNew = newNode.q;
                velocityIntegrator_.beginSegment(qNear, qNew, 1.0f);
            }
        }
    }

    EulerMult(0.1f);
    update();
}

//Esta función se llama cada 30 ms en RRTWidget por default, pero toma el valor segun la función SetTimeGrow
void RRTWidget::growTree()
{
    // amgl // visual
    if (paused_)
        return;
    executeSimulationTick(false);
}

// amgl // visual // Detiene la planificacion y congela su telemetria.
void RRTWidget::StopPlanning()
{
    timer_.stop();
    phaseTimer_.stop();
    stopElapsedTime();
    paused_ = true;
    setVisualState(VisualState::Paused);
    update();
    LOG_INFO("Planificacion detenida: pasos=", executedSteps_,
             ", nodos=", planner_ ? planner_->getTree().size() : 0);
}

// amgl // visual // Configura los obstaculos sin dejar un estado visual parcial.
void RRTWidget::SetObstacles(const std::vector<PolygonObstacle>& obstacles)
{
    LOG_INFO("Configurando obstaculos en el widget: total=", obstacles.size());
    timer_.stop();
    phaseTimer_.stop();
    stopElapsedTime();

    // Primero configurar PQP. Si falla, no cambiar la imagen.
    planner_->setObstacles(obstacles);
    obstacles_ = obstacles;

    velocityIntegrator_.setState(planner_->getOrigin());
    setVisualState(VisualState::Preparing);
    update();
    LOG_SUCCESS("Obstaculos configurados en el widget: total=", obstacles_.size());
}

// amgl // visual // Dibuja el relleno y contorno de los obstaculos del escenario.
void RRTWidget::drawObstacles()
{
    glColor4f(0.48f, 0.52f, 0.58f, 0.58f);
    glBegin(GL_TRIANGLES);
    for (const auto& obstacle : obstacles_) {
        for (const auto& triangle : obstacle.triangles) {
            for (const auto& point : triangle)
                glVertex2d(point.x, point.y);
        }
    }
    glEnd();

    glColor4f(0.16f, 0.19f, 0.24f, 0.90f);
    glLineWidth(2.0f);
    for (const auto& obstacle : obstacles_) {
        glBegin(GL_LINE_LOOP);
        for (const auto& point : obstacle.vertices)
            glVertex2d(point.x, point.y);
        glEnd();
    }
    glLineWidth(1.0f);
}

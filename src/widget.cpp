#include "widget.h"
#include "RRTPlanner.h"
#include <QOpenGLFunctions>
#include <QtMath>
#include <cmath>
#include <QPainter>
#include <algorithm>
#include <cstddef>
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

    velocityIntegrator_.setState(originQ);

    //cada timeGrow=30 ms nos conectamos al arbol llamando a growtree
    //para expandir el arbol
    connect(&timer_, &QTimer::timeout, this, &RRTWidget::growTree);
    timer_.start(timeGrowMs_);//**MEJORA:PONER EN VARIABLE
}

void RRTWidget::setPlanner(std::shared_ptr<IPlanner> planner)
{
    if (!planner) {
        qWarning("setPlanner: planner nulo");
        return;
    }

    planner_ = std::move(planner);
    velocityIntegrator_.setState(planner_->getOrigin());
    update();
}

//aqui inicializamos el opengl (solo se llama una vez) y se define el color de fonde (blanco en este caso)
void RRTWidget::initializeGL() {
    glClearColor(1, 1, 1, 1);
}

void RRTWidget::OriginTree(std::vector<float>& x, std::vector<float>& y, std::vector<float>& th,int Nrobots, float radio)
{

    if (Nrobots <= 0)
    {
        qWarning("OriginTree: NRobots <= 0");
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

    //Reset para la nueva posición de inicio (de otro modo lee la default)
    timer_.stop();
    timer_.start(timeGrowMs_); // mismo periodo que se usa ahora
    update();
}

void RRTWidget::GoalTree(std::vector<float>& x, std::vector<float>& y, std::vector<float>& th,int Nrobots, float radio)
{

    if (Nrobots <= 0) {
        qWarning("GoalTree: NRobots <= 0");
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
}

void RRTWidget::computeVelocities(float Tau, int Nrobots)
{
    (void)Nrobots;
    velocityIntegrator_.computeVelocities(Tau);
}


void RRTWidget::VelocitiesRobots(std::vector<float>& Vx, std::vector<float>& Vy, std::vector<float>& Wang, int Nrobots)
{
    std::vector<Velocities> velocities(Nrobots);
    for(int i=0;i<Nrobots; i++)
    {
        velocities[i]={Vx[i],Vy[i],Wang[i]};
    }
    velocityIntegrator_.setVelocities(velocities);
}

void RRTWidget::DrawMyNodes(bool All, bool FinalPath)
{
    drawAllNodes_=All;
    drawFinalPath_=FinalPath;
}


void RRTWidget::DistanceToTheGoal(float distance)//Definimos la distancia a la meta que consideramos suficiente para conectarla al arbol
{
    planner_->setGoalTolerance(distance);
}

void RRTWidget::ParamsTreeRRT(float Step, int MaximalNodes){//Definimos el tamaño de paso (step) y el numero maximo de nodos en el arbol
    planner_->setStepSize(Step);
    planner_->setMaxNodes(MaximalNodes);
}

void RRTWidget::SetTimeGrow(int Tgrow){//Tiempo de crecimiento en cada iteracion del arbol
    timeGrowMs_=Tgrow;
    timer_.start(timeGrowMs_);
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
    glViewport(0, 0, w, h);

    float worldW = 500.0f;//****
    float worldH = 500.0f;//****

    float aspectWindow = (float(w) / float(h));//****
    float aspectWorld  = (worldW / worldH);//****

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    if (aspectWindow > aspectWorld)//****
    {
        float newW = worldH * aspectWindow;//****
        float offset = (newW - worldW) / 2.0f;//****
        glOrtho(-offset, worldW + offset, worldH, 0, -1, 1);//****
    }
    else//****
    {
        float newH = worldW / aspectWindow;//****
        float offset = (newH - worldH) / 2.0f;//****
        glOrtho(0, worldW, worldH + offset, -offset, -1, 1);//****+++
    }

    glMatrixMode(GL_MODELVIEW);
}



//Aqui se redibuja el arbol
void RRTWidget::paintGL()
{   

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

        glClear(GL_COLOR_BUFFER_BIT);
        glLoadIdentity();


        // ---++++++++++++++++++++++++++++---

        // --- 1) Árbol completo: líneas por robot ---
        if (!tree.empty()) {
            // Asumimos que el tamaño de q en cada nodo = numRobots
            const int R = static_cast<int>(tree.front().q.size());
            for (int r = 0; r < R; ++r)// para cada robot en el nodo
            {
                auto [cr, cg, cb] = colorForRobot(r);//asignamos color al robot de cada nodo
                glColor3f(cr, cg, cb);
                glLineWidth(1.0f);
                glBegin(GL_LINES);
                for (const auto& node : tree) {
                    if (node.parent != -1) {
                        const auto &q_child  = node.q;
                        const auto &q_parent = tree[node.parent].q;
                        if (r < (int)q_child.size() && r < (int)q_parent.size()) {
                            glVertex2f(q_child[r].x,  q_child[r].y);
                            glVertex2f(q_parent[r].x, q_parent[r].y);


                            if(drawAllNodes_)
                            {
                                const float rn = 3;
                                glBegin(GL_LINE_LOOP);
                                for (int i = 0; i < 36; ++i) {
                                    float ang = i * 2.0f * M_PI / 36.0f;
                                    glVertex2f(q_child[r].x + rn * std::cos(ang), q_child[r].y + rn * std::sin(ang));
                                }
                            }
                        }
                    }
                }
                glEnd();
            }
        }



        // --- 2) Caminos finales por robot (si existe finalPath) ---
        if (!finalPath.empty()) {
            // robots presentes en el primer nodo del camino
            const int R = static_cast<int>(tree[finalPath.front()].q.size());
            for (int r = 0; r < R; ++r) {
                auto [cr, cg, cb] = colorForRobot(r);
                glColor3f(cr, cg, cb);
                glLineWidth(10.0f);
                glBegin(GL_LINE_STRIP);
                for (int idx : finalPath) {
                    const auto &q = tree[idx].q;
                    if (r < (int)q.size()) {
                        glVertex2f(q[r].x, q[r].y);
                    }

                    if(drawFinalPath_)
                    {
                        const float rnn = 3;
                        glBegin(GL_LINE_LOOP);
                        for (int i = 0; i < 36; ++i) {
                            float ang = i * 2.0f * M_PI / 36.0f;
                            glVertex2f(q[r].x + rnn * std::cos(ang), q[r].y + rnn * std::sin(ang));
                        }
                    }

                }
                glEnd();
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

        glClear(GL_COLOR_BUFFER_BIT);
        glLoadIdentity();

        // ---++++++++++++++++++++++++++++---

        // --- 1) Árbol completo: líneas por robot ---
        if (!tree.empty()) {
            // Asumimos que el tamaño de q en cada nodo = numRobots
            const int R = static_cast<int>(tree.front().q.size());
            for (int r = 0; r < R; ++r)// para cada robot en el nodo
            {
                auto [cr, cg, cb] = colorForRobot(r);//asignamos color al robot de cada nodo
                glColor3f(cr, cg, cb);
                glLineWidth(1.0f);
                glBegin(GL_LINES);
                for (const auto& node : tree) {
                    if (node.parent != -1) {
                        const auto &q_child  = node.q;
                        const auto &q_parent = tree[node.parent].q;
                        //if (r < (int)q_child.size() && r < (int)q_parent.size())
                        //{
                        //glVertex2f(q_child[r].x,  q_child[r].y);
                        //glVertex2f(q_parent[r].x, q_parent[r].y);

                        /*QTextStream out(stdout);
                        for(int i=0;i<numRobots;i++)
                        {
                            QTextStream out(stdout);
                            out<<"Xppp"<<i<<": "<<q_parent[i].x<<"\t Yppp"<<i<<": "<<q_parent[i].y<<Qt::endl;
                        }

                        for(int i=0;i<numRobots;i++)
                        {
                            QTextStream out(stdout);
                            out<<"PX"<<i<<": "<<Prev[i].x<<"\t PY"<<i<<": "<<Prev[i].y<<Qt::endl;
                        }*/

                        glVertex2f(Prev[r].x,Prev[r].y);
                        glVertex2f(Next[r].x,Next[r].y);

                        const float rne = 3;//radio del nodo

                        for (int i = 0; i < 36; ++i) {
                            float ang = i * 2.0f * M_PI / 36.0f;
                            glVertex2f(Next[r].x + rne * std::cos(ang), Next[r].y + rne * std::sin(ang));
                        }

                            if(drawAllNodes_)
                            {
                                const float rn = 3;//radio del nodo
                                //glBegin(GL_LINE_LOOP);
                                for (int i = 0; i < 36; ++i) {
                                    float ang = i * 2.0f * M_PI / 36.0f;
                                    glVertex2f(q_child[r].x + rn * std::cos(ang), q_child[r].y + rn * std::sin(ang));
                                }
                            }
                            //*******************ssssssssssssssssdfffffffffffff

                        //}
                    }
                }
                glEnd();
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
    QTextStream out(stdout);
    const int displayedPathNodes = planner_->getPathNodeCount();
    const int displayedRRTNodes = planner_->getNodeCount();
    // out << "Numero de nodos en el path: " << displayedPathNodes << Qt::endl;
    // out << "Numero de nodos en el path: " << (displayedRRTNodes+2) << Qt::endl;
}





void RRTWidget::EulerMult(float DeltaT)
{
    velocityIntegrator_.advance(DeltaT);
}

//Esta función se llama cada 30 ms en RRTWidget por default, pero toma el valor segun la función SetTimeGrow
void RRTWidget::growTree()
{
    if(!ActiveEuler)
    {
        if (planner_->step())
            timer_.stop();

        update();//refrescamos pantalla
        return;
    }

    if(velocityIntegrator_.needsSegment())
    {
        const std::size_t previousTreeSize = planner_->getTree().size();
        planner_->step();

        const auto &tree = planner_->getTree();
        if (tree.size() <= previousTreeSize)
            return;

        const auto &newNode = tree[previousTreeSize];
        if (newNode.parent < 0
            || newNode.parent >= static_cast<int>(tree.size())) {
            return;
        }

        const auto &qNear = tree[newNode.parent].q;
        const auto &qNew = newNode.q;
        velocityIntegrator_.beginSegment(qNear, qNew, 1.0f);

        /*QTextStream out(stdout);
        for(int i=0;i<numRobots;i++)
        {
            QTextStream out(stdout);
            out<<"Xp"<<i<<": "<<q_near[i].x<<"\t Yp"<<i<<": "<<q_near[i].y<<Qt::endl;
        }*/
    }





    EulerMult(0.1);
    update();//refrescamos pantalla
}

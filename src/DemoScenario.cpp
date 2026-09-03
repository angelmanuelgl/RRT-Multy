#include "DemoScenario.h"

#include "widget.h"

void configureDemoScenario(RRTWidget* widget)
{
    if (!widget) {
        return;
    }

    const int N = 3;
    float xi[N]  = { 200.0f, 210.0f, 240.0f };
    float yi[N]  = { 100.0f, 160.0f, 90.0f };
    float thi[N] = { 0.0f, 0.0f, 0.0f  };

    /*float xf[N]  = { 1000.0f, 1050.0f, 1020.0f };
    float yf[N]  = { 600.0f, 630.0f, 610.0f };
    float thf[N] = { 0.0f, 0.0f, 0.0f };*/

    float xf[N]  = { 200.0f, 410.0f, 440.0f };
    float yf[N]  = { 300.0f, 360.0f, 190.0f };
    float thf[N] = { 0.0f, 0.0f, 0.0f };

    //vectores de velocidad
    float Vxr[N] ={0.1f ,0.1f, 0.1f};
    float Vyr[N] ={0.1f ,0.1f, 0.1f};
    float Vangr[N] ={0.0f ,0.0f, 0.0f};

    widget->ActiveEuler=false;
    // widget->ActiveEuler=true;

    widget->OriginTree(xi,yi,thi,N,5);
    widget->GoalTree(xf,yf,thf,N,5);
    widget->VelocitiesRobots(Vxr,Vyr,Vangr,N);
    widget->DistanceToTheGoal(N*40);
    widget->ParamsTreeRRT(40,1000000);
    widget->DrawMyNodes(true,true);
    //widget->EulerMult(0.1);
    if(widget->ActiveEuler)
        widget->SetTimeGrow(2000);
    else
        widget->SetTimeGrow(10);
}

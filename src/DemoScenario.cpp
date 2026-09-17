#include "DemoScenario.h"
#include "widget.h"


void configureDemoScenario(RRTWidget* widget, const std::string& ruta)
{
    if (!widget) {
        return;
    }

    LOG_INFO("Iniciando carga del archivo: ", ruta);

    std::ifstream file(ruta);

    if (!file.is_open()) {
        LOG_ERROR("No se pudo abrir el archivo de escenarios: ", ruta);
        return;
    }

    int N = 0;
    if (!(file >> N) || N <= 0) {
        LOG_ERROR("Numero de robots invalido ");
        return;
    }

    // Usamos vectores dinamicos para N leido del archivo
    std::vector<float> xi(N), yi(N), thi(N);
    std::vector<float> xf(N), yf(N), thf(N);
    std::vector<float> Vxr(N), Vyr(N), Vangr(N);

    // Leer los datos por cada robot
    for (int i = 0; i < N; ++i) {
        if (!(file >> xi[i] >> yi[i] >> thi[i]
              >> xf[i] >> yf[i] >> thf[i]
              >> Vxr[i] >> Vyr[i] >> Vangr[i])) {
            LOG_ERROR("Error leyendo los datos del robot ", i , " en el archivo .in");
            return;
        }

        std::cout << xi[i] << yi[i] << thi[i]
             << xf[i] << yf[i] << thf[i]
             << Vxr[i] << Vyr[i] << Vangr[i];
    }

    file.close();

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

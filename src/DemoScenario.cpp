#include "DemoScenario.h"
#include "PolygonGeometry.h"
#include "widget.h"
#include <exception>

void configureDemoScenario(
    RRTWidget* widget,
    const std::string& ruta,
    const std::string& rutaObstaculos)
{
    if (!widget) {
        widget->StopPlanning();
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


    // CARGAR
    SceneData scene;
    if (!loadScene(ruta, scene)) {
        LOG_ERROR("Fallo al cargar el esceario (inicio, y final)");
        return;
    }


    std::vector<PolygonObstacle> obstacles;
    if( !loadPolygonObstacles(rutaObstaculos, obstacles)) {
        LOG_ERROR("Fallo al cargar obstaculos");
        return;
    }

    // internat que se guarden los obstaculos
    try {
        widget->SetObstacles(obstacles);
    } catch (const std::exception& ex) {
        LOG_ERROR("No se pudieron preparar los obstaculos: ", ex.what());
        return;
    }


    widget->ActiveEuler=false;
    // widget->ActiveEuler=true;

    widget->OriginTree(scene.xi, scene.yi, scene.thi, scene.numRobots, 5);
    widget->GoalTree(scene.xf, scene.yf, scene.thf, scene.numRobots, 5);
    widget->VelocitiesRobots(scene.Vxr, scene.Vyr, scene.Vangr, scene.numRobots);
    widget->DistanceToTheGoal(scene.numRobots * 40);

    widget->ParamsTreeRRT(40,1000000);
    widget->DrawMyNodes(true,true);

    //widget->EulerMult(0.1);
    if(widget->ActiveEuler)
        widget->SetTimeGrow(2000);
    else
        widget->SetTimeGrow(10);
}



/*
    CARGA DE ARCHIVOS
*/
bool loadScene(const std::string& ruta, SceneData& scene)
{
    LOG_INFO("Iniciando carga de la escena desde el archivo: ", ruta);

    std::ifstream file(ruta);
    if (!file.is_open()) {
        LOG_ERROR("No se pudo abrir el archivo de escena: ", ruta);
        return false;
    }

    int N = 0;
    if (!(file >> N) || N <= 0 || N > 1000) {
        LOG_ERROR("Número de robots inválido o fuera de rango: ", N);
        return false;
    }

    scene.numRobots = N;
    scene.xi.resize(N); scene.yi.resize(N); scene.thi.resize(N);
    scene.xf.resize(N); scene.yf.resize(N); scene.thf.resize(N);
    scene.Vxr.resize(N); scene.Vyr.resize(N); scene.Vangr.resize(N);

    // Leer los datos por cada robot
    for (int i = 0; i < N; ++i) {
        if (!(file >> scene.xi[i] >> scene.yi[i] >> scene.thi[i]
              >> scene.xf[i] >> scene.yf[i] >> scene.thf[i]
              >> scene.Vxr[i] >> scene.Vyr[i] >> scene.Vangr[i])) {
            LOG_ERROR("Error leyendo los datos del robot en el índice: ", i);
            return false;
        }
    }

    file.close();
    LOG_INFO("Escena cargada exitosamente. Total de robots procesados: ", N);
    return true;
}

bool loadPolygonObstacles(const std::string& path, std::vector<PolygonObstacle>& output)
{
    LOG_INFO("Iniciando carga de obstáculos desde el archivo: ", path);

    std::ifstream file(path);
    if (!file.is_open()) {
        LOG_ERROR("No se pudo abrir el archivo de obstáculos: ", path);
        return false;
    }

    int count = 0;
    if (!(file >> count) || count < 0 || count > 10000) {
        LOG_ERROR("Número de obstáculos inválido o fuera de rango: ", count);
        return false;
    }

    std::vector<PolygonObstacle> temporary;
    temporary.reserve(static_cast<std::size_t>(count));

    for (int i = 0; i < count; ++i) {
        PolygonObstacle obstacle;
        int vertexCount = 0;

        if (!(file >> obstacle.name >> vertexCount) || vertexCount < 3 || vertexCount > 1000) {
            LOG_ERROR("Cabecera inválida para el obstáculo en el índice ", i);
            return false;
        }

        obstacle.vertices.resize(static_cast<std::size_t>(vertexCount));

        for (auto& p : obstacle.vertices) {
            if (!(file >> p.x >> p.y)) {
                LOG_ERROR("Vértices incompletos en el obstáculo: ", obstacle.name);
                return false;
            }
        }

        try {
            preparePolygon(obstacle);
        } catch (const std::exception& ex) {
            LOG_ERROR("Error procesando polígono ", obstacle.name, ": ", ex.what());
            return false;
        }

        temporary.push_back(std::move(obstacle));
    }

    file >> std::ws;
    if (!file.eof()) {
        LOG_WARN("Advertencia: Se encontraron datos adicionales no procesados al final de ", path);
    }

    output = std::move(temporary);
    LOG_INFO("Obstáculos cargados exitosamente. Total procesados: ", output.size());
    return true;
}
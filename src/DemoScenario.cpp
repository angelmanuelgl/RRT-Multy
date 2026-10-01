#include "DemoScenario.h"
#include "PolygonGeometry.h"
#include "widget.h"
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <exception>
#include "Config.h"
#include "logger.h"

//  localiza archvios intentando /..
static std::string encontrarPath(const std::string& path, int max_niveles = 20)
{
    std::string intento = path;

    for (int i = 0; i < max_niveles; i++){
        std::ifstream file(intento);
        if ( file.is_open() ){

            LOG_DEBUG("Archivo localizado: solicitado=", path,
                      ", resuelto=", intento, ", nivel=", i);
            return intento;
        }

        LOG_TRACE("Ruta candidata no disponible: ", intento);
        // subir
        intento = "../" + intento;
    }

    LOG_ERROR("No se encontro el archivo ", path,
              " tras revisar ", max_niveles, " niveles");

    return "";
}

void configureDemoScenario(
    RRTWidget* widget,
    const std::string& ruta,
    const std::string& rutaObstaculos)
{
    if (!widget) {
        LOG_FATAL("No se puede configurar el escenario: widget nulo");
        return;
    }

    LOG_INFO("Configurando escenario RRT: escena=", ruta,
             ", obstaculos=", rutaObstaculos);

    // no usamos euler
    widget->ActiveEuler = false;
    widget->StopPlanning();

    const std::string scenePath = encontrarPath(ruta);
    const std::string obstaclesPath = encontrarPath(rutaObstaculos);



    LOG_INFO("Iniciando carga del archivo: ", scenePath);

    std::ifstream file(scenePath);

    if (!file.is_open()) {
        LOG_ERROR("No se pudo abrir el archivo de escenarios: ", scenePath);
        widget->StopPlanning(); // amgl // visual
        return;
    }

    int N = 0;
    if (!(file >> N) || N <= 0) {
        LOG_FATAL("Archivo de escenario corrupto: numero de robots invalido: ", N);
        widget->StopPlanning(); // amgl // visual
        return;
    }


    // CARGAR
    SceneData scene;
    if (!loadScene(scenePath, scene)) {
        LOG_FATAL("Escenario incompleto o corrupto: no se cargaron inicio y meta");
        widget->StopPlanning(); // amgl // visual
        return;
    }


    std::vector<PolygonObstacle> obstacles;
    if( !loadPolygonObstacles(obstaclesPath, obstacles)) {
        LOG_FATAL("Archivo de obstaculos incompleto o corrupto");
        widget->StopPlanning(); // amgl // visual
        return;
    }

    // intenar que se guarden los obstaculos
    try {
        widget->SetObstacles(obstacles);
    } catch (const std::exception& ex) {
        LOG_ERROR("No se pudieron preparar los obstaculos: ", ex.what());
        widget->StopPlanning(); // amgl // visual
        return;
    }


    widget->OriginTree(scene.xi, scene.yi, scene.thi, scene.numRobots, 5);
    widget->GoalTree(scene.xf, scene.yf, scene.thf, scene.numRobots, 5);
    widget->VelocitiesRobots(scene.Vxr, scene.Vyr, scene.Vangr, scene.numRobots);
    widget->DistanceToTheGoal(scene.numRobots * 40);

    widget->ParamsTreeRRT(40,1000000);
    widget->DrawMyNodes(true,true);

    LOG_INFO("Parametros iniciales: robots=", scene.numRobots,
             ", obstaculos=", obstacles.size(),
             ", paso_RRT=40, max_nodos=1000000");

    //widget->EulerMult(0.1);
    if(widget->ActiveEuler)
        widget->SetTimeGrow(2000);
    else
        widget->SetTimeGrow(10);

    LOG_SUCCESS("Simulacion RRT inicializada correctamente");
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
        LOG_FATAL("Archivo de escena corrupto: numero de robots fuera de rango: ", N);
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
            LOG_FATAL("Datos incompletos para el robot en el indice: ", i);
            return false;
        }
        LOG_DEBUG("Robot ", i,
                  ": inicio=(", scene.xi[i], ",", scene.yi[i], ",", scene.thi[i],
                  "), meta=(", scene.xf[i], ",", scene.yf[i], ",", scene.thf[i],
                  "), velocidad=(", scene.Vxr[i], ",", scene.Vyr[i], ",", scene.Vangr[i], ")");
    }

    file.close();
    LOG_SUCCESS("Escena cargada correctamente. Robots procesados: ", N);
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
        LOG_FATAL("Archivo de obstaculos corrupto: cantidad fuera de rango: ", count);
        return false;
    }

    LOG_DEBUG("Obstaculos declarados en el archivo: ", count);

    std::vector<PolygonObstacle> temporary;
    temporary.reserve(static_cast<std::size_t>(count));

    for (int i = 0; i < count; ++i) {
        PolygonObstacle obstacle;
        int vertexCount = 0;

        if (!(file >> obstacle.name >> vertexCount) || vertexCount < 3 || vertexCount > 1000) {
            LOG_FATAL("Cabecera invalida para el obstaculo en el indice ", i);
            return false;
        }

        obstacle.vertices.resize(static_cast<std::size_t>(vertexCount));

        for (auto& p : obstacle.vertices) {
            if (!(file >> p.x >> p.y)) {
                LOG_FATAL("Vertices incompletos en el obstaculo: ", obstacle.name);
                return false;
            }
        }

        try {
            preparePolygon(obstacle);
        } catch (const std::exception& ex) {
            LOG_FATAL("Geometria corrupta en el poligono ", obstacle.name,
                      ": ", ex.what());
            return false;
        }

        LOG_DEBUG("Obstaculo ", i, ": nombre=", obstacle.name,
                  ", vertices=", obstacle.vertices.size(),
                  ", triangulos=", obstacle.triangles.size());

        temporary.push_back(std::move(obstacle));
    }

    // ver si de caualiadad quedo algo mas
    file >> std::ws;
    if (!file.eof()) {
        LOG_WARN("Advertencia: Se encontraron datos adicionales no procesados al final de ", path);
    }

    output = std::move(temporary);
    LOG_SUCCESS("Obstaculos cargados correctamente. Total procesados: ", output.size());
    return true;
}

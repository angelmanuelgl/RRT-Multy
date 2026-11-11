#include "DemoScenario.h"
#include "PolygonGeometry.h"
#include "widget.h"
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QStringList>
#include <exception>
#include "Config.h"
#include "logger.h"
#include <QStringList>
#include <cmath>




// -- // -- // -- // -- // -- // -- // -- // -- // -- // -- -- // -- // -- // -- // -- // -- // -- // -- // -- // -- //
//                                                CONFIGURAR ESCENARIO
// -- // -- // -- // -- // -- // -- // -- // -- // -- // -- -- // -- // -- // -- // -- // -- // -- // -- // -- // -- //


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


    widget->OriginTreeWithRadii(scene.xi, scene.yi, scene.thi,
                                scene.numRobots, scene.radii);
    widget->GoalTreeWithRadii(scene.xf, scene.yf, scene.thf,
                              scene.numRobots, scene.radii);
    widget->VelocitiesRobots(scene.Vxr, scene.Vyr, scene.Vangr, scene.numRobots);
    widget->DistanceToTheGoal(DemoScenarioDefaults::goalTolerance);

    widget->ParamsTreeRRT(DemoScenarioDefaults::maxExtensionStep,
                          DemoScenarioDefaults::maxTreeNodes);
    widget->DrawMyNodes(true,true);

    LOG_INFO("Parametros iniciales: robots=", scene.numRobots,
             ", obstaculos=", obstacles.size(),
             ", paso_RRT=", DemoScenarioDefaults::maxExtensionStep,
             ", max_nodos=", DemoScenarioDefaults::maxTreeNodes,
             ", margen_meta=", DemoScenarioDefaults::goalTolerance);

    //widget->EulerMult(0.1);
    if(widget->ActiveEuler)
        widget->SetTimeGrow(DemoScenarioDefaults::eulerIntervalMs);
    else
        widget->SetTimeGrow(DemoScenarioDefaults::planningIntervalMs);

    LOG_SUCCESS("Simulacion RRT inicializada correctamente");
}



// -- // -- // -- // -- // -- // -- // -- // -- // -- // -- -- // -- // -- // -- // -- // -- // -- // -- // -- // -- //
//                                                AUXILIARES
// -- // -- // -- // -- // -- // -- // -- // -- // -- // -- -- // -- // -- // -- // -- // -- // -- // -- // -- // -- //



/*
 * INTENTAR LCOALIZAR LOS ARCHVIDO
*/

//  localiza archivos intentando /..
std::string encontrarPath(const std::string& path, int max_niveles)
{
    QString requested = QString::fromStdString(path);
    if (requested.isEmpty())
        return {};


    QStringList bases;
    bases << QDir::currentPath() << QCoreApplication::applicationDirPath();


    for (const QString& base : bases) {
        QDir directory(base);
        for (int level = 0; level <= max_niveles; ++level) {
            const QFileInfo candidate(directory.filePath(requested));
            if (candidate.isFile()) {
                const QString resolved = candidate.absoluteFilePath();
                LOG_INFO("--->Archivo localizado: solicitado=", path,
                         ", resuelto=", resolved.toStdString(),
                         ", base=", directory.absolutePath().toStdString());
                return resolved.toStdString();
            }
            if (!directory.cdUp())
                break;
        }
    }

    LOG_ERROR("No se encontro el archivo ", path,
              " desde el directorio de trabajo, el ejecutable ni el proyecto");
    return {};
}


/*
    CARGA DE ARCHIVOS
*/
bool loadScene(const std::string& ruta, SceneData& scene)
{
    LOG_INFO("INI carga de la escena desde el archivo: ", ruta);

    std::ifstream file(ruta);
    if (!file.is_open()) {
        LOG_ERROR("NO se pudo abrir el archivo de escena: ", ruta);
        return false;
    }

    int N = 0;
    if (!(file >> N) || N <= 0 || N > 1000) {
        LOG_FATAL("numero de robots fuera de rango: ", N);
        return false;
    }

    scene.numRobots = N;
    scene.xi.resize(N); scene.yi.resize(N); scene.thi.resize(N);
    scene.xf.resize(N); scene.yf.resize(N); scene.thf.resize(N);
    scene.Vxr.resize(N); scene.Vyr.resize(N); scene.Vangr.resize(N);
    scene.radii.resize(N, kDefaultRobotRadius);

    std::string line;
    std::getline(file, line);
    for (int i = 0; i < N; ++i) {
        std::vector<float> values;
        while (std::getline(file, line)) {
            const auto comment = line.find('#');
            if (comment != std::string::npos)
                line.erase(comment);
            std::istringstream row(line);
            float value = 0.0f;
            while (row >> value)
                values.push_back(value);
            if (!row.eof()) {
                LOG_FATAL("Valor invalido en la fila del robot ", i);
                return false;
            }
            if (!values.empty())
                break;
        }

        if (values.size() != 9 && values.size() != 10) {
            LOG_FATAL("Datos incompletos para el robot en el indice: ", i);
            return false;
        }
        scene.xi[i] = values[0];
        scene.yi[i] = values[1];
        scene.thi[i] = values[2];
        scene.xf[i] = values[3];
        scene.yf[i] = values[4];
        scene.thf[i] = values[5];
        scene.Vxr[i] = values[6];
        scene.Vyr[i] = values[7];
        scene.Vangr[i] = values[8];
        if (values.size() == 10)
            scene.radii[i] = values[9];
        if (!std::isfinite(scene.radii[i]) || scene.radii[i] <= 0.0f) {
            LOG_FATAL("Radio invalido para el robot ", i, ": ", scene.radii[i]);
            return false;
        }
        LOG_DEBUG("Robot ", i,
                  ": inicio=(", scene.xi[i], ",", scene.yi[i], ",", scene.thi[i],
                  "), meta=(", scene.xf[i], ",", scene.yf[i], ",", scene.thf[i],
                  "), velocidad=(", scene.Vxr[i], ",", scene.Vyr[i], ",", scene.Vangr[i],
                  "), radio=", scene.radii[i]);
    }

    file.close();
    LOG_SUCCESS("Escena cargada correctamente    Robots: ", N);
    return true;
}

bool loadPolygonObstacles(const std::string& path, std::vector<PolygonObstacle>& output)
{
    LOG_INFO("Iniciando carga de obstaculos desde el archivo: ", path);

    std::ifstream file(path);
    if (!file.is_open()) {
        LOG_ERROR("No se pudo abrir el archivo de obstaculos: ", path);
        return false;
    }

    int count = 0;
    if (!(file >> count) || count < 0 || count > 10000) {
        LOG_FATAL("cantidad fuera de rango: ", count);
        return false;
    }

    LOG_DEBUG("obstaculos declarados en el archivo: ", count);

    std::vector<PolygonObstacle> temporary;
    temporary.reserve(static_cast<std::size_t>(count));

    for (int i = 0; i < count; ++i) {
        PolygonObstacle obstacle;
        int vertexCount = 0;

        if (!(file >> obstacle.name >> vertexCount) || vertexCount < 3 || vertexCount > 1000) {
            LOG_FATAL("no se lee el formato adecuado para el obstaculo en el indice ", i);
            return false;
        }

        obstacle.vertices.resize(static_cast<std::size_t>(vertexCount));

        for (auto& p : obstacle.vertices) {
            if (!(file >> p.x >> p.y)) {
                LOG_FATAL("veritces incompletos en el obstaculo: ", obstacle.name);
                return false;
            }
        }

        try {
            preparePolygon(obstacle);
        } catch (const std::exception& ex) {
            LOG_FATAL("el poligono ", obstacle.name,
                      ": ", ex.what());
            return false;
        }

        LOG_DEBUG("Obstaculo ", i, ": nombre=", obstacle.name,
                  ", vertices=", obstacle.vertices.size(),
                  ", triangulos=", obstacle.triangles.size());

        temporary.push_back(std::move(obstacle));
    }

    // ver si de caualiadad quedo algo mas // para ver si eso es lo que daba error
    file >> std::ws;
    if (!file.eof()) {
        LOG_WARN("Advertencia: Se encontraron datos adicionales no procesados al final de ", path);
    }

    output = std::move(temporary);
    LOG_SUCCESS("Obstaculos cargados correctamente. Total procesados: ", output.size());
    return true;
}

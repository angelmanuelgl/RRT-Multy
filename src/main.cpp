#include "window.h"
#include "logger.h"

#include <QApplication>
#include <QCoreApplication>
#include <QDir>

#include <string>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // Cambia estos dos nombres para probar otro par de archivos en data/.
    // Tambien puedes escribir rutas absolutas.
    const std::string scenarioFile = "data/escenario3.in";
    const std::string obstaclesFile = "data/obstaculos3.in";

    LOG_INFO("El ejecutable se ejecuta desde:",
             QCoreApplication::applicationDirPath().toStdString());
    LOG_INFO("El Working Directory actual es:", QDir::currentPath().toStdString());
    LOG_INFO("Escenario seleccionado:", scenarioFile);
    LOG_INFO("Obstaculos seleccionados:", obstaclesFile);

    RRTWindow window(scenarioFile, obstaclesFile);
    window.show();
    LOG_SUCCESS("Aplicacion RRT lista; iniciando ciclo de eventos");
    return app.exec();
}

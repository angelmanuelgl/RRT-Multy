#include "window.h"
#include <QApplication>

#include <QCoreApplication>
#include <QDir>
#include "logger.h"


int main(int argc, char *argv[]) {
    QApplication a(argc, argv); //Inicializamos sistema grafico (eventos, ventanoas, timers, interacción usuario)

    // ruta absoluta
    QString exepath = QCoreApplication::applicationDirPath();
    std::string s_exepath = exepath.toStdString();
    LOG_INFO( "El ejecutable se ejecuta desde:" , s_exepath);

    // directorio
    QString actualpath = QDir::currentPath();
    std::string s_actualpath = actualpath.toStdString();
    LOG_INFO( "El Working Directory actual es:", s_actualpath);

    // la aplicaicon



    RRTWindow w;//crea ventana principal (entorno visual del RRT)
    w.show();//visualizar la ventana
    LOG_SUCCESS("Aplicacion RRT lista; iniciando ciclo de eventos");
    return a.exec();//bucle principal de eventos

    return 0;
}

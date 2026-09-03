#include "window.h"
#include <QApplication>

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);//Inicializamos sistema grafico (eventos, ventanoas, timers, interacción usuario)
    RRTWindow w;//crea ventana principal (entorno visual del RRT)
    w.show();//visualizar la ventana
    return a.exec();//bucle principal de eventos
}


#include "window.h"
#include "DemoScenario.h"
#include "widget.h"

//Constructor del RRTWindow, llama al contructor base QWidget *parent
//para inicializar con su posible padre (normalmente nullptr)
RRTWindow::RRTWindow(QWidget *parent) : QMainWindow(parent) {
    setWindowTitle("RRT");//Titulo de la ventana

    //Se crea un RRTWidget y se pone como "central widget"
    auto *w = new RRTWidget(this);
    setCentralWidget(w);
    //al construir el RRTWidget se inicializan las estructuras (arbol),
    //se crea y configura el QTimer interno para llamar periodicamente a growTree
    //al mostrar el widget (show) se hace el llamado a initializeGL, resize y paint en el ciclo del render
    //En conclusión SE INICIA Y EJECUTA EL SISTEMA DEL RRT y aque RRTWidget corre el tempoizador que le
    //permite crecer al arbol

    //AMGL// en DemoScenario.h estan las consiguraciones
    // posicion inicial
    // vectorces de velocidad
    // posiciones de origen
    // posiciones de meta
    // parametros RTT 
    configureDemoScenario(w);

    resize(500, 500);
}

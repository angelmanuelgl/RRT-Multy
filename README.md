# RRT multiagente con Qt 6

Aplicación en C++17 y Qt 6 para visualizar un árbol RRT para múltiples robots

* la interfaz utiliza `QOpenGLWidget` y `QTimer`
* el planificador RRT y el integrador Euler se encuentran desacoplados de Qt


### RESUMEN principal

* **`main.cpp`**: crea `QApplication` y la ventana principal `RRTWindow`.
* **`RRTWindow` (`window.h` / `window.cpp`)**: construye la ventana, instala `RRTWidget` como widget central y solicita la configuración del escenario de demostración
* **`DemoScenario` (`DemoScenario.h` / `DemoScenario.cpp`)**: carga las posiciones iniciales y finales, velocidades, opciones de dibujo y parámetros del escenario de tres robots de. archivo data/escenario.in
* **`RRTWidget` (`widget.h` / `widget.cpp`)**: mantiene la capa Qt, el temporizador y el renderizado OpenGL. Delega el árbol en `IPlanner` y el movimiento Euler en `VelocityIntegrator`
* **`IPlanner` (`IPlanner.h`)**: independiente de Qt: estructura general para marcar configurar, ejecutar y consultar un planificador
* **`RRTPlanner` (`RRTPlanner.h` / `RRTPlanner.cpp`)**: implementa el algoritmo RRT para multirobots heredando de IPlanner, solo usa bibliotecas estándar de C++
* **`VelocityIntegrator` (`VelocityIntegrator.h` / `VelocityIntegrator.cpp`)**: integra el movimiento sin depender de Qt y mantiene las velocidades y configuraciones de la simulación
* **`Confing` (`Confing.h` y `.Confing.cpp`)**: configuracion
* **`Obstacle` (`Obstacle.h`)**: 
* **`PolygonGeometry` (`PolygonGeometry.h` / `PolygonGeometry.cpp`)**: Validar y triangular, se encarga de que los obstaculos sean poligonos validos
* **`PQPCollision` (`PQPCollision.h` / `PQPCollision.cpp`)**: Usamos el repositorio[PQP - A Proximity Query Package](https://gamma.cs.unc.edu/SSV/)

## Estructura

```text
include/
  Config.h
  DemoScenario.h
  IPlanner.h
  PolygonGeoetry.h
  PQPCollisio.h
  RRTPlanner.h
  VelocityIntegrator.h
  widget.h
  window.h
src/
  DemoScenario.cpp
  PolygonGeoetry.cpp
  PQPCollisio.cpp
  RRTPlanner.cpp
  VelocityIntegrator.cpp
  main.cpp
  widget.cpp
  window.cpp

qt_rrt.pro
```

# Como usarlo?

## Escenario de prueba

El escenario que antes estaba dentro del constructor de `RRTWindow` fue movido a `configureDemoScenario()` en `DemoScenario.cpp`

`RRTWindow` ya no conoce directamente los parámetros numéricos del algoritmo es puramente de dibujos



## Obstaculos



# Informacion adicional

## Datos

`include/Config.h` contiene:

- `Config`: posicióßn cartesiana y orientación de un robot.
- `Velocities`: velocidades lineales y angular.
- `PlanNode`: configuración conjunta de los robots e índice de su nodo padre.
- `PlanSnapshot`: copia agrupada del árbol, camino, origen, meta y estado de finalización.

Estos tipos usan únicamente la biblioteca estándar de C++ y pueden incluirse en ejecutables sin Qt.

## Planificador mas general

`IPlanner` hace:

- Configurar origen, meta y número de robots.
- Configurar tam de paso, límite de nodos y tolerancia de llegada.
- Reiniciar el estado y ejecutar una iteración mediante `step()`.
- Consultar si la planificación terminó.
- Leer el árbol, camino final, origen, meta y telemetría sin modificar el estado interno.

La idea es que el GUI depende de esta interfaz y no de los detalles internos de `RRTPlanner`
asi estan independientes interfaz y algoritmo permitiendo incorporar otros algoritmos en el futuro.

## ARCHIVOS actuales

### Algoritmos C++

`RRTPlanner` contiene:

- `tree_` y `finalPath_`.
- `goalQ_` y `originQ_`.
- Parámetros del RRT y contadores de telemetría.
- Generación de muestras aleatorias.
- Cálculo de distancias.
- Selección del nodo más cercano.
- Operación de avance o `steer`.
- Inserción de nodos, detección de meta y backtracking del camino final.

`VelocityIntegrator` tiene las velocidades, las configuraciones anterior, actual y siguiente, 
el cálculo de velocidades entre configuraciones y el avance Euler. Tanto `RRTPlanner` como 
`VelocityIntegrator` usan solo C++ estándar y no incluyen ni utilizan clases de Qt para que sea totalmente independiente


### Capa Qt y renderizado (separado)

`RRTWidget` tiene sus métodos públicos originales para mantener compatibilidad. 


En modo no-Euler:

- Los métodos `OriginTree`, `GoalTree`, `ParamsTreeRRT` y `DistanceToTheGoal` validan o adaptan los datos y los delegan al planner.
- `growTree()` llama a `IPlanner::step()`, detiene el temporizador al alcanzar la meta y solicita el repintado.
- `paintGL()` obtiene árbol, camino, origen y meta mediante getters de solo lectura.
- Las preferencias `drawAllNodes_` y `drawFinalPath_` permanecen en la GUI.
- El intervalo configurado por `SetTimeGrow` continúa siendo responsabilidad del `QTimer`.

En modo Euler (EN PROCESO AUN...) se supone que el widget solicita al planner el primer tramo, 
lo entrega a `VelocityIntegrator` y después coordina sus avances desde el temporizador
`paintGL()` solo consulta referencias constantes y no modifica el estado del algoritmo ni de la simulación



## Cambios hechos

- Se movieron `Config`, `Velocities` y el nodo del árbol fuera de `widget.h`
- El nodo fue nombrado `PlanNode` para evitar colisiones con otros posibles algoritmos futuros
- `widget.h` ahora incluye `Config.h`s

- se cambiaron `randFloat`, `configDistance`, `steer` y `getNearest` como metodos privados de `RRTPlanner`

-  configuración, reinicio, getters y telemetría de `RRTPlanner`.


- `RRTWidget` recibió un `std::shared_ptr<IPlanner>` y un método `setPlanner()` 
- asi por defecto se crea una instancia de `RRTPlanner`.

- modo Euler conserva queda como anteriormente (EN PROCESO)

- creado `DemoScenario.h` y `DemoScenario.cpp`.
- se trasladaron ahi los arreglos de origen, meta y velocidades, junto con la configuración del ejemplo
- `window.cpp` ahora se limita a crear la ventana, instalar el widget y llamar a `configureDemoScenario()`
- Los nuevos archivos se añadieron a `qt_rrt.pro`.


- Se crearon `VelocityIntegrator.h` y `VelocityIntegrator.cpp` sin dependencias de Qt
- Se trasladaron las velocidades y las configuraciones `Next`, `Curr` y `Prev` al integrador
- `computeVelocities()`, `VelocitiesRobots()` y `EulerMult()` actúan ahora como fachadas delgadas hacia `VelocityIntegrator`
x

## Compilacion

El proyecto utiliza qmake y C++17:
    lo recomendable es abrirse `qt_rrt.pro` directamente desde Qt Creator y compilarse con un kit de Qt 6.

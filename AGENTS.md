# Instrucciones para el Asistente de IA (Codex / Claude) - Proyecto RRT/Qt6

Este proyecto es una aplicación en **C++17 y Qt6** que implementa un planificador de rutas (RRT multi-agente / Velocity Obstacles) con interfaz gráfica basada en Qt (`QOpenGLWidget` / `QTimer`).

## IMPORTANTE
Al finalizar cada fase de refactorización exitosa, actualiza el archivo CHANGELOG.md (o la sección correspondiente del README.md) detallando brevemente qué clases o funciones se movieron y el estado de la compilación. No crees archivos README secundarios en las subcarpetas del proyecto.

## 1. Reglas de Arquitectura y Estándares de Código
- **Separación de responsabilidades:** La lógica algorítmica pura (RRTPlanner, cálculos matemáticos, estructuras de datos) DEBE estar completamente desacoplada de la interfaz gráfica de Qt (`widget.cpp`, `window.cpp`).
- **Librerías:** Utiliza C++ estándar (`<vector>`, `<cmath>`, `<random>`, `<memory>`). La lógica del planificador no debe incluir dependencias de Qt (`QWidget`, `QPainter`, etc.).
- **Estilo de nombres:** 
  - Clases: `PascalCase` (ej. `RRTPlanner`, `Config`).
  - Métodos y variables locales: `camelCase` (ej. `stepSize`, `getNearestNode`).
  - Miembros privados: terminados en guion bajo (ej. `tree_`, `maxNodes_`).
- **Comentarios:** NUNCA elimines los comentarios existentes en el código al refactorizar.

## 2. Estructura de Archivos del Proyecto
- `include/Config.h`: Estructuras de datos puras (`Config`, `Velocities`, `PlanNode`, etc.).
- `include/IPlanner.h`: Interfaz abstracta para los planificadores.
- `include/RRTPlanner.h`: Implementación del algoritmo RRT.
- `include/widget.h` y `src/widget.cpp`: Capa exclusiva de renderizado y eventos GUI (Qt).
- `include/window.h` y `src/window.cpp`: Ventana principal de la aplicación.
- `qt_rrt.pro`: Archivo de configuración de qmake.

## 3. Restricciones Operativas para la IA
1. **Modificaciones incrementales:** No modifiques múltiples archivos de forma masiva a menos que se te indique explícitamente. Trabaja por componentes o fases.
2. **Código completo:** Cuando entregues código modificado, proporciona el archivo completo o el bloque exacto sin usar resúmenes de tipo `// ... resto del código igual`.
3. **Compatibilidad:** Todo el código nuevo debe compilar correctamente bajo C++17 y Qt6 usando qmake.
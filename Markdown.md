# Pendientes: Integracion  RRT-Multi con ORCA y Euler

Este archivo explica cómo funciona la extensión de nuestro árbol actualmente y los dos pasos que siguen para integrar **ORCA** y la simulación con **Euler**.

---

## 1. Lo que hacemos actualmente

Por ahora, la extensión del árbol se hace trazando una línea recta directa desde el nodo más cercano (`q_near`) hacia el punto aleatorio (`q_rand`). No se simula física ni esquive de obstáculos en este paso.

```text
/* --- --- ACTUALMENTE HACEMOS: --- ---

    q_rand <- RANDOM_STATE()
    q_nea <-  NEAREST_NEIGHBOR(q_rand, T)
    u <-  SELECT_INPUT_RECTA(q_rand, q_near)
    q_new <-  FINAL_DE_LA_RECTA(q_near, u, \delta t)

    T.ADD_VERTEX(q_new)
    T.ADD_EDGE(q_near, q_new, u)
*/
```

## 2. Posible paso intermedio

Como paso intermedio para probar la integración de ORCA y Euler, la idea es avanzar solo un paso corto $\delta t$. \
RRT propone la meta local y ORCA calcula la velocidad segura para dar ese único paso.


```text

qNear: RRT-Multy propone una meta local con su recta
                        |
                        V
    ORCA intenta conectar qNear con esa meta sin colisiones
                        |
                        V
    Euler simula esa conexion \delta t
                        |
                        V
    el estado realmente alcanzado es qNew
                        |
                        V
    RRT agrega qNew y guarda la trayectoria ORCA

```


Psudocodigo
```text
/* --- --- POSIBLE PASO INTERMEDIO: --- --- 

    q_rand <- RANDOM_STATE()
    q_near <- NEAREST_NEIGHBOR(q_rand, T)

    u <- ORCA(q_near, dirección hacia q_rand)
    q_new <- EULER(q_near, u, \delta t)

    T.ADD_VERTEX(q_new)
    T.ADD_EDGE(q_near, q_new, u
*/
```



## 3. La Meta (Implementación Final)
La meta es hacer una simulación completa durante un tiempo T. En lugar de dar un solo paso, ejecutamos un bucle donde:
Calculamos la velocidad deseada hacia q_rand.
ORCA nos da velocidades seguras esquivando vecinos u obstáculos.
Euler actualiza el estado paso a paso (Δt).
Guardamos toda la trayectoria calculada en el árbol si es válida.


```text

/* --- --- META: --- --- 
    q_rand <- RANDOM_STATE()
    q_near <- NEAREST_NEIGHBOR(q_rand, T)

    estado <- q_near
    trayectoria <- [estado]

    repetir durante T:

        velocidades_preferidas <- DIRECCION_HACIA(q_rand, estado)

        velocidades_seguras <- ORCA(
            estado,
            velocidades_preferidas,
            vecinos,
            obstaculos
        )

        estado <- EULER(
            estado,
            velocidades_seguras,
            delta_t
        )

        trayectoria.agregar(estado)

    q_new <- estado

    si la trayectoria es valida:
        T.ADD_VERTEX(q_new)
        T.ADD_EDGE(q_near, q_new, trayectoria)

*/

```
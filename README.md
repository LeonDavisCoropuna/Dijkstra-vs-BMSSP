# Implementación del Algoritmo BMSSP

## Integrantes

1. Davis Coropuna Leon Felipe
2. Mogollon Cáceres Sergio Daniel
3. Maldonado Casilla Braulio Nayap
4. Lupo Condori Avelino
5. Huaman Coaquira Luciana Jullisa

## Descripción General

El algoritmo **BMSSP (Bounded Multi-Source Shortest Path)** es una variante del problema clásico de caminos mínimos de fuente única (**SSSP**). Su objetivo es calcular las distancias más cortas desde un nodo fuente hacia todos los demás nodos en un grafo dirigido ponderado, con la particularidad de **acotar la exploración hasta una distancia máxima B**.

A diferencia de Dijkstra, que explora los vértices en orden creciente de distancia, BMSSP introduce una **estrategia de búsqueda acotada y jerárquica**, basada en dividir el espacio de búsqueda utilizando pivotes y reducir dinámicamente el conjunto de nodos “en frontera”. Esto puede reducir el costo de mantenimiento de estructuras de datos como la cola de prioridad, que es el principal cuello de botella en Dijkstra.

### Motivación

El algoritmo surge como alternativa a Dijkstra y Bellman-Ford en escenarios donde:

* Se necesita explorar caminos **solo hasta cierta distancia B**, evitando recorrer el grafo completo.
* Se busca optimizar el tiempo en **grafos dispersos** (sparse graphs) donde la frontera de Dijkstra puede llegar a contener muchos nodos.
* Se combinan ventajas de la **relajación por capas** (Bellman-Ford) con la **exploración dirigida por prioridad** (Dijkstra).


## Enfoque del Algoritmo BMSSP

El algoritmo combina las ideas de:

* **Relajación parcial:** Solo considera los nodos cuya distancia al origen está por debajo de un umbral ( B ).
* **Selección de pivote:** Usa una técnica de “median-of-three” para elegir nodos pivote en la frontera, dividiendo el problema en subproblemas más pequeños.
* **Búsqueda jerárquica:** Divide y conquista sobre el conjunto de nodos en la frontera, evitando mantener una cola de prioridad muy grande.
* **Delta-Stepping:** Implementa un esquema de colas con “cubetas” (buckets) para agrupar nodos por rangos de distancia y reducir la necesidad de operaciones de heap.

El proceso general es:

1. Inicializar las distancias desde la fuente con infinito y la distancia de la fuente como 0.
2. Definir un conjunto inicial de nodos frontera ( S ), comenzando solo con el nodo fuente.
3. Elegir un **pivote** a partir de la frontera para dividir el problema en dos subconjuntos:

   * Nodos cuyo camino mínimo está por debajo del umbral asociado al pivote.
   * Nodos restantes que requieren exploración adicional.
4. Aplicar **delta-stepping** para relajar eficientemente los caminos en los subconjuntos.
5. Repetir la partición recursiva hasta explorar todos los nodos alcanzables dentro del límite ( B ).






## Implementación en el Proyecto

El proyecto implementa BMSSP junto con Dijkstra para comparar su rendimiento.
La implementación se organiza en módulos:

### 1. Representación del Grafo

* Se utiliza una clase `Graph` basada en listas de adyacencia (`std::unordered_map<NodeID, std::vector<Edge>>`) para permitir grafos dispersos y acceso rápido a aristas salientes.
* Cada arista se modela con una estructura `Edge` que almacena el nodo destino y el peso.

### 2. Conjunto de Nodos (Frontera)

* Se define la clase `NodeSet` que encapsula los nodos activos en la frontera de exploración, permitiendo añadir, verificar y convertir el conjunto a un vector para operaciones de pivoteo.

### 3. Búsqueda BMSSP

* La función principal `BMSSP` implementa la lógica recursiva:

  * Selecciona el pivote usando el criterio **median-of-three**, que reduce la sensibilidad a valores extremos.
  * Llama a una función auxiliar basada en **delta-stepping** para relajar caminos dentro de un límite de distancia ( B ).
  * Divide los nodos en subconjuntos (`left` y `right`) dependiendo de si están por debajo o por encima del límite parcial y continúa recursivamente.
* La función `bmsspSingleSource` sirve como **punto de entrada**: inicializa distancias, la frontera con la fuente y arranca el proceso BMSSP.

### 4. Búsqueda Delta-Stepping

* Se introduce la clase `BucketQueue` para implementar una cola de prioridad basada en cubetas.
* Agrupa nodos por intervalos de distancia (delta) en lugar de ordenarlos completamente, logrando menor sobrecarga que un heap binario.
* Esta estructura se usa en `dijkstraDeltaStepping` para explorar eficientemente los nodos cercanos a la fuente.


## Flujo del Programa

1. **Generación de Grafos:**
   El `main` permite crear grafos de dos tipos:

   * **Aleatorios:** con número de aristas proporcional al número de nodos.
   * **Tipo cuadrícula (grid):** útil para pruebas de escalabilidad con estructura más regular.

2. **Ejecución de Benchmarks:**
   Para cada tamaño de grafo:

   * Se ejecuta Dijkstra desde el nodo 0 y se mide el tiempo.
   * Se ejecuta BMSSP desde el nodo 0 con un umbral ( B ) fijo y se mide el tiempo.

3. **Resultados:**
   Se registran los tiempos de ejecución para análisis comparativo.


## Algoritmos

1. **Dijkstra**

   - Algoritmo clásico para grafos ponderados sin aristas negativas.
   - Utiliza un heap de prioridad (min-heap) para seleccionar el nodo con la menor distancia estimada en cada paso.
   - Complejidad: (O((V+E) \log V)) usando heap.

2. **BMSSP (Bounded Multi-Source Shortest Path Algorithm)**

   - Variante de búsqueda de caminos mínimos basada en expansión de múltiples fuentes o heurísticas de priorización.
   - En la práctica puede comportarse similar a Dijkstra, pero el orden de exploración depende de su estrategia de “best-first”.

## Resultados

Se realizaron benchmarks generando grafos aleatorios y midiendo el tiempo de ejecución en función del número de nodos:

- **Gráfica 1 (nodos grandes, hasta 100000)**
  ![Grafica1](results/1000-1000-100000.png)

  - Observación: Para grafos muy grandes, BMSSP tiende a ser más lento que Dijkstra en promedio.
  - Ambos algoritmos muestran un crecimiento aproximadamente lineal con el número de nodos y aristas, pero con cierta variabilidad debido a la aleatoriedad del grafo.

- **Gráfica 2 (nodos pequeños, hasta 1000)**
  ![Grafica2](results/10-10-1000.png)

  - Observación: En grafos pequeños, ambos algoritmos tienen tiempos comparables, con BMSSP mostrando ligeros picos en algunos nodos.
  - Esto sugiere que para grafos pequeños, BMSSP y Dijkstra tienen un desempeño similar, mientras que Dijkstra es más estable.

## Ejecución del Benchmark

Para reproducir los benchmarks, siga estos pasos:

1. **Clonar el repositorio:**

   ```bash
   git clone https://github.com/LeonDavisCoropuna/Dijkstra-vs-BMSSP.git
   cd Dijkstra-vs-BMSSP
   ```

2. **Dar permisos de ejecución al script `run.sh`:**

   ```bash
   chmod +x run.sh
   ```

3. **Modificar el bucle del script `run.sh`** (si se desea personalizar el rango de nodos):

   Abra `run.sh` en un editor de texto y cambie el bucle `for` según el formato:

   ```bash
   for N in $(seq INICIO PASO FINAL); do
       ./benchmark_run $N
   done
   ```

   Por ejemplo, para ejecutar desde 1000 hasta 100000 en pasos de 1000:

   ```bash
   for N in $(seq 1000 1000 100000); do
       ./benchmark_run $N
   done
   ```

4. **Ejecutar el script:**

   ```bash
   ./run.sh
   ```

   Esto compilará el programa (`benchmark_run.cpp`) y ejecutará el benchmark para los tamaños de grafo especificados.

---

## Ejecución del Benchmark / Test

Para reproducir los ejemplos de BMSSP y Dijkstra usando `test.cpp`, siga estos pasos:

1. **Dar permisos de ejecución al script:**

```bash
chmod +x run_test.sh
```

2. **Ejecutar el script:**

```bash
./run_test.sh
```

Esto compilará `test.cpp` y ejecutará el programa, mostrando:

- Ejemplo de BMSSP en grafo simple.
  
  ![alt text](.docs/image.png)

- Ejemplo de Dijkstra.
  
  ![alt text](.docs/image2.png)

- Ejemplo de BMSSP en grafo tipo cuadrícula (grid).
 
  ![alt text](.docs/image-1.png)

- Parámetros recomendados para distintos tamaños de grafo.
  
  ![alt text](.docs/image-2.png)


## Conclusiones

* **Dijkstra** sigue siendo un algoritmo robusto y predecible, especialmente en grafos grandes con pesos positivos y relativamente densos.  
* **BMSSP** demuestra un mejor desempeño en **grafos dispersos**, ya que su exploración acotada reduce significativamente el número de vértices y aristas evaluadas, lo que lo hace más eficiente en este tipo de estructuras.  
* Aunque para aplicaciones generales **Dijkstra** mantiene la ventaja en consistencia y simplicidad de implementación, **BMSSP** se posiciona como una alternativa más competitiva cuando se trabaja con grafos dispersos o cuando se requiere optimizar recursos de cómputo.  

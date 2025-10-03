#include "benchmark.hpp"
#include <iostream>
#include <cmath>
#include <map>

// Ejemplo básico de BMSSP
void exampleBMSSP()
{
    Graph g;

    // Agregar aristas (source, destino, peso)
    g.addEdge(0, 1, 2.0);
    g.addEdge(0, 2, 5.0);
    g.addEdge(1, 3, 4.0);
    g.addEdge(2, 3, 1.0);
    g.addEdge(1, 4, 1.0);
    g.addEdge(3, 5, 3.0);
    g.addEdge(4, 5, 2.0);

    // Ejecutar BMSSP desde el nodo 0
    auto dhat = bmsspSingleSource(g, 0, 1000.0);

    // Contar nodos alcanzables
    int reachable = 0;
    for (const auto &[node, dist] : dhat)
    {
        if (dist < INFINITY)
            reachable++;
    }
    std::cout << "Reached " << reachable << " nodes\n";

    // Imprimir distancias más cortas
    std::cout << "Shortest distances from node 0:\n";
    for (int i = 0; i <= 5; ++i)
    {
        if (dhat[i] == INFINITY)
        {
            std::cout << "  Node " << i << ": unreachable\n";
        }
        else
        {
            std::cout << "  Node " << i << ": " << dhat[i] << "\n";
        }
    }
}

// Ejemplo de Dijkstra
void exampleDijkstra()
{
    Graph g;
    g.addEdge(0, 1, 2.0);
    g.addEdge(0, 2, 5.0);
    g.addEdge(1, 3, 4.0);
    g.addEdge(2, 3, 1.0);

    auto distances = dijkstra(g, 0);

    std::cout << "Dijkstra distances from node 0:\n";
    for (int i = 0; i <= 3; ++i)
    {
        std::cout << "  Node " << i << ": " << distances[i] << "\n";
    }
}

// Ejemplo de grafo tipo grid
void exampleBMSSPGridGraph()
{
    Graph g;
    int width = 3;
    int height = 3;

    // Crear edges para un grid 3x3
    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            int node = i * width + j;
            if (j < width - 1)
                g.addEdge(node, i * width + (j + 1), 1.0);
            if (i < height - 1)
                g.addEdge(node, (i + 1) * width + j, 1.0);
            if (j > 0)
                g.addEdge(node, i * width + (j - 1), 1.0);
            if (i > 0)
                g.addEdge(node, (i - 1) * width + j, 1.0);
        }
    }

    auto dhat = bmsspSingleSource(g, 0, 100.0);

    std::cout << "Grid distances from corner (0,0):\n";
    for (int i = 0; i < width * height; ++i)
    {
        std::cout << "  Node " << i << ": " << dhat[i] << "\n";
    }
}

// Ejemplo de ajuste de parámetros
void exampleParameterTuning()
{
    std::cout << "Small graph parameters:\n  l=1, k=50, t=1\n";
    std::cout << "Medium graph parameters:\n  l=2, k=100, t=1\n";
    std::cout << "Large graph parameters:\n  l=2, k=200, t=1\n";
    std::cout << "Structured graph parameters:\n  l=1, k=200, t=1\n";
}

int main()
{
    std::cout << "=== Example BMSSP ===\n";
    exampleBMSSP();
    std::cout << "\n=== Example Dijkstra ===\n";
    exampleDijkstra();
    std::cout << "\n=== Example BMSSP Grid Graph ===\n";
    exampleBMSSPGridGraph();
    std::cout << "\n=== Example Parameter Tuning ===\n";
    exampleParameterTuning();

    return 0;
}

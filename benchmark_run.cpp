#include "benchmark.hpp"   // Debe definir Graph, Edge, dijkstra, bmsspSingleSource
#include <chrono>
#include <iostream>
#include <cstdlib>
#include <random>
#include <string>
#include <cmath>

// Generar grafo aleatorio con numNodes nodos y numEdges aristas
Graph generateRandomGraph(int numNodes, int numEdges, double maxWeight, int seed = 42) {
    Graph graph;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> nodeDist(0, numNodes - 1);
    std::uniform_real_distribution<double> weightDist(1.0, maxWeight);

    int edgeCount = 0;
    while (edgeCount < numEdges) {
        int u = nodeDist(rng);
        int v = nodeDist(rng);
        if (u != v) { // evitar bucles
            double weight = weightDist(rng);
            graph.addEdge(u, v, weight);
            ++edgeCount;
        }
    }

    return graph;
}

// Generar grafo tipo cuadrícula (gridWidth x gridHeight)
Graph generateGridGraph(int gridWidth, int gridHeight) {
    Graph graph;
    int numNodes = gridWidth * gridHeight;

    for (int row = 0; row < gridHeight; ++row) {
        for (int col = 0; col < gridWidth; ++col) {
            int node = row * gridWidth + col;

            if (col < gridWidth - 1) graph.addEdge(node, row * gridWidth + (col + 1), 1.0); // derecha
            if (row < gridHeight - 1) graph.addEdge(node, (row + 1) * gridWidth + col, 1.0); // abajo
            if (col > 0) graph.addEdge(node, row * gridWidth + (col - 1), 1.0); // izquierda
            if (row > 0) graph.addEdge(node, (row - 1) * gridWidth + col, 1.0); // arriba
        }
    }

    return graph;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Uso: " << argv[0] << " NUM_NODOS [tipo: random|grid]\n";
        return 1;
    }

    int numNodes = std::atoi(argv[1]);
    std::string graphType = (argc >= 3) ? argv[2] : "random";

    Graph graph;
    if (graphType == "grid") {
        int width = static_cast<int>(std::sqrt(numNodes));
        int height = width;
        if (width * height < numNodes) height += 1;
        graph = generateGridGraph(width, height);
    } else {
        int numEdges = numNodes * 2; // número de aristas
        graph = generateRandomGraph(numNodes, numEdges, 10.0);
    }

    // Benchmark Dijkstra
    auto startTime = std::chrono::high_resolution_clock::now();
    auto distancesDijkstra = dijkstra(graph, 0);
    auto endTime = std::chrono::high_resolution_clock::now();
    auto dijkstraTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000.0;
    std::cout << "DIJKSTRA " << numNodes << " " << dijkstraTime << "\n";

    // Benchmark BMSSP
    startTime = std::chrono::high_resolution_clock::now();
    auto distancesBmssp = bmsspSingleSource(graph, 0, 1000.0);
    endTime = std::chrono::high_resolution_clock::now();
    auto bmsspTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000.0;
    std::cout << "BMSSP " << numNodes << " " << bmsspTime << "\n";

    return 0;
}

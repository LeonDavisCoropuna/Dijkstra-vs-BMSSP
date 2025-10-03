#include "benchmark.hpp"
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

// Generar coordenadas para nodos del grafo
std::unordered_map<NodeID, Coordinate> generateCoordinates(int numNodes, std::string graphType) {
    std::unordered_map<NodeID, Coordinate> coords;
    
    if (graphType == "grid") {
        int width = static_cast<int>(std::sqrt(numNodes));
        int height = width;
        if (width * height < numNodes) height += 1;
        
        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                int node = row * width + col;
                if (node < numNodes) {
                    coords[node] = {static_cast<double>(col), static_cast<double>(row)};
                }
            }
        }
    } else {
        std::mt19937 rng(42);
        std::uniform_real_distribution<double> coordDist(0.0, 1000.0);
        
        for (int i = 0; i < numNodes; ++i) {
            coords[i] = {coordDist(rng), coordDist(rng)};
        }
    }
    
    return coords;
}

// Generar múltiples fuentes cercanas (para BMSSP)
NodeSet generateSourceNodes(int numNodes, std::string graphType, int numSources) {
    NodeSet sources;
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> nodeDist(0, numNodes - 1);
    
    if (graphType == "grid") {
        int width = static_cast<int>(std::sqrt(numNodes));
        // Elegir una región pequeña en la cuadrícula
        int startRow = rng() % (width - 3);
        int startCol = rng() % (width - 3);
        for (int i = 0; i < numSources; ++i) {
            int row = startRow + (rng() % 3);
            int col = startCol + (rng() % 3);
            int node = row * width + col;
            if (node < numNodes) sources.add(node);
        }
    } else {
        // Para grafos aleatorios, elegir nodos aleatorios
        for (int i = 0; i < numSources; ++i) {
            sources.add(nodeDist(rng));
        }
    }
    
    return sources;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Uso: " << argv[0] << " NUM_NODOS [tipo: random|grid]\n";
        return 1;
    }

    int numNodes = std::atoi(argv[1]);
    std::string graphType = (argc >= 3) ? argv[2] : "random";

    Graph graph;
    double B = (graphType == "grid") ? 20.0 : 50.0; // Bounds más pequeños
    int numSources = 5; // Múltiples fuentes para BMSSP

    if (graphType == "grid") {
        int width = static_cast<int>(std::sqrt(numNodes));
        int height = width;
        if (width * height < numNodes) height += 1;
        graph = generateGridGraph(width, height);
    } else {
        int numEdges = numNodes * 2;
        graph = generateRandomGraph(numNodes, numEdges, 10.0);
    }

    auto coords = generateCoordinates(numNodes, graphType);
    int goalNode = numNodes - 1;
    NodeSet sources = generateSourceNodes(numNodes, graphType, numSources);

    // Benchmark Dijkstra (single-source)
    auto startTime = std::chrono::high_resolution_clock::now();
    auto distancesDijkstra = dijkstra(graph, 0);
    auto endTime = std::chrono::high_resolution_clock::now();
    auto dijkstraTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000.0;
    std::cout << "DIJKSTRA " << numNodes << " " << dijkstraTime << " ms\n";

    // Benchmark BMSSP (multi-source)
    startTime = std::chrono::high_resolution_clock::now();
    auto distancesBmssp = bmsspMultiSource(graph, sources, B);
    endTime = std::chrono::high_resolution_clock::now();
    auto bmsspTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000.0;
    std::cout << "BMSSP " << numNodes << " " << bmsspTime << " ms\n";

    // Benchmark A* Euclidean (single-source, single-target)
    auto heuristicEuclidean = createEuclideanHeuristic(coords);
    startTime = std::chrono::high_resolution_clock::now();
    auto distancesAStarEuclidean = astar(graph, 0, goalNode, heuristicEuclidean);
    endTime = std::chrono::high_resolution_clock::now();
    auto astarEuclideanTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000.0;
    std::cout << "ASTAR_EUCLIDEAN " << numNodes << " " << astarEuclideanTime << " ms\n";

    // Benchmark A* Manhattan (single-source, single-target)
    auto heuristicManhattan = createManhattanHeuristic(coords);
    startTime = std::chrono::high_resolution_clock::now();
    auto distancesAStarManhattan = astar(graph, 0, goalNode, heuristicManhattan);
    endTime = std::chrono::high_resolution_clock::now();
    auto astarManhattanTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000.0;
    std::cout << "ASTAR_MANHATTAN " << numNodes << " " << astarManhattanTime << " ms\n";

    return 0;
}

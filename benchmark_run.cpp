#include "benchmark.hpp"   // Debe definir Graph, Edge, dijkstra, bmsspSingleSource
#include <chrono>
#include <iostream>
#include <cstdlib>
#include <random>
#include <string>

// Generar grafo aleatorio con N nodos y M aristas
Graph generateRandomGraph(int N, int M, double maxWeight, int seed = 42) {
    Graph g;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> node_dist(0, N-1);
    std::uniform_real_distribution<double> weight_dist(1.0, maxWeight);

    int edgeCount = 0;
    while (edgeCount < M) {
        int u = node_dist(rng);
        int v = node_dist(rng);
        if (u != v) { // evitar bucles
            double w = weight_dist(rng);
            g.addEdge(u, v, w);
            ++edgeCount;
        }
    }

    return g;
}

// Generar grafo tipo cuadrícula (grid width x height)
Graph generateGridGraph(int width, int height) {
    Graph g;
    int N = width * height;

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            int node = i*width + j;

            if (j < width-1) g.addEdge(node, i*width + (j+1), 1.0); // derecha
            if (i < height-1) g.addEdge(node, (i+1)*width + j, 1.0); // abajo
            if (j > 0) g.addEdge(node, i*width + (j-1), 1.0); // izquierda
            if (i > 0) g.addEdge(node, (i-1)*width + j, 1.0); // arriba
        }
    }

    return g;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Uso: " << argv[0] << " NUM_NODOS [tipo: random|grid]\n";
        return 1;
    }

    int N = std::atoi(argv[1]);
    std::string type = (argc >= 3) ? argv[2] : "random";

    Graph g;
    if (type == "grid") {
        int width = static_cast<int>(std::sqrt(N));
        int height = width;
        if (width*height < N) height += 1;
        g = generateGridGraph(width, height);
    } else {
        int M = N * 2; // número de aristas
        g = generateRandomGraph(N, M, 10.0);
    }

    // Benchmark Dijkstra
    auto start = std::chrono::high_resolution_clock::now();
    auto dist1 = dijkstra(g, 0);
    auto end = std::chrono::high_resolution_clock::now();
    auto dijkstra_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    std::cout << "DIJKSTRA " << N << " " << dijkstra_time << "\n";

    // Benchmark BMSSP
    start = std::chrono::high_resolution_clock::now();
    auto dist2 = bmsspSingleSource(g, 0, 1000.0);
    end = std::chrono::high_resolution_clock::now();
    auto bmssp_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    std::cout << "BMSSP " << N << " " << bmssp_time << "\n";

    return 0;
}

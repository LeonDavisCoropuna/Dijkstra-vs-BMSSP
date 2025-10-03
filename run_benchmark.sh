#!/bin/bash
# run_benchmark.sh

echo "Ejecutando benchmarks..."
> resultados.txt  # Limpiar archivo

for nodes in  20000 40000 60000 80000 100000 200000 400000 600000 800000 1000000; do
    echo "Probando con $nodes nodos..."
    ./benchmark_run $nodes grid >> resultados.txt
done

echo "Resultados guardados en resultados.txt"
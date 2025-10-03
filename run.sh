#!/bin/bash

# Compile
g++ -O2 benchmark_run.cpp -o benchmark_run

# Loop start / step /  end
for N in $(seq 1000 1000 100000); do
    ./benchmark_run $N
done

#!/bin/bash

# Compilar
g++ -std=c++17 -O2 test.cpp benchmark.hpp -o test

if [ $? -eq 0 ]; then
    echo "Compilación exitosa. Ejecutando test..."
    ./test
else
    echo "Error en la compilación."
fi

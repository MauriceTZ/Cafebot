#!/bin/sh
# Compila el firmware para el PC con una planta simulada y corre las pruebas.
#   ./correr.sh               -> pruebas automáticas
#   ./correr.sh --interactivo -> firmware simulado por stdin/stdout (escribe "A 500", "P", ...)
cd "$(dirname "$0")" && g++ -std=c++17 -O1 -I. -include Arduino.h -x c++ sim.cpp -o /tmp/cafebot_sim && /tmp/cafebot_sim "$@"

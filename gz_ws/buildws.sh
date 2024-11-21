#!/usr/bin/env bash

if [ $# -eq 0 ]; then
    >&2 echo "No arguments provided"
    exit 1
fi

cmake -B "$1/src/micromouse/build" -S "$1/src/micromouse" && cmake --build "$1/src/micromouse/build"
cmake -B "$1/src/micromouse_system/build" -S "$1/src/micromouse_system" && cmake --build "$1/src/micromouse_system/build"
cmake -B "$1/src/encoder_sensor/build" -S "$1/src/encoder_sensor" && cmake --build "$1/src/encoder_sensor/build"
cmake -B "$1/src/encoder_sensor_system/build" -S "$1/src/encoder_sensor_system" && cmake --build "$1/src/encoder_sensor_system/build"

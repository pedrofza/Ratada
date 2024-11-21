#!/usr/bin/env bash

if [ $# -eq 0 ]; then
    >&2 echo "No arguments provided"
    exit 1
fi

rm -rf "$1/src/micromouse/build"
rm -rf "$1/src/micromouse/.cache"
rm -rf "$1/src/micromouse_system/build"
rm -rf "$1/src/micromouse_system/.cache"
rm -rf "$1/src/encoder_sensor/build"
rm -rf "$1/src/encoder_sensor/.cache"
rm -rf "$1/src/encoder_sensor_system/build"
rm -rf "$1/src/encoder_sensor_system/.cache"
rm -rf "$1/src/encoder_sensor_system/encoder_sensor"
rm -rf "$1/install"

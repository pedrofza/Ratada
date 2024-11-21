#!/usr/bin/env bash

if [ $# -eq 0 ]; then
    >&2 echo "No arguments provided"
    exit 1
fi

mkdir -p "$1/install/system"
mkdir -p "$1/install/gui"

cp "$1/src/micromouse/build/libMicromousePlugin.so" "$1/install/gui"
cp "$1/src/micromouse_system/build/libMicromouseSystem.so" "$1/install/system/"
cp "$1/src/encoder_sensor/build/libencoder.so" "$1/install/system/"
cp "$1/src/encoder_sensor_system/build/libEncoderSystem.so" "$1/install/system/"

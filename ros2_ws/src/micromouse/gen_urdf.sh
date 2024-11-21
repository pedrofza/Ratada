#!/usr/bin/env bash

if [ $# -eq 0 ]; then
    >&2 echo "No arguments provided"
    exit 1
fi

xacro "$1"/xacro/urdf/micromouse_robot.urdf.xacro -o "$1"/urdf/micromouse_robot.urdf

# gz_ws

## Introduction

gz_ws is the workspace for Gazebo plugins part of Ratada.

Source files are located in `src` and are installed to `install`.

- `encoder_sensor` is the implementation of an incremental encoder for Gazebo, as a custom sensor.
- `encoder_sensor_system` is the system that manages the encoder instances.
- `micromouse` is a GUI plugin for setting up and interacting with the simulation.
- `micromouse_system` is the underlying system that processes commands sent via the GUI plugin and keeps track of lap times.

There are helper scripts for building, installing and cleaning the entire workspace (also removes installed binaries).

## Quick Start

Make the scripts executable:

```sh
chmod +x buildws.sh
chmod +x installws.sh
chmod +x cleanws.sh
```

Build and install the plugins:

```sh
./buildws.sh . && ./installws.sh .
```

Point Gazebo to the plugin installation directories:

```sh
export GZ_GUI_PLUGIN_PATH=~/gz_ws/install/gui
export GZ_SIM_SYSTEM_PLUGIN_PATH=~/gz_ws/install/system
```

Run this if running Gazebo on Wayland ([source](https://gazebosim.org/docs/harmonic/troubleshooting/#wayland-issues)):

```sh
export QT_QPA_PLATFORM=xcb 
```

**Tip: you can add the commands above to `~/.bashrc` so they're run every time you open a terminal.**

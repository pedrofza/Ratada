# Micromouse

## Quick Start

Ensure this package has been built, installed and sourced.

Launch simulator with micromouse robot node controller

```sh
ros2 launch micromouse micromouse_launch.py
```

If not using the launch file provided and not starting running the micromouse package from the launch file:

```sh
ros2 run micromouse micromouse --ros-args -p use_sim_time:=true
```

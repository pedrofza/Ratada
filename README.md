# Ratada

![Screenshot of Ratada with RoboChallenge 2024 Day 3 maze](https://github.com/user-attachments/assets/3e1510fe-5551-4718-a904-39e6cc5a3e53)

## Introduction

Ratada is a 3D simulation environment for Micromouse robots.

## Supported Platform

Ratada is implemented as a set of plugins for Gazebo and ROS2.

It has been tested and developed with [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/) and [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) on an [Ubuntu 24.04 (Noble Numbat)](https://releases.ubuntu.com/noble/) virtual machine.

## Features

- Import robots from [URDF](http://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html) or [SDF](http://sdformat.org/spec?elem=sdf&ver=1.11) files
- Import Classic 16 x 16 mazes in [Micromouse Online's mazefiles repository](https://github.com/micromouseonline/mazefiles) file format
- Support for built-in [sensors](https://gazebosim.org/docs/harmonic/sensors/)
- Keeps track of solve times
- Implementation of wheel encoder sensor
- Example robot with range sensors, differential drive and wheel encoders
- Maze solving algorithm implementation based on the flood fill strategy

## Quick Start

This assumes an environment running Ubuntu 24.04, since ROS2 distributions are tightly tied to particular Ubuntu releases.

### Install ROS2

Follow the [ROS2 installation guide](http://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

### Install Gazebo

See [Installing Gazebo with ROS](https://gazebosim.org/docs/harmonic/ros_installation/) for detailed instructions.

The following command may be enough:

```sh
sudo apt-get install ros-jazzy-ros-gz
```

### Run the simulator

See the README files in the `gz_ws` and `ros_ws` folders for further instructions.

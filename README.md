# `hydra_sandbox`

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

Hydra Sandbox is a collection of experimental ROS 2 demos and utilities for
testing multi-robot task execution.

## Dependencies

This package has the following dependencies:

- [pyrobopath_ros](https://github.com/alexarbogast/pyrobopath_ros)
- [cartesian_planning](https://github.com/alexarbogast/cartesian_planning)

## Usage

Bring up the multi-robot system.

```sh
roslaunch hydra_sandbox bringup.launch
```

#### Pyrobopath Gcode Execution

Pyrobopath can be used to execute the toolpath from the gcode files in the
[resources folder](./resources/). The gcode file can be selected in the
[`scripts/gcode_execution`](./scripts/gcode_execution.py) script.

```sh
roslaunch hydra_sandbox gcode_execution.launch
```

#### Collision geometry visualization

Visualizing collision geometry can be useful when setting up
[`pyrobopath`](https://github.com/alexarbogast/pyrobopath). After bringing up
the robot system, visualize the collision geometry in rviz by running:

```sh
roslaunch hydra_sandbox visualize_collision_geometry
```

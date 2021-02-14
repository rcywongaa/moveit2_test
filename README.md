# Remy Test

## Assumptions
- Encoder values are sent as four-byte IEEE-754 floating point values with little endian
- Commands are represented in the same form as encoder values

## Build
### Docker
#### Base image
In project root directory:
```
sudo docker build -t remy_test:base -f docker/base.Dockerfile .
```

## Future Development
1. Use `ros-tooling` and [`industrial-ci`](https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#workspace-management)
to manage CI (See [example](https://github.com/ros-controls/ros2_control_demos/blob/master/.github/workflows/ci.yml))
1. Set up Gazebo simulation (classic, since ignition and `ros2_controls` isn't supported yet, [source](https://discourse.ros.org/t/announcing-ros2-control-for-foxy/18274/6))
1. Make `RobotController` comply with `ros2_control`
1. Make `RobotController` provide action interface


### Making `RobotController` comply with `ros2_control`
1. Fill out `ros2_control` specific interfaces in URDF (See [here](https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md))
1. Write the [`robot_hardware_interface.hpp`](https://github.com/ros-controls/ros2_control/blob/dashing/hardware_interface/include/hardware_interface/robot_hardware_interface.hpp)
1. Wire up `robot_hardware_interface` with the `ControllerManager` (See [here](https://github.com/ros-controls/ros2_control#writing-a-demo-for-your-own-robot))
1. Launch `ControllerManager`
1. Use `JointTrajectoryController` on the joints to control it (See [here](https://github.com/ros-controls/ros2_control_demos))

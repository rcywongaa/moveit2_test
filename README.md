# Remy Test

## Assumptions
- Encoder values are sent as four-byte IEEE-754 floating point values with little endian
- Commands are represented in the same form as encoder values
## Prerequisites
1. [Install ROS2 Dashing](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/)
1. Install dependencies
   ```
   sudo apt install \
   python3-colcon-common-extensions
   ```

## Steps
1. Set up minimal arm URDF in gazebo
1. Fill out `ros2_control` specific interfaces in URDF (See [here](https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md))
1. Write the [`robot_hardware_interface.hpp`](https://github.com/ros-controls/ros2_control/blob/dashing/hardware_interface/include/hardware_interface/robot_hardware_interface.hpp)
1. Wire up `robot_hardware_interface` with the `ControllerManager` (See [here](https://github.com/ros-controls/ros2_control#writing-a-demo-for-your-own-robot))
1. Launch `ControllerManager`
1. Use `JointTrajectoryController` on the joints to control it (See [here](https://github.com/ros-controls/ros2_control_demos))

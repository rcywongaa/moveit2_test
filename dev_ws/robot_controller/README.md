# Robot Controller

This package is responsible for taking in a joint trajectory,
interpolating and sending it to the arm 50Hz.

## Parameters
- `state_topic`: Topic name to publish joint states (`sensor_msgs::msg::JointState`)
- `joint_trajectory_topic`: Topic name to subscribe to for trajectories (`trajectory_msgs::msg::JointTrajectory`)

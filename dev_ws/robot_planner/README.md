# Robot Planner

This node subscribes for a point trajectory and converts it into a joint trajectory

## Parameters
- `joint_trajectory_topic`: Topic to publish joint trajectory (`trajectory_msgs::msg::JointTrajectory`)
- `ee_trajectory_topic`: Topic to subcribe for point trajectory (`robot_msgs::msg::PointTrajectory`)
- `robot_description`: URDF string for the robot

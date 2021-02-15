# Trajectory file reader

This node reads from a file, converts it into a `robot_msgs::msg::PointTrajectory` and publishes it.

## Parameters
- `trajectory_topic`: The topic to publish the point trajectory (`robot_msgs::msg::PointTrajectory`)
- `filename`: The filename containing the end effector trajectory

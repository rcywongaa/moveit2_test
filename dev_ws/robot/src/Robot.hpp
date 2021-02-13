#pragma once
#include <mutex>
#include <optional>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include "robot_msgs/msg/point_trajectory.hpp"

class Robot
{
  public:
    Robot(moveit::core::RobotModelPtr model);

    std::optional<trajectory_msgs::msg::JointTrajectory>
    create_joint_trajectory(robot_msgs::msg::PointTrajectory traj);

    void update(const sensor_msgs::msg::JointState::SharedPtr input);

  private:
    std::optional<trajectory_msgs::msg::JointTrajectory>
    calcIK(robot_msgs::msg::PointTrajectory traj);

    std::mutex mtx;
    moveit::core::RobotModelPtr kinematic_model;
    moveit::core::RobotStatePtr current_state;
};

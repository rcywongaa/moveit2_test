#pragma once
#include <mutex>

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

    // FIXME: Convert this into action server
    bool run(robot_msgs::msg::PointTrajectory traj);

    void update(const sensor_msgs::msg::JointState::SharedPtr input);

  private:
    std::mutex mtx;
    moveit::core::RobotModelPtr kinematic_model;
    moveit::core::RobotStatePtr current_state;
};

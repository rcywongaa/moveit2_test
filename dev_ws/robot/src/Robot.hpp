#pragma once

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "Connection.hpp"
#include "robot/msg/point_trajectory.hpp"

class Robot
{
  public:
    Robot(std::unique_ptr<Connection> connection, const robot_model_loader::RobotModelLoader& model_loader);

    bool run(robot::msg::PointTrajectory traj);

  private:
    std::array<float, 3> parse(std::vector<unsigned char>& data);

    std::unique_ptr<Connection> connection_;

    moveit::core::RobotModelPtr kinematic_model;
    moveit::core::RobotStatePtr current_state;

};

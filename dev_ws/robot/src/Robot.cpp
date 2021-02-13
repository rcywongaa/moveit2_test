#include <array>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "Robot.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Robot");

Robot::Robot(moveit::core::RobotModelPtr model)
  : kinematic_model(model)
{
  current_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
}

std::optional<trajectory_msgs::msg::JointTrajectory>
Robot::create_joint_trajectory(robot_msgs::msg::PointTrajectory traj)
{
  return calcIK(traj);
}

std::optional<trajectory_msgs::msg::JointTrajectory>
Robot::calcIK(robot_msgs::msg::PointTrajectory traj)
{
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  // FIXME: Take joint names as parameter
  joint_trajectory.joint_names = {"q1", "q2", "q3"};

  for (unsigned int i = 0; i < traj.points.size(); i++)
  {
    double x = traj.points[i].x;
    double y = traj.points[i].y;
    double z = traj.points[i].z;
    RCLCPP_INFO(LOGGER, "Calculating IK for (%f, %f, %f)", x, y, z);

    trajectory_msgs::msg::JointTrajectoryPoint joint_point;
    double q1 = 0.0;
    double q2 = 0.0;
    double q3 = 0.0;

    // HACK: Handle special case where singularity is required to achieve pose
    // Strangely, only singularity in q2 fails to solve, singularity for q3 can be solved
    // e.g. (x,y,z) = (18.66,0.0,5.0) corresponding to (q1, q2, q3) = (0.0, 30.0, 0.0) can be solved
    if (x*x + y*y == 20*20)
    {
      q1 = atan2(y, x);
      q2 = 0.0;
      q3 = 0.0;
      RCLCPP_INFO(LOGGER, "Singularity required! (q1, q2, q3) = (%f, %f, %f)", q1, q2, q3);
    }
    else
    {
      const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
      Eigen::Isometry3d end_effector_state = Eigen::Isometry3d::Identity();
      end_effector_state.translate(Eigen::Vector3d(x, y, z));

      moveit::core::RobotStatePtr desired_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
      bool is_found_ik = desired_state->setFromIK(joint_model_group, end_effector_state);
      if (is_found_ik)
      {
        q1 = *(desired_state->getJointPositions("q1"));
        q2 = *(desired_state->getJointPositions("q2"));
        q3 = *(desired_state->getJointPositions("q3"));
        RCLCPP_INFO(LOGGER, "Found IK solution: %f, %f, %f", q1, q2, q3);
      }
      else
      {
        RCLCPP_INFO(LOGGER, "Failed to find IK solution!");
        return {};
      }
    }
    joint_point.positions = {q1, q2, q3};
    joint_point.time_from_start = traj.time_from_start[i];
    joint_trajectory.points.push_back(joint_point);
  }
  return joint_trajectory;
}

void Robot::update(const sensor_msgs::msg::JointState::SharedPtr input)
{
  std::scoped_lock<std::mutex> lock(mtx);
  for (unsigned int i = 0; i < input->name.size(); i++)
  {
    current_state->setJointPositions(input->name[i], &(input->position[i]));
  }
}

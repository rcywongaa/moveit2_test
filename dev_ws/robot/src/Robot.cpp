#include <array>

#include <rclcpp/rclcpp.hpp>

#include "Robot.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Robot");

Robot::Robot(moveit::core::RobotModelPtr model)
  : kinematic_model(model)
{
  current_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
}

bool Robot::run(robot_msgs::msg::PointTrajectory traj)
{
  for (geometry_msgs::msg::PointStamped point_stamped : traj.trajectory)
  {
    std::scoped_lock<std::mutex> lock(mtx);
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    Eigen::Isometry3d end_effector_state = Eigen::Isometry3d::Identity();
    end_effector_state.translate(Eigen::Vector3d(
          point_stamped.point.x,
          point_stamped.point.y,
          point_stamped.point.z));
    double timeout = 0.1;
    bool is_found_ik = current_state->setFromIK(joint_model_group, end_effector_state, timeout);
    if (is_found_ik)
    {
      // execute IK
      ;
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Failed to find IK solution!");
      return false;
    }
  }
  return true;
}

void Robot::update(const sensor_msgs::msg::JointState::SharedPtr input)
{
  std::scoped_lock<std::mutex> lock(mtx);
  for (unsigned int i = 0; i < input->name.size(); i++)
  {
    current_state->setJointPositions(input->name[i], &(input->position[i]));
  }
}

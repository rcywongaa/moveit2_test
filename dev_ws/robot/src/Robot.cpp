#include "Robot.hpp"

#include <array>

Robot::Robot(std::unique_ptr<Connection> connection, const robot_model_loader::RobotModelLoader& model_loader) :
  connection_(std::move(connection))
{
  // Set up MoveIt kinematic model
  kinematic_model = model_loader.getModel();
  current_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
}

bool Robot::run(robot::msg::PointTrajectory traj)
{
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
  const Eigen::Isometry3d& end_effector_state = current_state->getGlobalLinkTransform("ee");
  double timeout = 0.1;
  bool is_found_ik = current_state->setFromIK(joint_model_group, end_effector_state, timeout);
  if (is_found_ik)
  {
    // execute IK
    ;
  }
}

std::array<float, 3> Robot::parse(std::vector<unsigned char>& data)
{
  // Parse 12 unsigned chars into 3 joint values
  // Each joint value is 4 unsigned char
  return {0, 0, 0};
}

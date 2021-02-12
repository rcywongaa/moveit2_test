#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/point_trajectory.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "Robot.hpp"

class RobotNode : public rclcpp::Node
{
  public:
    RobotNode() :
      rclcpp::Node("RobotNode")
    {
      RCLCPP_INFO(this->get_logger(), "RobotNode created!");
    }

    void init()
    {
      RCLCPP_INFO(this->get_logger(), "RobotNode initialized!");
      robot_model_loader::RobotModelLoader model_loader(shared_from_this(), std::string("robot_description"));
      traj_sub = this->create_subscription<robot_msgs::msg::PointTrajectory>(
          declare_and_get_parameter("trajectory_topic").as_string(),
          1,
          [&](const robot_msgs::msg::PointTrajectory::SharedPtr msg)
          {
            RCLCPP_INFO(this->get_logger(), "Received trajectory!");
            robot->run(*msg);
          });
      robot = std::make_unique<Robot>(model_loader.getModel());
    }

  private:
    rclcpp::Parameter declare_and_get_parameter(std::string param_name)
    {
      this->declare_parameter(param_name);
      return this->get_parameter(param_name);
    }
    std::unique_ptr<Robot> robot;

    rclcpp::Subscription<robot_msgs::msg::PointTrajectory>::SharedPtr traj_sub;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<RobotNode> node = std::make_shared<RobotNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
}

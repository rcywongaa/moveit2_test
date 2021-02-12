#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot/msg/point_trajectory.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "Robot.hpp"

class RobotNode : public rclcpp::Node
{
  public:
    RobotNode() :
      Node("RobotNode")
  {
    std::string input_file = declare_and_get_parameter("input_file").as_string();
    robot_model_loader::RobotModelLoader model_loader(shared_from_this(), std::string("robot_description"));
    robot = std::make_unique<Robot>(model_loader);
  }

  private:
    rclcpp::Parameter declare_and_get_parameter(std::string param_name)
    {
      this->declare_parameter(param_name);
      return this->get_parameter(param_name);
    }
    std::unique_ptr<Robot> robot;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotNode>());
  rclcpp::shutdown();
}

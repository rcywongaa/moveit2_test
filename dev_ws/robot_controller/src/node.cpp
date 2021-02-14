#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "RobotController.hpp"
#include "Connection.hpp"

using namespace std::chrono_literals;

class PrintingConnection : public Connection
{
  public:
    PrintingConnection()
      : logger(rclcpp::get_logger("PrintingConnection"))
    {}

    virtual int open() override
    {
      RCLCPP_INFO(logger, "open() called!");
      return 0;
    }

    virtual int close() override
    {
      RCLCPP_INFO(logger, "close() called!");
      return 0;
    }

    virtual int send(std::vector<unsigned char>& data) override
    {
      RCLCPP_INFO(logger, "----- send -----\n%x %x %x %x\n%x %x %x %x\n%x %x %x %x",
          data[0], data[1], data[2], data[3],
          data[4], data[5], data[6], data[7],
          data[8], data[9], data[10], data[11]);
      RCLCPP_INFO(logger, "----------");
      return 0;
    }

    virtual int receive(std::vector<unsigned char>& data) override
    {
      data = {
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00
      };

      //RCLCPP_INFO(logger, "receive(\n%x %x %x %x\n%x %x %x %x\n%x %x %x %x",
          //data[0], data[1], data[2], data[3],
          //data[4], data[5], data[6], data[7],
          //data[8], data[9], data[10], data[11]);
      return 0;
    }

  private:
    rclcpp::Logger logger;
};

class RobotControllerNode : public rclcpp::Node
{
  public:
    RobotControllerNode() : rclcpp::Node("RobotControllerNode")
    {
      RCLCPP_INFO(this->get_logger(), "RobotControllerNode created!");

      interface_ = std::make_unique<RobotController>(std::make_unique<PrintingConnection>());

      state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
          declare_and_get_parameter("state_topic").as_string(),
          1);
      joint_trajectory_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
          declare_and_get_parameter("joint_trajectory_topic").as_string(),
          1,
          [&](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
          {
            RCLCPP_INFO(this->get_logger(), "Received joint trajectory!");
            interface_->set_trajectory(*msg);
          });

      timer_ = this->create_wall_timer(
          20ms, std::bind(&RobotControllerNode::timer_callback, this));
    }

    void timer_callback()
    {
      interface_->spin_once();
      state_publisher_->publish(interface_->get_joint_state());
    }

  private:
    rclcpp::Parameter declare_and_get_parameter(std::string param_name)
    {
      this->declare_parameter(param_name);
      return this->get_parameter(param_name);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<RobotController> interface_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_subscriber_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotControllerNode>());
  rclcpp::shutdown();
}

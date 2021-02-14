#pragma once

#include <memory>
#include <array>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "Connection.hpp"

// TODO: This should eventually be modified to comply with ros2_control
class RobotController
{
  private:
    static const unsigned int NUM_JOINTS = 3;
    static const unsigned int DATA_SIZE = 12;
    static const unsigned int VALUE_PER_REVOLUTION = 4096; // 12 bits per revolution

  public:
    using JointData = std::array<float, NUM_JOINTS>;

    RobotController(std::unique_ptr<Connection> connection);
    ~RobotController();
    // We assume non-copyable for now
    RobotController(const RobotController &) = delete; // Copy constructor
    RobotController(RobotController &&) noexcept = delete; // Move constructor
    RobotController& operator=(const RobotController &) = delete; // Copy assignment
    RobotController& operator=(RobotController &&) noexcept = delete; // Move assignment

    void set_trajectory(trajectory_msgs::msg::JointTrajectory trajectory);

    sensor_msgs::msg::JointState get_joint_state();

    void spin_once();

  private:
    // JointState position data are represented in the form of [-pi, pi]
    static JointData parse(std::vector<unsigned char>& data);
    static std::vector<unsigned char> encode(JointData input);

    void send_setpoint(trajectory_msgs::msg::JointTrajectoryPoint setpoint);
    void receive_joint_data();

    std::mutex trajectory_mtx;
    std::mutex joint_angles_mtx;
    std::unique_ptr<Connection> connection_;
    sensor_msgs::msg::JointState joint_angles_;

    std::optional<trajectory_msgs::msg::JointTrajectory> trajectory_;
    std::optional<rclcpp::Time> trajectory_start_time_;
};

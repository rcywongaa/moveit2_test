#pragma once

#include <memory>
#include <array>
#include <mutex>

#include "Connection.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

// TODO: This should eventually be modified to comply with ros2_control
class RobotHwInterface
{
  private:
    static const unsigned int NUM_JOINTS = 3;
    static const bool IS_BIG_ENDIAN = true;
    static const unsigned int DATA_SIZE = 12;
    static const unsigned int BYTES_PER_JOINT = 4;
    static const unsigned int VALUE_PER_REVOLUTION = 4096; // 12 bits per revolution

  public:
    using JointData = std::array<double, NUM_JOINTS>;
    // JointState position data are represented in the form of [-pi, pi]
    static JointData parse(std::vector<unsigned char>& data);
    static std::vector<unsigned char> encode(JointData input);

    RobotHwInterface(std::unique_ptr<Connection> connection);
    ~RobotHwInterface();
    // We assume non-copyable for now
    RobotHwInterface(const RobotHwInterface &) = delete; // Copy constructor
    RobotHwInterface(RobotHwInterface &&) noexcept = delete; // Move constructor
    RobotHwInterface& operator=(const RobotHwInterface &) = delete; // Copy assignment
    RobotHwInterface& operator=(RobotHwInterface &&) noexcept = delete; // Move assignment

    void set_setpoint(sensor_msgs::msg::JointState setpoint);

    sensor_msgs::msg::JointState get_joint_state();

    void spin_once();

  private:
    void send_setpoint();
    void receive_joint_data();

    std::mutex setpoint_mtx;
    std::mutex joint_angles_mtx;
    std::unique_ptr<Connection> connection_;
    sensor_msgs::msg::JointState joint_angles_;
    sensor_msgs::msg::JointState setpoint_;

};

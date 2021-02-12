#include "RobotHwInterface.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

RobotHwInterface::JointData RobotHwInterface::parse(std::vector<unsigned char>& data)
{
  std::array<double, NUM_JOINTS> ret;
  unsigned int data_idx = 0;
  for (unsigned int joint_idx = 0; joint_idx < 3; joint_idx++)
  {
    unsigned int joint_angle_int = 0;
    for (unsigned int byte_idx = 0; byte_idx < BYTES_PER_JOINT; byte_idx++, data_idx++)
    {
      unsigned int byte_shift = IS_BIG_ENDIAN ? BYTES_PER_JOINT - byte_idx : byte_idx;
      joint_angle_int |= (data[data_idx] << byte_shift);
    }
    double joint_position = double(joint_angle_int) / double(VALUE_PER_REVOLUTION) * 2*M_PI;
    // convert from range [0, 2*pi] to [-pi, pi]
    if (joint_position > M_PI)
    {
      joint_position -= 2*M_PI;
    }
    ret[joint_idx] = joint_position;
  }
  return ret;
}

std::vector<unsigned char> RobotHwInterface::encode(RobotHwInterface::JointData input)
{
  std::vector<unsigned char> ret;
  ret.resize(DATA_SIZE);
  unsigned int data_idx = 0;
  for (unsigned int joint_idx = 0; joint_idx < input.size(); joint_idx++)
  {
    double joint_position = input[joint_idx];
    // convert from range [-pi, pi] to [0, 2*pi]
    if (joint_position < 0)
    {
      joint_position += 2*M_PI;
    }
    unsigned int position_int = joint_position / (2*M_PI) * double(VALUE_PER_REVOLUTION);
    for (unsigned int byte_idx = 0; byte_idx < BYTES_PER_JOINT; byte_idx++, data_idx++)
    {
      unsigned int byte_shift = IS_BIG_ENDIAN ? BYTES_PER_JOINT - byte_idx : byte_idx;
      ret[byte_idx] |= (position_int << byte_shift);
    }
  }
  return ret;
}

RobotHwInterface::RobotHwInterface(std::unique_ptr<Connection> connection)
  : connection_(std::move(connection))
{
  // TODO: Get joint names from parameter
  // robot_model.getActiveJointModels() or robot_model.getJointModelNames()
  joint_angles_.name = {"q1", "q2", "q3"};
  joint_angles_.position = {0, 0, 0};

  //FIXME: Error checking
  connection_->open();
}

RobotHwInterface::~RobotHwInterface()
{
  //FIXME: Error checking
  connection_->close();
}

void RobotHwInterface::set_setpoint(sensor_msgs::msg::JointState setpoint)
{
  std::scoped_lock<std::mutex> lock(setpoint_mtx);
  setpoint_ = setpoint;
}

sensor_msgs::msg::JointState RobotHwInterface::get_joint_state()
{
  std::scoped_lock<std::mutex> lock(joint_angles_mtx);
  return joint_angles_;
}

void RobotHwInterface::spin_once()
{
  send_setpoint();
  receive_joint_data();
}

void RobotHwInterface::send_setpoint()
{
  std::scoped_lock<std::mutex> lock(setpoint_mtx);
  JointData joint_data;
  std::copy(setpoint_.position.begin(), setpoint_.position.end(), joint_data.begin());
  std::vector<unsigned char> data = encode(joint_data);
  //FIXME: Error checking
  connection_->send(data);
}

void RobotHwInterface::receive_joint_data()
{
  std::vector<unsigned char> data;
  //FIXME: Error checking
  connection_->receive(data);
  JointData joint_data = parse(data);
  std::scoped_lock<std::mutex> lock(joint_angles_mtx);
  std::copy(joint_data.begin(), joint_data.end(), joint_angles_.position.begin());
}

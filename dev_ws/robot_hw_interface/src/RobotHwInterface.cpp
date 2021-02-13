#include "RobotHwInterface.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("RobotHwInterface");

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
      ret[data_idx] = (position_int << byte_shift);
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

void RobotHwInterface::set_trajectory(trajectory_msgs::msg::JointTrajectory trajectory)
{
  std::scoped_lock<std::mutex> lock(trajectory_mtx);
  trajectory_ = trajectory;
}

sensor_msgs::msg::JointState RobotHwInterface::get_joint_state()
{
  std::scoped_lock<std::mutex> lock(joint_angles_mtx);
  return joint_angles_;
}

void RobotHwInterface::spin_once()
{
  receive_joint_data();

  std::scoped_lock<std::mutex> lock(trajectory_mtx);
  if (!trajectory_)
  {
    return;
  }
  if (!trajectory_start_time_)
  {
    trajectory_start_time_ = rclcpp::Clock().now();
  }
  
  rclcpp::Duration time_from_start = rclcpp::Clock().now() - *trajectory_start_time_;
  // Find the current active segment
  // identified by trajectory_->position[end_segment_idx-1] to trajectory_->position[end_segment_idx]
  // Start at 1 because trajectory point at 0 will always have time 0
  unsigned int end_segment_idx = 1;
  for (end_segment_idx = 1;
      end_segment_idx < trajectory_->points.size()
      && rclcpp::Duration(trajectory_->points[end_segment_idx].time_from_start) < time_from_start;
      end_segment_idx++);

  if (end_segment_idx >= trajectory_->points.size())
  {
    RCLCPP_INFO(LOGGER, "Trajectory completed!");
    trajectory_start_time_.reset();
    trajectory_.reset();
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Time: %f, segment: %d", time_from_start.seconds(), end_segment_idx);
    trajectory_msgs::msg::JointTrajectoryPoint interpolated_point;
    std::vector<double> start_point = trajectory_->points[end_segment_idx-1].positions;
    std::vector<double> end_point = trajectory_->points[end_segment_idx].positions;
    rclcpp::Duration start_time = trajectory_->points[end_segment_idx-1].time_from_start;
    rclcpp::Duration end_time = trajectory_->points[end_segment_idx].time_from_start;
    rclcpp::Duration segment_duration = end_time - start_time;
    trajectory_msgs::msg::JointTrajectoryPoint setpoint;
    setpoint.positions.resize(NUM_JOINTS);

    // Interpolate position
    double start_point_weight = (time_from_start - start_time).seconds() / segment_duration.seconds();
    for (unsigned int joint_idx = 0; joint_idx < NUM_JOINTS; joint_idx++)
    {
      setpoint.positions[joint_idx] =
        start_point_weight*start_point[joint_idx] + (1.0-start_point_weight)*end_point[joint_idx];
    }
    send_setpoint(setpoint);
  }
}

void RobotHwInterface::send_setpoint(trajectory_msgs::msg::JointTrajectoryPoint setpoint)
{
  JointData joint_data;
  std::copy(setpoint.positions.begin(), setpoint.positions.end(), joint_data.begin());
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

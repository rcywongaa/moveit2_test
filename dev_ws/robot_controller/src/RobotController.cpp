#include "RobotController.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("RobotController");

// Taken from https://stackoverflow.com/questions/3991478/building-a-32-bit-float-out-of-its-4-composite-bytes
float bytesToFloat(unsigned char b0, unsigned char b1, unsigned char b2, unsigned char b3)
{
    float output;

    *((unsigned char*)(&output) + 3) = b0;
    *((unsigned char*)(&output) + 2) = b1;
    *((unsigned char*)(&output) + 1) = b2;
    *((unsigned char*)(&output) + 0) = b3;

    return output;
}

RobotController::JointData RobotController::parse(std::vector<unsigned char>& data)
{
  RobotController::JointData ret;
  for (unsigned int joint_idx = 0; joint_idx < 3; joint_idx++)
  {
    float joint_position = bytesToFloat(
        data[4*joint_idx],
        data[4*joint_idx + 1],
        data[4*joint_idx + 2],
        data[4*joint_idx + 3]);

    if (joint_position > M_PI)
    {
      joint_position -= 2*M_PI;
    }
    ret[joint_idx] = joint_position;
  }
  return ret;
}

std::vector<unsigned char> RobotController::encode(RobotController::JointData input)
{
  std::vector<unsigned char> ret;
  ret.resize(DATA_SIZE);
  for (unsigned int joint_idx = 0; joint_idx < input.size(); joint_idx++)
  {
    float joint_position = input[joint_idx];
    RCLCPP_INFO(LOGGER, "Encoding joint value: %f", joint_position);
    // convert from range [-pi, pi] to [0, 2*pi]
    if (joint_position < 0)
    {
      joint_position += 2*M_PI;
    }
    // Taken from https://stackoverflow.com/questions/14018894/how-to-convert-float-to-byte-array-of-length-4-array-of-char
    unsigned char const * p = reinterpret_cast<unsigned char const *>(&joint_position);
    for (std::size_t i = 0; i < sizeof(float); i++)
    {
      ret[4*joint_idx + i] = p[i];
      RCLCPP_INFO(LOGGER, "Encoded byte[%d]: 0x%x", i, p[i]);
    }
  }
  return ret;
}

RobotController::RobotController(std::unique_ptr<Connection> connection)
  : connection_(std::move(connection))
{
  // TODO: Get joint names from parameter
  joint_angles_.name = {"q1", "q2", "q3"};
  joint_angles_.position = {0, 0, 0};

  //FIXME: Error checking
  connection_->open();
}

RobotController::~RobotController()
{
  //FIXME: Error checking
  connection_->close();
}

void RobotController::set_trajectory(trajectory_msgs::msg::JointTrajectory trajectory)
{
  std::scoped_lock<std::mutex> lock(trajectory_mtx);
  trajectory_ = trajectory;
}

sensor_msgs::msg::JointState RobotController::get_joint_state()
{
  std::scoped_lock<std::mutex> lock(joint_angles_mtx);
  return joint_angles_;
}

void RobotController::spin_once()
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
    double end_point_weight = (time_from_start - start_time).seconds() / segment_duration.seconds();
    for (unsigned int joint_idx = 0; joint_idx < NUM_JOINTS; joint_idx++)
    {
      setpoint.positions[joint_idx] =
        (1.0-end_point_weight)*start_point[joint_idx] + end_point_weight*end_point[joint_idx];
      RCLCPP_INFO(LOGGER, "Setpoint[%d]: %f", joint_idx, setpoint.positions[joint_idx]);
    }
    send_setpoint(setpoint);
  }
}

void RobotController::send_setpoint(trajectory_msgs::msg::JointTrajectoryPoint setpoint)
{
  JointData joint_data;
  std::copy(setpoint.positions.begin(), setpoint.positions.end(), joint_data.begin());
  std::vector<unsigned char> data = encode(joint_data);
  //FIXME: Error checking
  connection_->send(data);
}

void RobotController::receive_joint_data()
{
  std::vector<unsigned char> data;
  //FIXME: Error checking
  connection_->receive(data);
  JointData joint_data = parse(data);
  std::scoped_lock<std::mutex> lock(joint_angles_mtx);
  std::copy(joint_data.begin(), joint_data.end(), joint_angles_.position.begin());
}

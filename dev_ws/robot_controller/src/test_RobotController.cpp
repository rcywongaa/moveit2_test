#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "gmock/gmock.h"

#include "Connection.hpp"
#include "RobotController.hpp"

using namespace ::testing;
using ::testing::StrictMock;
using ::testing::NiceMock;
using ::testing::InSequence;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_RobotController");

class MockConnection : public Connection
{
    public:
        MOCK_METHOD0(open, int());
        MOCK_METHOD0(close, int());
        MOCK_METHOD1(send, int(std::vector<unsigned char>&));
        MOCK_METHOD1(receive, int(std::vector<unsigned char>&));
};

class RobotControllerTest : public testing::Test
{
  protected:
    RobotControllerTest()
    {
      std::unique_ptr<StrictMock<MockConnection>> connection = std::make_unique<StrictMock<MockConnection>>();
      connection_ = connection.get();
      EXPECT_CALL(*connection_, open()).WillOnce(Return(0));
      EXPECT_CALL(*connection_, close()).WillOnce(Return(0));
      interface_ = std::make_unique<RobotController>(std::move(connection));
    }

    StrictMock<MockConnection>* connection_;
    std::unique_ptr<RobotController> interface_;
};

trajectory_msgs::msg::JointTrajectoryPoint create_joint_point(
    double q1, double q2, double q3, double seconds_from_start)
{
  trajectory_msgs::msg::JointTrajectoryPoint joint_point;
  joint_point.positions = {q1, q2, q3};
  std::chrono::milliseconds ms_from_start(int(round(seconds_from_start*1000)));
  joint_point.time_from_start = rclcpp::Duration(ms_from_start);
  return joint_point;
}

trajectory_msgs::msg::JointTrajectory create_default_trajectory()
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"q1", "q2", "q3"};
  trajectory.points.push_back(
      create_joint_point(0.0, 0.0, 0.0, 0.0));
  trajectory.points.push_back(
      create_joint_point(1.0, -2.0, 0.0, 1.0));
  return trajectory;
}

trajectory_msgs::msg::JointTrajectory create_hold_trajectory()
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"q1", "q2", "q3"};
  trajectory.points.push_back(
      create_joint_point(1.4, -2.6, 0.0, 0.0));
  trajectory.points.push_back(
      create_joint_point(1.4, -2.6, 0.0, 1.0));
  return trajectory;
}

void dummy_receive(std::vector<unsigned char>& output)
{
  output = {
    0, 0, 0, 0, // = 0.0
    0x9a, 0x99, 0x19, 0x3f, // = 0.6
    0xCD, 0xCC, 0xAC, 0x40 // = 5.4
  };
}

// FIXME: This test is flaky since any slight timing delay will change the float
// which will lead to completely different bytes...
TEST_F(RobotControllerTest, DISABLED_set_trajectory_and_spin)
{
  interface_->set_trajectory(create_default_trajectory());

  // Ignore receive calls for now
  EXPECT_CALL(*connection_, receive(_)).WillRepeatedly(DoAll(
        Invoke(dummy_receive),
        Return(0)));

  {
    InSequence seq;
    // Expectation for first spin_once()
    std::vector<unsigned char> expected_input1 = {
      //0, 0, 0, 0,
      //0, 0, 0, 0,
      //0, 0, 0, 0

      // 1.0*0.2 = 0.2 radians = 0x3e4ccccd
      0x3E, 0x4C, 0xCC, 0xCD,
      // -2.0*0.2 = -0.4 radians = 0xbecccccd
      0xBE, 0xCC, 0xCC, 0xCD,
      // 0 radians = 0 encoder ticks = 0x0
      0x0, 0x0, 0x0, 0x0
    };
    EXPECT_CALL(*connection_, send(expected_input1)).WillOnce(Return(0));
    // Expectation for second spin_once()
    // Converted with https://www.h-schmidt.net/FloatConverter/IEEE754.html
    std::vector<unsigned char> expected_input2 = {
      // 1.0*0.2 = 0.2 radians = 0x3e4ccccd
      0x3E, 0x4C, 0xCC, 0xCD,
      // -2.0*0.2 = -0.4 radians = 0xbecccccd
      0xBE, 0xCC, 0xCC, 0xCD,
      // 0 radians = 0 encoder ticks = 0x0
      0x0, 0x0, 0x0, 0x0
    };
    EXPECT_CALL(*connection_, send(expected_input2)).WillOnce(Return(0));
  }
  interface_->spin_once();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  interface_->spin_once();
}

TEST_F(RobotControllerTest, set_trajectory_and_spin_hold)
{
  interface_->set_trajectory(create_hold_trajectory());

  // Ignore receive calls for now
  EXPECT_CALL(*connection_, receive(_)).WillRepeatedly(DoAll(
        Invoke(dummy_receive),
        Return(0)));

  {
    InSequence seq;
    // Expectation for first spin_once()
    std::vector<unsigned char> expected_input = {
      // 1.4 = 0x3FB33333
      0x33, 0x33, 0xB3, 0x3F,
      // -2.6 = 3.68318530717958647693 = 0x406BB94F
      0x4F, 0xB9, 0x6B, 0x40,
      // 0 radians = 0 encoder ticks = 0x0
      0x0, 0x0, 0x0, 0x0
    };
    EXPECT_CALL(*connection_, send(expected_input)).WillOnce(Return(0));
    EXPECT_CALL(*connection_, send(expected_input)).WillOnce(Return(0));
  }
  interface_->spin_once();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  interface_->spin_once();
}

// https://stackoverflow.com/questions/28768359/comparison-of-floating-point-arrays-using-google-test-and-google-mock
MATCHER_P(FloatNearPointwise, tol, "is near") {
    return (std::get<0>(arg) > std::get<1>(arg)-tol && std::get<0>(arg) < std::get<1>(arg)+tol);
}

TEST_F(RobotControllerTest, get_joint_state)
{
  EXPECT_CALL(*connection_, receive(_)).WillOnce(DoAll(
        Invoke(dummy_receive),
        Return(0)));

  // Ignore send calls
  EXPECT_CALL(*connection_, send(_)).WillRepeatedly(Return(0));

  interface_->spin_once();

  sensor_msgs::msg::JointState expected_joint_state;
  expected_joint_state.name = {"q1", "q2", "q3"};
  expected_joint_state.position = {0.0, 0.6, -0.8832};
  sensor_msgs::msg::JointState actual_joint_state = interface_->get_joint_state();
  ASSERT_THAT(actual_joint_state.name, ContainerEq(expected_joint_state.name));
  ASSERT_THAT(actual_joint_state.position, Pointwise(FloatNearPointwise(0.001), expected_joint_state.position));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


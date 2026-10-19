#include "motor_driver.h"
#include <gtest/gtest.h>
#include <limits>

using autonomous_mobile_robot::DifferentialDriveMixer;
using autonomous_mobile_robot::Direction;
using autonomous_mobile_robot::DriveLimits;

TEST(DifferentialDriveMixer, StopIsCoastWithZeroDuty) {
  const auto command = DifferentialDriveMixer::compute(0.0, 0.0, DriveLimits{});
  EXPECT_EQ(Direction::kStop, command.left.direction);
  EXPECT_EQ(Direction::kStop, command.right.direction);
  EXPECT_EQ(0U, command.left.duty_percent);
  EXPECT_EQ(0U, command.right.duty_percent);
}

TEST(DifferentialDriveMixer, PositiveForwardDrivesBothWheels) {
  const auto command = DifferentialDriveMixer::compute(0.1, 0.0, DriveLimits{});
  EXPECT_EQ(Direction::kForward, command.left.direction);
  EXPECT_EQ(Direction::kForward, command.right.direction);
  EXPECT_EQ(24U, command.left.duty_percent);
  EXPECT_EQ(24U, command.right.duty_percent);
}

TEST(DifferentialDriveMixer, ReverseUsesPositiveDuty) {
  const auto command = DifferentialDriveMixer::compute(-0.1, 0.0, DriveLimits{});
  EXPECT_EQ(Direction::kReverse, command.left.direction);
  EXPECT_EQ(Direction::kReverse, command.right.direction);
  EXPECT_EQ(24U, command.left.duty_percent);
  EXPECT_EQ(24U, command.right.duty_percent);
}

TEST(DifferentialDriveMixer, LeftTurnSpinsWheelsInOppositeDirections) {
  const auto command = DifferentialDriveMixer::compute(0.0, 0.5, DriveLimits{});
  EXPECT_EQ(Direction::kReverse, command.left.direction);
  EXPECT_EQ(Direction::kForward, command.right.direction);
  EXPECT_EQ(25U, command.left.duty_percent);
  EXPECT_EQ(25U, command.right.duty_percent);
}

TEST(DifferentialDriveMixer, SaturationPreservesWheelRatio) {
  const auto command = DifferentialDriveMixer::compute(0.5, 1.0, DriveLimits{});
  EXPECT_EQ(Direction::kForward, command.left.direction);
  EXPECT_EQ(Direction::kForward, command.right.direction);
  EXPECT_EQ(41U, command.left.duty_percent); // (120-50)/170 * 100
  EXPECT_EQ(100U, command.right.duty_percent);
}

TEST(DifferentialDriveMixer, InvalidVelocityIsRejected) {
  EXPECT_THROW(DifferentialDriveMixer::compute(
      std::numeric_limits<double>::quiet_NaN(), 0.0, DriveLimits{}),
      std::invalid_argument);
}

TEST(DifferentialDriveMixer, InvalidGainIsRejected) {
  DriveLimits limits;
  limits.angular_pwm_per_radps = 0.0;
  EXPECT_THROW(DifferentialDriveMixer::compute(0.0, 0.0, limits),
               std::invalid_argument);
}

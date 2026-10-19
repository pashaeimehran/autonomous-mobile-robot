#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autonomous_mobile_robot {

enum class Direction { kStop, kForward, kReverse };

struct WheelCommand {
  Direction direction = Direction::kStop;
  unsigned duty_percent = 0;
};

struct DriveCommand {
  WheelCommand left;
  WheelCommand right;
};

// Scale command velocities to open-loop PWM percentages.
struct DriveLimits {
  double max_linear_mps = 0.5;
  double max_angular_radps = 2.75;
  double linear_pwm_per_mps = 240.0;
  double angular_pwm_per_radps = 50.0;
};

// Pure command mixing. No ROS, GPIO, or Raspberry Pi dependencies.
class DifferentialDriveMixer {
 public:
  static DriveCommand compute(double linear_mps, double angular_radps,
                              const DriveLimits& limits) {
    const auto valid_positive = [](double value) {
      return std::isfinite(value) && value > 0.0;
    };
    if (!valid_positive(limits.max_linear_mps) ||
        !valid_positive(limits.max_angular_radps) ||
        !valid_positive(limits.linear_pwm_per_mps) ||
        !valid_positive(limits.angular_pwm_per_radps)) {
      throw std::invalid_argument("Drive limits and gains must be finite and positive");
    }
    if (!std::isfinite(linear_mps) || !std::isfinite(angular_radps)) {
      throw std::invalid_argument("Velocity command must be finite");
    }

    const double linear = clamp(linear_mps, -limits.max_linear_mps,
                                limits.max_linear_mps);
    const double angular = clamp(angular_radps, -limits.max_angular_radps,
                                 limits.max_angular_radps);

    // ROS convention: positive angular.z turns counterclockwise (left).
    // For a left turn, the right wheel is commanded faster than the left.
    const double translation_pwm = linear * limits.linear_pwm_per_mps;
    const double rotation_pwm = angular * limits.angular_pwm_per_radps;
    const double left_signed = translation_pwm - rotation_pwm;
    const double right_signed = translation_pwm + rotation_pwm;
    if (!std::isfinite(left_signed) || !std::isfinite(right_signed)) {
      throw std::invalid_argument("PWM calculation overflowed");
    }

    // Scale both wheels together at saturation to preserve the turn ratio.
    const double max_magnitude =
        std::max(std::abs(left_signed), std::abs(right_signed));
    const double divisor = std::max(1.0, max_magnitude / 100.0);
    return {toWheelCommand(left_signed / divisor),
            toWheelCommand(right_signed / divisor)};
  }

 private:
  static double clamp(double value, double low, double high) {
    return std::max(low, std::min(value, high));
  }

  static WheelCommand toWheelCommand(double signed_pwm) {
    const auto duty = static_cast<unsigned>(
        std::lround(clamp(std::abs(signed_pwm), 0.0, 100.0)));
    if (duty == 0U) {
      return {};
    }
    return {signed_pwm > 0.0 ? Direction::kForward : Direction::kReverse, duty};
  }
};

// Wiring is specified with Raspberry Pi BCM GPIO numbers.
struct MotorPins {
  unsigned right_in1_bcm = 17;
  unsigned right_in2_bcm = 27;
  unsigned right_enable_bcm = 14;
  unsigned left_in1_bcm = 23;
  unsigned left_in2_bcm = 24;
  unsigned left_enable_bcm = 18;
};

class MotorDriver {
 public:
  MotorDriver(const MotorPins& pins, const DriveLimits& limits,
              bool invert_right = false, bool invert_left = false);
  ~MotorDriver() noexcept;

  MotorDriver(const MotorDriver&) = delete;
  MotorDriver& operator=(const MotorDriver&) = delete;

  void drive(double linear_mps, double angular_radps);
  void stop();

 private:
  static void check(int result, const char* operation);
  void applyWheel(const WheelCommand& command, unsigned in1, unsigned in2,
                  unsigned enable, bool inverted);
  void stopNoThrow() noexcept;

  MotorPins pins_;
  DriveLimits limits_;
  bool invert_right_;
  bool invert_left_;
  bool initialized_ = false;
};

}  // namespace autonomous_mobile_robot

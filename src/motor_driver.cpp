#include "motor_driver.h"

#include <pigpio.h>

#include <array>
#include <set>
#include <stdexcept>
#include <string>

namespace autonomous_mobile_robot {

void MotorDriver::check(int result, const char* operation) {
  if (result < 0) {
    throw std::runtime_error(std::string(operation) + " failed (pigpio error " +
                             std::to_string(result) + ")");
  }
}

MotorDriver::MotorDriver(const MotorPins& pins, const DriveLimits& limits,
                         bool invert_right, bool invert_left)
    : pins_(pins),
      limits_(limits),
      invert_right_(invert_right),
      invert_left_(invert_left) {
  // Validate parameters before touching any hardware.
  (void)DifferentialDriveMixer::compute(0.0, 0.0, limits_);
  const std::array<unsigned, 6> gpio = {
      pins_.right_in1_bcm, pins_.right_in2_bcm, pins_.right_enable_bcm,
      pins_.left_in1_bcm, pins_.left_in2_bcm, pins_.left_enable_bcm};
  if (std::any_of(gpio.begin(), gpio.end(),
                  [](unsigned pin) { return pin > 31U; }) ||
      std::set<unsigned>(gpio.begin(), gpio.end()).size() != gpio.size()) {
    throw std::invalid_argument("Use six distinct BCM GPIO pins between 0 and 31");
  }

  check(gpioInitialise(), "gpioInitialise");
  initialized_ = true;
  try {
    for (const auto pin : gpio) {
      check(gpioSetMode(pin, PI_OUTPUT), "gpioSetMode");
    }

    // Start stopped. Remove the L298N ENA/ENB jumpers to use external PWM.
    check(gpioPWM(pins_.right_enable_bcm, 0), "right PWM off");
    check(gpioPWM(pins_.left_enable_bcm, 0), "left PWM off");
    check(gpioWrite(pins_.right_in1_bcm, PI_LOW), "right IN1 low");
    check(gpioWrite(pins_.right_in2_bcm, PI_LOW), "right IN2 low");
    check(gpioWrite(pins_.left_in1_bcm, PI_LOW), "left IN1 low");
    check(gpioWrite(pins_.left_in2_bcm, PI_LOW), "left IN2 low");

    for (const auto enable : {pins_.right_enable_bcm, pins_.left_enable_bcm}) {
      check(gpioSetPWMrange(enable, 100), "gpioSetPWMrange");
      check(gpioSetPWMfrequency(enable, 1000), "gpioSetPWMfrequency");
    }
  } catch (...) {
    stopNoThrow();
    gpioTerminate();
    initialized_ = false;
    throw;
  }
}

MotorDriver::~MotorDriver() noexcept {
  if (initialized_) {
    stopNoThrow();
    gpioTerminate();
  }
}

void MotorDriver::stopNoThrow() noexcept {
  // Disable PWM and set H-bridge inputs low during shutdown.
  (void)gpioPWM(pins_.right_enable_bcm, 0);
  (void)gpioPWM(pins_.left_enable_bcm, 0);
  (void)gpioWrite(pins_.right_in1_bcm, PI_LOW);
  (void)gpioWrite(pins_.right_in2_bcm, PI_LOW);
  (void)gpioWrite(pins_.left_in1_bcm, PI_LOW);
  (void)gpioWrite(pins_.left_in2_bcm, PI_LOW);
}

void MotorDriver::stop() {
  check(gpioPWM(pins_.right_enable_bcm, 0), "stop right PWM");
  check(gpioPWM(pins_.left_enable_bcm, 0), "stop left PWM");
  check(gpioWrite(pins_.right_in1_bcm, PI_LOW), "stop right IN1");
  check(gpioWrite(pins_.right_in2_bcm, PI_LOW), "stop right IN2");
  check(gpioWrite(pins_.left_in1_bcm, PI_LOW), "stop left IN1");
  check(gpioWrite(pins_.left_in2_bcm, PI_LOW), "stop left IN2");
}

void MotorDriver::applyWheel(const WheelCommand& command, unsigned in1,
                             unsigned in2, unsigned enable, bool inverted) {
  // Disable PWM before modifying H-bridge direction to avoid abrupt reversals.
  check(gpioPWM(enable, 0), "disable wheel PWM");
  check(gpioWrite(in1, PI_LOW), "clear IN1");
  check(gpioWrite(in2, PI_LOW), "clear IN2");
  if (command.direction == Direction::kStop || command.duty_percent == 0U) {
    return;
  }

  const bool forward = (command.direction == Direction::kForward) != inverted;
  check(gpioWrite(in1, forward ? PI_HIGH : PI_LOW), "set IN1");
  check(gpioWrite(in2, forward ? PI_LOW : PI_HIGH), "set IN2");
  check(gpioPWM(enable, command.duty_percent), "set wheel PWM");
}

void MotorDriver::drive(double linear_mps, double angular_radps) {
  const auto command =
      DifferentialDriveMixer::compute(linear_mps, angular_radps, limits_);
  try {
    applyWheel(command.right, pins_.right_in1_bcm, pins_.right_in2_bcm,
               pins_.right_enable_bcm, invert_right_);
    applyWheel(command.left, pins_.left_in1_bcm, pins_.left_in2_bcm,
               pins_.left_enable_bcm, invert_left_);
  } catch (...) {
    stopNoThrow();
    throw;
  }
}

}  // namespace autonomous_mobile_robot

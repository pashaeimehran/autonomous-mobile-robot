#include "motor_driver.h"

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>

namespace autonomous_mobile_robot {

class MotorDriverNode {
 public:
  MotorDriverNode() : private_nh_("~") {
    DriveLimits limits;
    private_nh_.param("max_linear_mps", limits.max_linear_mps,
                      limits.max_linear_mps);
    private_nh_.param("max_angular_radps", limits.max_angular_radps,
                      limits.max_angular_radps);
    private_nh_.param("linear_pwm_per_mps", limits.linear_pwm_per_mps,
                      limits.linear_pwm_per_mps);
    private_nh_.param("angular_pwm_per_radps", limits.angular_pwm_per_radps,
                      limits.angular_pwm_per_radps);

    MotorPins pins;
    loadPin("right_in1_bcm", pins.right_in1_bcm);
    loadPin("right_in2_bcm", pins.right_in2_bcm);
    loadPin("right_enable_bcm", pins.right_enable_bcm);
    loadPin("left_in1_bcm", pins.left_in1_bcm);
    loadPin("left_in2_bcm", pins.left_in2_bcm);
    loadPin("left_enable_bcm", pins.left_enable_bcm);

    bool invert_right = false;
    bool invert_left = false;
    private_nh_.param("invert_right", invert_right, invert_right);
    private_nh_.param("invert_left", invert_left, invert_left);
    private_nh_.param("cmd_timeout_sec", timeout_sec_, 0.5);
    if (!std::isfinite(timeout_sec_) || timeout_sec_ <= 0.0) {
      throw std::invalid_argument("cmd_timeout_sec must be finite and positive");
    }

    driver_.reset(new MotorDriver(pins, limits, invert_right, invert_left));
    cmd_sub_ = nh_.subscribe("cmd_vel", 1, &MotorDriverNode::onCommand, this);
    watchdog_ = nh_.createWallTimer(ros::WallDuration(0.05),
                                    &MotorDriverNode::onWatchdog, this);
    ROS_INFO("Motor driver ready: cmd_vel; watchdog %.3f s", timeout_sec_);
  }

 private:
  void loadPin(const char* name, unsigned& destination) {
    int value = static_cast<int>(destination);
    private_nh_.param(name, value, value);
    if (value < 0 || value > 31) {
      throw std::invalid_argument("BCM pin parameter outside 0..31");
    }
    destination = static_cast<unsigned>(value);
  }

  void fatalStop(const std::exception& error) {
    ROS_FATAL_STREAM("Motor driver failure: " << error.what());
    has_command_ = false;
    try {
      driver_->stop();
    } catch (const std::exception& stop_error) {
      ROS_ERROR_STREAM("Could not stop both motors: " << stop_error.what());
    }
    ros::shutdown();
  }

  void onCommand(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
      ROS_WARN_THROTTLE(1.0, "Rejected non-finite cmd_vel; stopping");
      has_command_ = false;
      try {
        driver_->stop();
      } catch (const std::exception& error) {
        fatalStop(error);
      }
      return;
    }
    try {
      driver_->drive(msg->linear.x, msg->angular.z);
      last_command_ = std::chrono::steady_clock::now();
      has_command_ = true;
    } catch (const std::exception& error) {
      fatalStop(error);
    }
  }

  void onWatchdog(const ros::WallTimerEvent&) {
    if (!has_command_) {
      return;
    }
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      last_command_)
            .count();
    if (elapsed <= timeout_sec_) {
      return;
    }
    has_command_ = false;
    try {
      driver_->stop();
      ROS_WARN("cmd_vel timed out after %.3f s; motors stopped", elapsed);
    } catch (const std::exception& error) {
      fatalStop(error);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cmd_sub_;
  ros::WallTimer watchdog_;
  std::unique_ptr<MotorDriver> driver_;
  std::chrono::steady_clock::time_point last_command_{};
  double timeout_sec_ = 0.5;
  bool has_command_ = false;
};

}  // namespace autonomous_mobile_robot

int main(int argc, char** argv) {
  ros::init(argc, argv, "motor_driver_node");
  try {
    autonomous_mobile_robot::MotorDriverNode node;
    ros::spin();
    return 0;
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Unable to initialize motor driver: " << error.what());
    return 1;
  }
}

#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# L298N motor driver pin configurations for Motor A
IN1_M1 = 11
IN2_M1 = 13
ENA_M1 = 8

# L298N motor driver pin configurations for Motor B
IN1_M2 = 16
IN2_M2 = 18
ENA_M2 = 12

# Setup GPIO
GPIO.setmode(GPIO.BOARD)  # Use the BOARD numbering system for GPIO pins
GPIO.setup([IN1_M1, IN2_M1, ENA_M1, IN1_M2, IN2_M2, ENA_M2], GPIO.OUT)  # Set motor driver pins as output

# Initialize PWM for both motors with 1000 Hz frequency
pwm_M1 = GPIO.PWM(ENA_M1, 1000)
pwm_M2 = GPIO.PWM(ENA_M2, 1000)

# Start PWM with 0% duty cycle (motors are initially off)
pwm_M1.start(0)
pwm_M2.start(0)

# Adjusted duty cycle values for forward and backward motion
duty_forward = 40   # Forward movement speed (percentage)
duty_backward = 25  # Backward movement speed (percentage)

# Constants to scale the speed and angular movement (rate factors)
rate_x = 6  # Scaling factor for forward/backward movement
rate_z = 2  # Scaling factor for turning

# Callback function to process Twist messages from ROS topic "cmd_vel"
def callback(msg):
    linear_x = round(msg.linear.x, 3)  # Get forward/backward velocity (linear_x) from message
    print(linear_x)
    angular_z = round(msg.angular.z, 3)  # Get turning velocity (angular_z) from message

    # Determine the direction of motion (1 for forward, -1 for backward)
    direction = 1 if linear_x >= 0 else -1
    linear_x = abs(linear_x)  # Make linear_x positive for further calculations

    # Calculate the motor speeds based on linear_x (forward/backward) and angular_z (turning)
    r_motor = direction * (duty_forward * linear_x * rate_x + duty_backward * angular_z * rate_z)
    l_motor = direction * (duty_forward * linear_x * rate_x - duty_backward * angular_z * rate_z)

    # Ensure motor PWM values are within the valid range [0, 100]
    r_motor_pwm = min(max(0, r_motor), 100)
    l_motor_pwm = min(max(0, l_motor), 100)

    print(linear_x, angular_z)  # Print input values for debugging
    print(r_motor_pwm, l_motor_pwm)  # Print calculated PWM values for debugging

    # Apply PWM values to the motors (Motor A - right, Motor B - left)
    pwm_M1.ChangeDutyCycle(r_motor_pwm)  # Set the PWM for the right motor
    GPIO.output(IN1_M1, GPIO.HIGH if r_motor > 0 else GPIO.LOW)  # Set the direction for the right motor
    GPIO.output(IN2_M1, GPIO.LOW if r_motor > 0 else GPIO.HIGH)

    pwm_M2.ChangeDutyCycle(l_motor_pwm)  # Set the PWM for the left motor
    GPIO.output(IN1_M2, GPIO.HIGH if l_motor > 0 else GPIO.LOW)  # Set the direction for the left motor
    GPIO.output(IN2_M2, GPIO.LOW if l_motor > 0 else GPIO.HIGH)

if __name__ == '__main__':
    # Initialize the ROS node for controlling the robot
    rospy.init_node('navi_raspi_move', anonymous=True)
    
    # Subscribe to the "cmd_vel" topic to receive Twist messages
    rospy.Subscriber("cmd_vel", Twist, callback)
    
    # Keep the script running and process incoming messages
    rospy.spin()

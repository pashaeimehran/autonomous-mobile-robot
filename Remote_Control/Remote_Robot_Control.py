from flask import Flask, render_template
import RPi.GPIO as GPIO
import time

# Initialize Flask app with template folder
app = Flask(__name__, template_folder='templates')

# Pin configurations for Motor A (L298N motor driver)
IN1_M1 = 11
IN2_M1 = 13
ENA_M1 = 8

# Pin configurations for Motor B (L298N motor driver)
IN1_M2 = 16
IN2_M2 = 18
ENA_M2 = 12

# Set up GPIO mode to BOARD and configure motor pins as outputs
GPIO.setmode(GPIO.BOARD)
GPIO.setup(IN1_M1, GPIO.OUT)
GPIO.setup(IN2_M1, GPIO.OUT)
GPIO.setup(ENA_M1, GPIO.OUT)
GPIO.setup(IN1_M2, GPIO.OUT)
GPIO.setup(IN2_M2, GPIO.OUT)
GPIO.setup(ENA_M2, GPIO.OUT)

# Set PWM frequency to 1000 Hz for both motors
pwm_motor_a = GPIO.PWM(ENA_M1, 1000)
pwm_motor_b = GPIO.PWM(ENA_M2, 1000)

# Start PWM with 0% duty cycle (motors off)
pwm_motor_a.start(0)
pwm_motor_b.start(0)

# Function to set motor speed and direction
def set_motor_speed(pwm, in1, in2, speed):
    # Set IN1 and IN2 for forward or backward motion based on speed
    GPIO.output(in1, GPIO.HIGH if speed > 0 else GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH if speed < 0 else GPIO.LOW)
    # Set the PWM duty cycle to control motor speed
    pwm.ChangeDutyCycle(abs(speed))

# Route for the home page
@app.route('/')
def index():
    return render_template('index.html')

# Route to move the motors forward
@app.route('/move_forward')
def move_forward():
    set_motor_speed(pwm_motor_a, IN1_M1, IN2_M1, 50)  # Set Motor A to move forward at 50% speed
    set_motor_speed(pwm_motor_b, IN1_M2, IN2_M2, 50)  # Set Motor B to move forward at 50% speed
    return 'Moving Forward'

# Route to move the motors backward
@app.route('/move_backward')
def move_backward():
    set_motor_speed(pwm_motor_a, IN1_M1, IN2_M1, -50)  # Set Motor A to move backward at 50% speed
    set_motor_speed(pwm_motor_b, IN1_M2, IN2_M2, -50)  # Set Motor B to move backward at 50% speed
    return 'Moving Backward'

# Route to turn the motors left
@app.route('/turn_left')
def turn_left():
    set_motor_speed(pwm_motor_a, IN1_M1, IN2_M1, -20)  # Set Motor A to turn left at 20% speed
    set_motor_speed(pwm_motor_b, IN1_M2, IN2_M2, 20)   # Set Motor B to turn right at 20% speed
    return 'Turning Left'

# Route to turn the motors right
@app.route('/turn_right')
def turn_right():
    set_motor_speed(pwm_motor_a, IN1_M1, IN2_M1, 20)   # Set Motor A to turn right at 20% speed
    set_motor_speed(pwm_motor_b, IN1_M2, IN2_M2, -20)  # Set Motor B to turn left at 20% speed
    return 'Turning Right'

# Route to stop the motors
@app.route('/stop')
def stop():
    set_motor_speed(pwm_motor_a, IN1_M1, IN2_M1, 0)  # Stop Motor A
    set_motor_speed(pwm_motor_b, IN1_M2, IN2_M2, 0)  # Stop Motor B
    return 'Stopped'

# Run the app in debug mode on all IP addresses
if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')

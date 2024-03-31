import time
import RPi.GPIO as GPIO

# Set up the GPIO for the servo
GPIO.setmode(GPIO.BCM)
servo_pin = 19
GPIO.setup(servo_pin, GPIO.OUT)
servo_pwm = GPIO.PWM(servo_pin, 50)  # 50 Hz (20 ms PWM period)
servo_pwm.start(0)

def set_servo_angle(angle):
    duty_cycle = angle / 18.0 + 2
    servo_pwm.ChangeDutyCycle(duty_cycle)

# Set the servo to 180 degrees
set_servo_angle(180)
time.sleep(1)  # Wait for the servo to move

# Set the servo to 90 degrees
set_servo_angle(90)
time.sleep(1)  # Wait for the servo to move

# Set the servo to 0 degrees
set_servo_angle(0)
time.sleep(1)  # Wait for the servo to move

# Clean up
servo_pwm.stop()
GPIO.cleanup()

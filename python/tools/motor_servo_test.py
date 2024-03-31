import can
import cantools
import time
import RPi.GPIO as GPIO

# Load the DBC file
db = cantools.database.load_file("odrive-cansimple.dbc")

# Set up the CAN bus
bus = can.Bus("can0", bustype="socketcan")
axis_id = 0x1  # Assuming the axis ID is set to 1

# Set up the GPIO for the servo
GPIO.setmode(GPIO.BCM)
servo_pin = 19
GPIO.setup(servo_pin, GPIO.OUT)
servo_pwm = GPIO.PWM(servo_pin, 50)  # 50 Hz (20 ms PWM period)
servo_pwm.start(0)

def send_can_message(message_name, **kwargs):
    msg = db.get_message_by_name(message_name)
    data = msg.encode(kwargs)
    message = can.Message(arbitration_id=msg.frame_id | axis_id << 5, is_extended_id=False, data=data)
    bus.send(message)

def set_servo_angle(angle):
    duty_cycle = angle / 18.0 + 2
    servo_pwm.ChangeDutyCycle(duty_cycle)

# Set the motor to closed-loop control mode
send_can_message('Set_Axis_State', Axis_Requested_State=0x08)
time.sleep(2)  # Wait for the motor to enter closed-loop control

# Set position control mode
send_can_message('Set_Controller_Mode', Control_Mode=0x03, Input_Mode=0x00)

# Move the motor to position 5.0 revolutions
send_can_message('Set_Input_Pos', Input_Pos=5.0, Vel_FF=0.0, Torque_FF=0.0)
time.sleep(3)  # Wait for the motor to reach the position

# Set the servo to 180 degrees
set_servo_angle(180)
time.sleep(1)  # Wait for the servo to move

# Set velocity control mode
send_can_message('Set_Controller_Mode', Control_Mode=0x02, Input_Mode=0x00)

# Move the motor at a velocity of 1.0 revolution per second
send_can_message('Set_Input_Vel', Input_Vel=1.0, Input_Torque_FF=0.0)
time.sleep(3)  # Wait for the motor to reach the velocity

# Set the servo to 90 degrees
set_servo_angle(90)
time.sleep(1)  # Wait for the servo to move

# Set torque control mode
send_can_message('Set_Controller_Mode', Control_Mode=0x01, Input_Mode=0x00)

# Apply a torque of 0.2 Nm
send_can_message('Set_Input_Torque', Input_Torque=0.2)
time.sleep(3)  # Wait for the motor to apply the torque

# Stop the motor
send_can_message('Set_Input_Vel', Input_Vel=0.0, Input_Torque_FF=0.0)
send_can_message('Set_Axis_State', Axis_Requested_State=0x01)  # Set to idle state

# Set the servo to 0 degrees
set_servo_angle(0)
time.sleep(1)  # Wait for the servo to move

# Clean up
servo_pwm.stop()
GPIO.cleanup()

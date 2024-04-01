import can
import cantools
import time

# Load the DBC file
db = cantools.database.load_file("odrive-cansimple.dbc")

# Set up the CAN bus
bus = can.Bus("can0", bustype="socketcan")
axis_id = 0x0  # Assuming the axis ID is set to 0

def send_can_message(message_name, **kwargs):
    msg = db.get_message_by_name(message_name)
    data = msg.encode(kwargs)
    message = can.Message(arbitration_id=msg.frame_id | axis_id << 5, is_extended_id=False, data=data)
    bus.send(message)

def calibrate_motor():
    # Set the motor to full calibration sequence
    send_can_message('Axis0_Set_Axis_State', Axis_Requested_State=0x03)
    time.sleep(10)  # Wait for calibration to complete
    # Set the motor to closed-loop control mode
    send_can_message('Axis0_Set_Axis_State', Axis_Requested_State=0x08)
    time.sleep(2)

def set_motor_velocity(velocity):
    # Set velocity control mode
    send_can_message('Axis0_Set_Controller_Mode', Control_Mode=0x02, Input_Mode=0x00)
    # Move the motor at the specified velocity
    send_can_message('Axis0_Set_Input_Vel', Input_Vel=velocity, Input_Torque_FF=0.0)
    time.sleep(3)  # Wait for the motor to reach the velocity

# Calibrate the motor
calibrate_motor()

# Test shifting the motor back and forth

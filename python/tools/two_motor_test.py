import can
import cantools
import time

# Load the DBC file
db = cantools.database.load_file("odrive-cansimple.dbc")

# Set up the CAN bus
bus = can.Bus("can0", bustype="socketcan")
axis_id_1 = 0x1  # Assuming the first axis ID is set to 1
axis_id_2 = 0x2  # Assuming the second axis ID is set to 2

def send_can_message(axis_id, message_name, **kwargs):
    msg = db.get_message_by_name(message_name)
    data = msg.encode(kwargs)
    message = can.Message(arbitration_id=msg.frame_id | axis_id << 5, is_extended_id=False, data=data)
    bus.send(message)

# Set both motors to closed-loop control mode
send_can_message(axis_id_1, 'Set_Axis_State', Axis_Requested_State=0x08)
send_can_message(axis_id_2, 'Set_Axis_State', Axis_Requested_State=0x08)
time.sleep(2)  # Wait for the motors to enter closed-loop control

# Set position control mode for both motors
send_can_message(axis_id_1, 'Set_Controller_Mode', Control_Mode=0x03, Input_Mode=0x00)
send_can_message(axis_id_2, 'Set_Controller_Mode', Control_Mode=0x03, Input_Mode=0x00)

# Move the motors to position 5.0 revolutions
send_can_message(axis_id_1, 'Set_Input_Pos', Input_Pos=5.0, Vel_FF=0.0, Torque_FF=0.0)
send_can_message(axis_id_2, 'Set_Input_Pos', Input_Pos=5.0, Vel_FF=0.0, Torque_FF=0.0)
time.sleep(3)  # Wait for the motors to reach the position

# Set velocity control mode for both motors
send_can_message(axis_id_1, 'Set_Controller_Mode', Control_Mode=0x02, Input_Mode=0x00)
send_can_message(axis_id_2, 'Set_Controller_Mode', Control_Mode=0x02, Input_Mode=0x00)

# Move the motors at a velocity of 1.0 revolution per second
send_can_message(axis_id_1, 'Set_Input_Vel', Input_Vel=1.0, Input_Torque_FF=0.0)
send_can_message(axis_id_2, 'Set_Input_Vel', Input_Vel=1.0, Input_Torque_FF=0.0)
time.sleep(3)  # Wait for the motors to reach the velocity

# Set torque control mode for both motors
send_can_message(axis_id_1, 'Set_Controller_Mode', Control_Mode=0x01, Input_Mode=0x00)
send_can_message(axis_id_2, 'Set_Controller_Mode', Control_Mode=0x01, Input_Mode=0x00)

# Apply a torque of 0.2 Nm to both motors
send_can_message(axis_id_1, 'Set_Input_Torque', Input_Torque=0.2)
send_can_message(axis_id_2, 'Set_Input_Torque', Input_Torque=0.2)
time.sleep(3)  # Wait for the motors to apply the torque

# Stop both motors
send_can_message(axis_id_1, 'Set_Input_Vel', Input_Vel=0.0, Input_Torque_FF=0.0)
send_can_message(axis_id_2, 'Set_Input_Vel', Input_Vel=0.0, Input_Torque_FF=0.0)
send_can_message(axis_id_1, 'Set_Axis_State', Axis_Requested_State=0x01)  # Set to idle state
send_can_message(axis_id_2, 'Set_Axis_State', Axis_Requested_State=0x01)  # Set to idle state

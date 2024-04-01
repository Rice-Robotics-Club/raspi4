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

# Set the motor to closed-loop control mode
send_can_message('Axis0_Set_Axis_State', Axis_Requested_State=0x08)
time.sleep(2)  # Wait for the motor to enter closed-loop control

# Set position control mode
send_can_message('Axis0_Set_Controller_Mode', Control_Mode=0x03, Input_Mode=0x00)

# Move the motor to position 5.0 revolutions
send_can_message('Axis0_Set_Input_Pos', Input_Pos=5.0, Vel_FF=0.0, Torque_FF=0.0)
time.sleep(3)  # Wait for the motor to reach the position

# Set velocity control mode
send_can_message('Axis0_Set_Controller_Mode', Control_Mode=0x02, Input_Mode=0x00)

# Move the motor at a velocity of 1.0 revolution per second
send_can_message('Axis0_Set_Input_Vel', Input_Vel=1.0, Input_Torque_FF=0.0)
time.sleep(3)  # Wait for the motor to reach the velocity

# Set torque control mode
send_can_message('Axis0_Set_Controller_Mode', Control_Mode=0x01, Input_Mode=0x00)

# Apply a torque of 0.2 Nm
send_can_message('Axis0_Set_Input_Torque', Input_Torque=0.2)
time.sleep(3)  # Wait for the motor to apply the torque

# Stop the motor
send_can_message('Axis0_Set_Input_Vel', Input_Vel=0.0, Input_Torque_FF=0.0)
send_can_message('Axis0_Set_Axis_State', Axis_Requested_State=0x01)  # Set to idle state

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

def set_motor_state(state, control_mode=None, input_mode=None, position=None, velocity=None, torque=None):
    send_can_message('Axis0_Set_Axis_State', Axis_Requested_State=state)
    time.sleep(2)
    if control_mode is not None:
        send_can_message('Axis0_Set_Controller_Mode', Control_Mode=control_mode, Input_Mode=input_mode)
    if position is not None:
        send_can_message('Axis0_Set_Input_Pos', Input_Pos=position, Vel_FF=0.0, Torque_FF=0.0)
    elif velocity is not None:
        send_can_message('Axis0_Set_Input_Vel', Input_Vel=velocity, Input_Torque_FF=0.0)
    elif torque is not None:
        send_can_message('Axis0_Set_Input_Torque', Input_Torque=torque)

# Set the motor to closed-loop control mode and move to position
set_motor_state(0x08, control_mode=0x03, position=5.0)

# Set velocity control mode and move at a velocity
set_motor_state(0x08, control_mode=0x02, velocity=1.0)

# Set torque control mode and apply a torque
set_motor_state(0x08, control_mode=0x01, torque=0.2)

# Stop the motor
set_motor_state(0x01)

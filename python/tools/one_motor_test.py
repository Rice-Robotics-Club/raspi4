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

def set_motor_to_closed_loop_control():
    send_can_message('Axis0_Set_Axis_State', Axis_Requested_State=0x08)
    time.sleep(2)

def set_position(position, vel_ff=0.0, torque_ff=0.0):
    send_can_message('Axis0_Set_Input_Pos', Input_Pos=position, Vel_FF=vel_ff, Torque_FF=torque_ff)

def set_velocity(velocity, input_torque_ff=0.0):
    send_can_message('Axis0_Set_Input_Vel', Input_Vel=velocity, Input_Torque_FF=input_torque_ff)

def set_torque(torque):
    send_can_message('Axis0_Set_Input_Torque', Input_Torque=torque)

# Set motor to closed-loop control
set_motor_to_closed_loop_control()

# Example usage
set_position(1.0)  # Set position to 1 revolution
time.sleep(2)
set_velocity(0.5)  # Set velocity to 0.5 revolutions per second
time.sleep(2)
set_torque(0.1)    # Set torque to 0.1 Nm

# Properly shut down the SocketCAN bus
bus.shutdown()

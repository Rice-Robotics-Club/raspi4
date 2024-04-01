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

def receive_can_message(timeout=1.0):
    message = bus.recv(timeout)
    if message:
        msg = db.decode_message(message.arbitration_id, message.data)
        print(f"Received message: {msg}")
        return msg
    return None

def calibrate_motor():
    # Set the motor to full calibration sequence
    send_can_message('Axis0_Set_Axis_State', Axis_Requested_State=0x03)
    time.sleep(20)  # Wait for calibration to complete
    
    # Check for errors
    send_can_message('Axis0_Get_Axis_Error', Get_Axis_Error=0x00)
    axis_error = receive_can_message()
    send_can_message('Axis0_Get_Motor_Error', Get_Motor_Error=0x00)
    motor_error = receive_can_message()

    if axis_error and motor_error:
        if axis_error['Axis_Error'] != 0 or motor_error['Motor_Error'] != 0:
            print(f"Calibration failed with Axis Error: {axis_error['Axis_Error']} and Motor Error: {motor_error['Motor_Error']}")
            return False

    # Set the motor to closed-loop control mode
    send_can_message('Axis0_Set_Axis_State', Axis_Requested_State=0x08)
    time.sleep(2)
    
    # Check if the motor is in closed-loop control mode
    send_can_message('Axis0_Get_Axis_State', Get_Axis_State=0x00)
    axis_state = receive_can_message()
    if axis_state and axis_state['Axis_State'] == 8:
        print("Calibration successful and motor is in closed-loop control mode")
        return True
    else:
        print("Calibration failed or motor is not in closed-loop control mode")
        return False

# Calibrate the motor
if calibrate_motor():
    print("Motor calibrated successfully")
else:
    print("Motor calibration failed")

# Properly shut down the SocketCAN bus
bus.shutdown()

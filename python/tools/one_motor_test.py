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
    if message is not None:
        msg = db.decode_message(message.arbitration_id, message.data)
        return msg
    return None

def calibrate_motor():
    # Set the motor to full calibration sequence
    send_can_message('Axis0_Set_Axis_State', Axis_Requested_State=0x03)
    print("Calibration started. Please wait...")
    time.sleep(15)  # Wait for calibration to complete

    # Check if calibration was successful
    state_msg = receive_can_message()
    if state_msg is not None and state_msg.get('Axis_State') == 8:  # 8 is CLOSED_LOOP_CONTROL
        print("Calibration successful!")
    else:
        print("Calibration failed or timed out.")

# Calibrate the motor
calibrate_motor()

# Properly shut down the SocketCAN bus
bus.shutdown()

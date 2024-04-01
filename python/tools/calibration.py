def calibrate_motor():
    # Set the motor to full calibration sequence
    send_can_message('Axis0_Set_Axis_State', Axis_Requested_State=0x03)
    time.sleep(20)  # Wait for calibration to complete
    
    # Check for motor errors
    send_can_message('Axis0_Get_Motor_Error')
    motor_error = receive_can_message()
    
    # Check for encoder errors
    send_can_message('Axis0_Get_Encoder_Error')
    encoder_error = receive_can_message()

    if motor_error and encoder_error:
        if motor_error['Motor_Error'] != 0 or encoder_error['Encoder_Error'] != 0:
            print(f"Calibration failed with Motor Error: {motor_error['Motor_Error']} and Encoder Error: {encoder_error['Encoder_Error']}")
            return False

    # Set the motor to closed-loop control mode
    send_can_message('Axis0_Set_Axis_State', Axis_Requested_State=0x08)
    time.sleep(2)
    
    # Check if the motor is in closed-loop control mode
    send_can_message('Axis0_Get_Axis_State')
    axis_state = receive_can_message()
    if axis_state and axis_state['Axis_State'] == 8:
        print("Calibration successful and motor is in closed-loop control mode")
        return True
    else:
        print("Calibration failed or motor is not in closed-loop control mode")
        return False

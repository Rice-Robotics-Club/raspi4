def start_positions_phase(self):
    self.phase = 'Positions'
    self.get_logger().info('Starting positions phase')
    self.positions_phase()

def positions_phase(self):
    self.motor_msg1.control_mode = 3
    self.motor_msg2.control_mode = 3
    self.motor_msg1.input_pos = self.standard_angle - self.zero_angle1
    self.motor_msg2.input_pos = self.standard_angle - self.zero_angle2
    self.motor_control_publisher1.publish(self.motor_msg1)
    self.motor_control_publisher2.publish(self.motor_msg2)
    self.get_logger().info('positions phase: motors set to standard angles')
    threading.Timer(2, self.transition_to_winding).start()

def transition_to_winding(self):
    self.phase = 'Winding'
    self.get_logger().info('Transitioning to winding phase')
    self.winding_phase()

def winding_phase(self):
    self.motor_msg2.control_mode = 1
    self.motor_msg2.input_torque = self.poising_torque
    self.motor_control_publisher2.publish(self.motor_msg2)
    self.get_logger().info('Winding phase: bottom motor winding the spring')
    threading.Timer(3, self.transition_to_pouncing_bracing).start()

def transition_to_pouncing_bracing(self):
    self.phase = 'Pouncing/Bracing'
    self.get_logger().info('switching to pouncing/bracing phase')
    self.pouncing_bracing_phase()

def pouncing_bracing_phase(self):
    self.motor_msg1.control_mode = 1
    self.motor_msg2.control_mode = 1
    self.motor_msg1.input_torque = MAX_TORQUE
    self.motor_msg2.input_torque = MAX_TORQUE
    self.motor_control_publisher1.publish(self.motor_msg1)
    self.motor_control_publisher2.publish(self.motor_msg2)
    self.get_logger().info('pouncing/bracing phase: motors fully extend the spring')
    threading.Timer(2, self.transition_to_landing).start()

def transition_to_landing(self):
    self.phase = 'Landing'
    self.get_logger().info('transition to landing phase')
    self.landing_phase()

def landing_phase(self):
    self.motor_msg1.control_mode = 3
    self.motor_msg2.control_mode = 3
    self.motor_msg1.input_pos = self.standard_angle - self.zero_angle1
    self.motor_msg2.input_pos = self.standard_angle - self.zero_angle2
    self.motor_control_publisher1.publish(self.motor_msg1)
    self.motor_control_publisher2.publish(self.motor_msg2)
    self.get_logger().info('landing phase: Motors set to standard angle')
    self.phase = 'Positions'
    self.get_logger().info('landing completed. Resetting to Positions phase for next cycle')

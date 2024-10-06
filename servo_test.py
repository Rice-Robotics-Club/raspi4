from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)

COUNT = 16

for i in range(COUNT):
  kit.servo[i].set_pulse_width_range(500, 2500)
  
for i in range(COUNT):
  kit.servo[i].angle = 45.0
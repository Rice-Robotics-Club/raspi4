# raspi4

Repository containing ROS2 workspace. Built packages are not stored in this repo, as instead `colcon build` should be run after cloning. 

Before running:

1. Navigate to src folder 
2. Run in terminal: "git clone https://github.com/odriverobotics/ros_odrive.git".
3. Navigate to ws (ie ros2_ws)
4. Run in terminal: "source /opt/ros/humble/setup.bash"
5. Run in terminal: "colcon build --cmake-clean-cache --packages-select odrive_can"


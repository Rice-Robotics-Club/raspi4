import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

	package_description = "servo_test"

	# Robot State Publisher

	servo_test_node = Node(
		package='catbot',
		executable='servo_test',
		name='servo_test'
	)


	# create and return launch description object
	return LaunchDescription(
		[		 
			servo_test_node
		]
	)

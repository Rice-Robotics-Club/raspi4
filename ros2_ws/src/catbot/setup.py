import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'catbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lopatoj',
    maintainer_email='justin@lopato.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_node = catbot.simple_node:main',
            'stop = catbot.stop:main',
            'motor_test = catbot.motor_test:main',
            'listener_node = catbot.listener_node:main',
            'servo_control_node = catbot.servo_control_node:main',
            'servo_test = catbot.servo_test_node:main',
            'jump = catbot.jump:main',
            'motor_servo_test = catbot.motor_servo_test:main',
            'two_motor_and_servo = catbot.two_motor_and_servo:main',
        ],
    },
)

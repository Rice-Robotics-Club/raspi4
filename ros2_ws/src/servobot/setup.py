from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'servobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.yaml')))
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
            'servo_node = servobot.servo_node:main',
            'multi_servo_node = servobot.multi_servo_node:main',
            'ik_node = servobot.ik_node:main',
            'gait_node = servobot.gait_node:main',
            'test_node = servobot.test_node:main',
        ],
    },
)

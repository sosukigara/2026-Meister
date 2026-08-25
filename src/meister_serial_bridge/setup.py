from setuptools import find_packages, setup
from glob import glob

package_name = 'meister_serial_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='so',
    maintainer_email='s25089@tokyo.kosen-ac.jp',
    description='ROS 2 <-> ESP32 UART bridge: /cmd_vel -> steering + motor PWM frames.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = meister_serial_bridge.serial_bridge_node:main',
        ],
    },
)

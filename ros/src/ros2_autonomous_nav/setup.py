from setuptools import setup
from glob import glob

package_name = 'ros2_autonomous_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf',   glob('urdf/*.urdf')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*.rviz') + glob('config/*.yaml')),
        ('share/' + package_name + '/maps',   glob('maps/*.pgm') + glob('maps/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Meister',
    maintainer_email='',
    description='Nav2/SLAM integration for the Meister rover (simulation and real machine).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_follower = scripts.waypoint_follower:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/yolo_ros_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Ume Onigiri',
    maintainer_email='s25089@tokyo.kosen-ac.jp',
    description='Image recognition package for Meister autonomous robot',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'human_tracker_node = perception.human_tracker:main',
        ],
    },
)

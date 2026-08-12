from setuptools import setup
from glob import glob

package_name = 'meister_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='so',
    maintainer_email='s25089@tokyo.kosen-ac.jp',
    description='ONNX (YOLOv8n) による汎用物体検出の基盤パッケージ (ROS2 ament_python)。',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = meister_vision.detection_node:main',
            'download_model = meister_vision.download_model:main',
        ],
    },
)

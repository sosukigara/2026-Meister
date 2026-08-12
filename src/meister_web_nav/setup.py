from setuptools import setup
from glob import glob

package_name = 'meister_web_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/webui', glob('webui/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='so',
    maintainer_email='s25089@tokyo.kosen-ac.jp',
    description='Local web UI for sending Nav2 waypoints with a clickable map.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_nav_server = meister_web_nav.web_nav_server:main',
        ],
    },
)

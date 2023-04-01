"""webots_ros2 package setup file."""

from setuptools import setup
from glob import glob
import os

package_name = 'autoware_webots'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))

data_files.append((os.path.join('share', package_name, 'launch'), glob('launch/*.py')))
data_files.append((os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')))
data_files.append((os.path.join('share', package_name, 'resource'), glob('resource/*.urdf')))

data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='2023.0.2',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Tesla ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_follower = autoware_webots.lane_follower:main',
            'key = autoware_webots.teleop_twist_keyboard:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)

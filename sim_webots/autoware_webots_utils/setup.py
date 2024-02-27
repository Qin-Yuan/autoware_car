from setuptools import setup
from glob import glob
import os

package_name = 'autoware_webots_utils'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append((os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='1677168241@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "AckermannControlCommand_to_cmd_vel = autoware_webots_utils.AckermannControlCommand_to_cmd_vel:main",
            "odom_to_gnss = autoware_webots_utils.odom_to_gnss:main",
            "tf2_base_link_to_map = autoware_webots_utils.tf2_base_link_to_map:main",
            "vehlicle_webots_pub = autoware_webots_utils.vehlicle_webots_pub:main",
            "msg2pub = autoware_webots_utils.msg2pub:main"
        ],
    },
)

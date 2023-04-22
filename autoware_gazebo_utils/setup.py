from setuptools import setup

package_name = 'autoware_gazebo_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='1677168241@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "AckermannControlCommand_to_cmd_vel = autoware_gazebo_utils.AckermannControlCommand_to_cmd_vel:main",
        ],
    },
)

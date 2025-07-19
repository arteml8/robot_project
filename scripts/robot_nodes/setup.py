from setuptools import setup

package_name = 'robot_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.utils'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Artem Lepilov',
    description='ROS 2 nodes for robot control of Self Driving House Robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ble_node = robot_nodes.ble_node:main',
            'velocity_controller_node = robot_nodes.velocity_controller_node:main',
            'planner_node = robot_nodes.planner_node:main',
            'odometry_node = robot_nodes.odometry_node:main'
        ],
    },
)
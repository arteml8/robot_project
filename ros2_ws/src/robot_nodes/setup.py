from setuptools import setup

package_name = 'robot_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artem',
    maintainer_email='alepilov@hotmail.com',
    tests_require=['pytest'],
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

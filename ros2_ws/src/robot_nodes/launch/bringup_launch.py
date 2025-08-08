# bringup_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_nodes', executable='ble_node', name='ble_node', output='screen'),
        Node(package='robot_nodes', executable='odometry_node', name='odometry', output='screen'),
        Node(package='robot_nodes', executable='velocity_controller_node', name='velocity_controller', output='screen'),
        Node(package='robot_nodes', executable='drive_distance_node', name='drive_distance', output='screen'),
        Node(package='robot_nodes', executable='mock_planner_node', name='mock_planner', output='screen'),
    ])
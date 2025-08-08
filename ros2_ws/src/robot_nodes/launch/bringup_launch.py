# bringup_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_nodes', node_executable='ble_node', node_name='ble_node', output='screen'),
        Node(package='robot_nodes', node_executable='odometry_node', node_name='odometry', output='screen'),
        Node(package='robot_nodes', node_executable='velocity_controller_node', node_name='velocity_controller', output='screen'),
        Node(package='robot_nodes', node_executable='drive_distance_node', node_name='drive_distance', output='screen'),
        Node(package='robot_nodes', node_executable='mock_planner_node', node_name='mock_planner', output='screen'),
    ])
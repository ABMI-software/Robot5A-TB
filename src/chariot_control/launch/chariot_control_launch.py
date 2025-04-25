import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chariot_control',
            executable='serial_node',  
            name='serial',
            output='screen',
        ),
        Node(
            package='robot_visual',
            executable='aruco_detector_single',
            name='aruco_detector_single',
            output='screen',  
        ),
        Node(
            package='chariot_control',
            executable='commands_executor_node',
            name='commands_executor',
            output='screen',  
        ),
    ])
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chariot_control',
            executable='commands_executor_node',
            name='commands_executor',
            output='screen',
            parameters=[{'param_name': 'param_value'}],  
        ),
        Node(
            package='chariot_control',
            executable='commands_node',  
            name='commands',
            output='screen',
        ),
    ])
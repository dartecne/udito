#
# ros2 launch turtlesim_mimic_launch.py
#
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='body_manager',
            executable='inroduce_myself_behavior',
            name='introduce_myself'
        ),
        Node(
            package='multimodal_expression',
            executable='multimodal_expression_server',
            name='multimodal_expression_server'
        ),
        Node(
            package='speech_to_text',
            executable='doa_publisher',
            name='doa_publisher'
        ),
    ])
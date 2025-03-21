#
# ros2 launch turtlesim_mimic_launch.py
#
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='body_manager',
            executable='inroduce_myself',
            name='introduce_myself'
        ),
        Node(
            package='head_package',
            executable='head_server',
            name='head_server'
        ),
        Node(
            package='speech_to_text',
            executable='doa_publisher',
            name='doa_publisher'
        ),
        Node(
            package='text_to_speech',
            executable='text_to_speech_server',
            name='text_to_speech_server'
        )
    ])
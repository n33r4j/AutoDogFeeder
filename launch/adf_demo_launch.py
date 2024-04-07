from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adf_prototype',
            #namespace='',
            executable='main_console',
            name='adf'
        ),
        Node(
            package='adf_prototype',
            #namespace='',
            executable='camera',
            name='adf'
        ),
        Node(
            package='adf_prototype',
            #namespace='',
            executable='detector',
            name='adf'
        ),
        Node(
            package='adf_prototype',
            #namespace='',
            executable='feeder',
            name='adf'
        ),
    ])
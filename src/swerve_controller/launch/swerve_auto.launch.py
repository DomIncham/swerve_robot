from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swerve_controller',
            executable='swerve_commander',
            name='swerve_commander',
            output='screen',
            parameters=[
                os.path.join(
                    os.path.dirname(__file__),
                    '../config/robot_geometry.yaml'
                )
            ],
        ),
        #Node(
        #    package='swerve_controller',
        #    executable='swerve_serial_publisher',
        #    name='swerve_serial_publisher',
        #    output='screen',
        #),
        #Node(
        #    package='swerve_controller',
        #    executable='swerve_auto_commander',
        #    name='swerve_auto_commander',
         #   output='screen',
        #),
    ])

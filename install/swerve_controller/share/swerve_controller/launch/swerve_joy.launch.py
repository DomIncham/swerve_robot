from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
                # Joy node จาก ros-humble-joy package
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/event5',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
                'default_trig_mode': True
            }]
        ),
        # Joystick controller ที่ส่ง cmd_vel
        Node(
            package='swerve_controller',
            executable='swerve_joy_controller',
            name='swerve_joy_controller',
            output='screen',
        ),

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
        Node(
            package='swerve_controller',
            executable='swerve_serial_publisher',
            name='swerve_serial_publisher',
            output='screen',
        ),
        
    ])
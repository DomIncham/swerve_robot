import sys
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SwerveCommander(Node):
    def __init__(self):
        super().__init__("swerve_commander")

        self.namespace_prefix = "/swerve_drive"

        # Declare parameters
        self.wheelbase = self.declare_parameter("wheelbase", 0.3).value
        self.wheel_track = self.declare_parameter("wheel_track", 0.3).value
        self.wheel_radius = self.declare_parameter("wheel_radius", 0.05).value

        # Joint ordering as requested
        self.joint_names = [
            "wheel_front_right",
            "wheel_front_left",
            "wheel_rear_right",
            "wheel_rear_left",
            "steering_front_right",
            "steering_front_left",
            "steering_rear_right",
            "steering_rear_left"
        ]

        self.joint_state_pub = self.create_publisher(JointState, f"{self.namespace_prefix}/joint_states", 10)

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            f"{self.namespace_prefix}/cmd_vel",
            self.twist_to_swerve,
            10
        )

    def twist_to_swerve(self, msg: Twist):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z

        if np.allclose([vx, vy, wz], [0.0, 0.0, 0.0]):
            self.stop()
            return

        pos = []
        vel = []

        # Wheel calculations
        wheels = {
            "wheel_front_right": (1, 1),
            "wheel_front_left": (1, -1),
            "wheel_rear_right": (-1, 1),
            "wheel_rear_left": (-1, -1),
        }

        steerings = {
            "steering_front_right": (1, 1),
            "steering_front_left": (1, -1),
            "steering_rear_right": (-1, 1),
            "steering_rear_left": (-1, -1),
        }

        # wheel velocities
        for name in ["wheel_front_right", "wheel_front_left", "wheel_rear_right", "wheel_rear_left"]:
            ky, kx = wheels[name]
            wheel_vel_x = vx + (kx * self.wheelbase * wz / 2)
            wheel_vel_y = vy + (ky * self.wheel_track * wz / 2)
            wheel_angular_velocity = np.sqrt(wheel_vel_x**2 + wheel_vel_y**2) / self.wheel_radius
            pos.append(0.0)
            vel.append(wheel_angular_velocity)

        # steering angles
        for name in ["steering_front_right", "steering_front_left", "steering_rear_right", "steering_rear_left"]:
            ky, kx = steerings[name]
            wheel_vel_x = vx + (kx * self.wheelbase * wz / 2)
            wheel_vel_y = vy + (ky * self.wheel_track * wz / 2)
            steering_angle = np.arctan2(wheel_vel_y, wheel_vel_x)
            pos.append(steering_angle)
            vel.append(0.0)

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = pos
        joint_msg.velocity = vel

        self.joint_state_pub.publish(joint_msg)

    def stop(self):
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = [0.0] * 8
        joint_msg.velocity = [0.0] * 8
        self.joint_state_pub.publish(joint_msg)

def main():
    rclpy.init()
    node = SwerveCommander()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

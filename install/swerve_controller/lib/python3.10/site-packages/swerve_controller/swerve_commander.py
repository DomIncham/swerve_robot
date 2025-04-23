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

        self.wheelbase = self.declare_parameter("wheelbase", 0.3).value
        self.wheel_track = self.declare_parameter("wheel_track", 0.3).value
        self.wheel_radius = self.declare_parameter("wheel_radius", 0.05).value

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

        self.joint_state_pub = self.create_publisher(
            JointState, f"{self.namespace_prefix}/joint_states", 10
        )

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

        wheel_pos = [0.0] * 4
        wheel_vel = []
        steering_pos = []
        steering_vel = [0.0] * 4

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

        # คำนวณ wheel velocity (signed) + steering angle
        for name in ["wheel_front_right", "wheel_front_left", "wheel_rear_right", "wheel_rear_left"]:
            ky, kx = wheels[name]
            wx = vx + (kx * self.wheelbase * wz / 2)
            wy = vy + (ky * self.wheel_track * wz / 2)

            vel_vector = np.array([wx, wy])
            speed = np.linalg.norm(vel_vector)

            raw_angle = np.arctan2(wy, wx)
            flip = False

            if raw_angle > np.pi / 2:
                flip = True
                angle = raw_angle - np.pi
            elif raw_angle < -np.pi / 2:
                flip = True
                angle = raw_angle + np.pi
            else:
                angle = raw_angle

            signed_speed = (-1.0 if flip else 1.0) * speed / self.wheel_radius
            wheel_vel.append(signed_speed)

        # Steering angles (มุมถูกลิมิตแล้วจาก loop ด้านบน)
        for name in ["steering_front_right", "steering_front_left", "steering_rear_right", "steering_rear_left"]:
            ky, kx = steerings[name]
            wx = vx + (kx * self.wheelbase * wz / 2)
            wy = vy + (ky * self.wheel_track * wz / 2)

            raw_angle = np.arctan2(wy, wx)
            if raw_angle > np.pi / 2:
                angle = raw_angle - np.pi
            elif raw_angle < -np.pi / 2:
                angle = raw_angle + np.pi
            else:
                angle = raw_angle

            steering_pos.append(angle)

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = wheel_pos + steering_pos
        joint_msg.velocity = wheel_vel + steering_vel

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

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

        self.last_steering_pos = [0.0] * 4

        self.joint_state_pub = self.create_publisher(
            JointState, f"{self.namespace_prefix}/joint_states", 10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            f"{self.namespace_prefix}/cmd_vel",
            self.twist_to_swerve,
            10
        )

        # Robot physical parameters for inverse kinematics
        self.wheelbase = 0.705  # meters
        self.wheel_track = 0.30  # meters
        self.wheel_radius = 0.12  # meters

    def twist_to_swerve(self, msg: Twist):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z

        if vx == 0 and vy == 0 and wz == 0:
            self.last_steering_pos = [0.0] * 4

        if np.isclose(vx, 0.0, atol=0.05) and np.isclose(vy, 0.0, atol=0.05) and np.isclose(wz, 0.01, atol=0.01):
            self.stop()
            return

        # Choose dispatcher here
        self.manual_command_dispatcher(vx, vy, wz)
        # To use inverse kinematics instead, comment the line above and uncomment below:
        # self.inverse_kinematics_dispatch(vx, vy, wz)

    def manual_command_dispatcher(self, vx, vy, wz):
        wheel_vel = [0.0] * 4
        steering_pos = [0.0] * 4

        # Thresholds
        LINEAR_THRESHOLD = 0.2
        ANGULAR_THRESHOLD = 0.2

        # Constants (now used as maximum values)
        MAX_WHEEL_SPEED = 8.3
        ROTATE_SERVO_OFFSET = 1.29
        DIAGONAL_SERVO_OFFSET = 0.79
        PURE_ROTATE_SERVO_OFFSET = 1.29

        # Calculate the overall speed and rotation factors (0 to 1)
        linear_speed_factor = np.sqrt(vx**2 + vy**2)  # Normalize if needed
        angular_speed_factor = min(abs(wz), 1.0)  # Cap at 1.0

        if np.isclose(vx, 0.0, atol=0.05) and np.isclose(vy, 0.0, atol=0.05) and abs(wz) > ANGULAR_THRESHOLD:
            # Pure rotation
            speed = MAX_WHEEL_SPEED * angular_speed_factor
            if wz > 0:
                wheel_vel = [speed, -speed, -speed, speed]
                steering_pos = [ROTATE_SERVO_OFFSET, -ROTATE_SERVO_OFFSET, -ROTATE_SERVO_OFFSET, ROTATE_SERVO_OFFSET]
            else:
                wheel_vel = [-speed, speed, speed, -speed]
                steering_pos = [ROTATE_SERVO_OFFSET, -ROTATE_SERVO_OFFSET, -ROTATE_SERVO_OFFSET, ROTATE_SERVO_OFFSET]

        elif abs(vx) > abs(vy) and abs(vx) > abs(wz):
            # X-axis movement (forward/backward)
            speed = MAX_WHEEL_SPEED * linear_speed_factor * np.sign(vx)
            wheel_vel = [speed, speed, -speed, -speed]  # Adjust signs if needed for your configuration
            steering_pos = [0.0] * 4

        elif abs(vy) > abs(vx) and abs(vy) > abs(wz):
            # Y-axis movement (strafe left/right)
            speed = MAX_WHEEL_SPEED * linear_speed_factor #* np.sign(vy)
            wheel_vel = [speed, speed, -speed, -speed]  # Adjust signs if needed
            steering_pos = [1.57 if vy > 0 else -1.57] * 4

        elif abs(vx) > LINEAR_THRESHOLD and abs(vy) > LINEAR_THRESHOLD:
            # Diagonal movement
            speed = MAX_WHEEL_SPEED * linear_speed_factor
            if vx > 0 and vy > 0:
                wheel_vel = [speed, speed, -speed, -speed]
                steering_pos = [DIAGONAL_SERVO_OFFSET] * 4
            elif vx > 0 and vy < 0:
                wheel_vel = [speed, speed, -speed, -speed]
                steering_pos = [-DIAGONAL_SERVO_OFFSET] * 4
            elif vx < 0 and vy > 0:
                wheel_vel = [-speed, -speed, speed, speed]
                steering_pos = [-DIAGONAL_SERVO_OFFSET] * 4
            elif vx < 0 and vy < 0:
                wheel_vel = [-speed, -speed, speed, speed]
                steering_pos = [DIAGONAL_SERVO_OFFSET] * 4

        self.last_steering_pos = steering_pos.copy()

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = [0.0] * 4 + steering_pos
        joint_msg.velocity = wheel_vel + [0.0] * 4

        self.joint_state_pub.publish(joint_msg)

    def inverse_kinematics_dispatch(self, vx, vy, wz):
        wheel_vel = []
        steering_pos = []
        steering_vel = [0.0] * 4
        wheel_pos = [0.0] * 4

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

        for name in wheels:
            ky, kx = wheels[name]
            wx = vx + (kx * self.wheelbase * wz / 2)
            wy = vy + (ky * self.wheel_track * wz / 2)
            vel_vector = np.array([wx, wy])
            speed = np.linalg.norm(vel_vector)
            raw_angle = np.arctan2(wy, wx)

            if raw_angle > np.pi / 2:
                angle = raw_angle - np.pi
                speed *= -1
            elif raw_angle < -np.pi / 2:
                angle = raw_angle + np.pi
                speed *= -1
            else:
                angle = raw_angle

            steering_pos.append(angle)
            wheel_vel.append(speed / self.wheel_radius)

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
        joint_msg.position = [0.0] * 4 + self.last_steering_pos
        joint_msg.velocity = [0.0] * 8
        self.joint_state_pub.publish(joint_msg)


def main():
    rclpy.init()
    node = SwerveCommander()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

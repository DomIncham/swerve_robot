#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import busio
import adafruit_bno055
from tf_transformations import quaternion_from_euler
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException

class BNO055IMUPublisher(Node):
    def __init__(self):
        super().__init__('bno055_imu_publisher')

        # Initialize BNO055 IMU
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.sensor.mode = adafruit_bno055.IMUPLUS_MODE

        # Calibration
        self.yaw_offset_deg = None
        self.last_yaw_deg = None
        self.unwrapped_yaw_deg = 0.0

        # ROS Publisher
        self.publisher_ = self.create_publisher(Imu, '/swerve_drive/imu', 10)
        self.timer = self.create_timer(0.05, self.publish_imu_data)  # 20 Hz

        # Covariance (จาก TurtleBot3)
        self.orientation_cov = np.array([0.0025, 0, 0,
                                         0, 0.0025, 0,
                                         0, 0, 0.0025], dtype=np.float64)
        self.angular_vel_cov = np.array([0.0004, 0, 0,
                                         0, 0.0004, 0,
                                         0, 0, 0.0004], dtype=np.float64)
        self.linear_accel_cov = np.array([0.01, 0, 0,
                                          0, 0.01, 0,
                                          0, 0, 0.01], dtype=np.float64)

    def publish_imu_data(self):
        try:
            # Read sensor
            euler = self.sensor.euler
            gyro = self.sensor.gyro
            accel = self.sensor.linear_acceleration

            if euler is None or None in euler:
                return

            yaw_raw = euler[0]
            pitch_deg = euler[1]
            roll_deg = euler[2]

            # Init calibration
            if self.yaw_offset_deg is None:
                self.yaw_offset_deg = yaw_raw
                self.last_yaw_deg = yaw_raw
                self.get_logger().info(f'[IMU] Yaw offset initialized: {self.yaw_offset_deg:.2f}°')
                return

            # Compute delta
            delta_yaw = yaw_raw - self.last_yaw_deg
            if delta_yaw > 180:
                delta_yaw -= 360
            elif delta_yaw < -180:
                delta_yaw += 360

            self.unwrapped_yaw_deg += delta_yaw
            self.last_yaw_deg = yaw_raw

            # Subtract offset
            yaw_corrected_deg = self.unwrapped_yaw_deg - self.yaw_offset_deg

            # Convert to radians and invert yaw direction (ให้ตรงกับ TurtleBot3)
            yaw = math.radians(yaw_corrected_deg)
            pitch = math.radians(pitch_deg)
            roll = math.radians(roll_deg)

            # Quaternion
            q = quaternion_from_euler(roll, pitch, yaw)

            # Message
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"

            msg.orientation.x = float(q[0])
            msg.orientation.y = float(q[1])
            msg.orientation.z = float(q[2])
            msg.orientation.w = float(q[3])
            msg.orientation_covariance = self.orientation_cov.flatten().tolist()

            if gyro and None not in gyro:
                msg.angular_velocity.x = float(gyro[0])
                msg.angular_velocity.y = float(gyro[1])
                msg.angular_velocity.z = float(gyro[2])
                msg.angular_velocity_covariance = self.angular_vel_cov.flatten().tolist()

            if accel and None not in accel:
                msg.linear_acceleration.x = float(accel[0])
                msg.linear_acceleration.y = float(accel[1])
                msg.linear_acceleration.z = float(accel[2])
                msg.linear_acceleration_covariance = self.linear_accel_cov.flatten().tolist()

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f'[IMU] Error: {str(e)}', throttle_duration_sec=5)

def main(args=None):
    rclpy.init(args=args)
    node = BNO055IMUPublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().warn("📴 IMU shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

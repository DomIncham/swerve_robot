import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
import serial
import struct
from decimal import Decimal, ROUND_HALF_UP
import rclpy
from rclpy.executors import ExternalShutdownException

class SwerveSerialPublisher(Node):
    def __init__(self):
        super().__init__('swerve_serial_publisher')

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
            self.get_logger().info("✅ Serial port opened successfully.")
        except serial.SerialException:
            self.get_logger().error("❌ Failed to open Serial port! Check connection.")
            self.ser = None

        self.subscription = self.create_subscription(
            JointState,
            '/swerve_drive/joint_states',
            self.joint_state_callback,
            50
        )

        self.latest_joint_state = None
        self.previous_motor_values = {
            "wheel_front_right": 0.0,
            "wheel_front_left":  0.0,
            "wheel_rear_right":  0.0,
            "wheel_rear_left":   0.0,
        }
        self.deadzone_threshold = 0.4

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg
        self.send_serial_data()

    def calculate_checksum(self, values):
        int_values = [int(v * 100) for v in values]
        return sum(int_values)

    def process_wheel_value(self, name, value):
        if abs(value) < 0.10:
            self.previous_motor_values[name] = 0.0
            return 0.0

        prev = self.previous_motor_values[name]
        if abs(value - prev) < self.deadzone_threshold:
            return prev

        rounded = float(Decimal(value).quantize(Decimal('0.1'), rounding=ROUND_HALF_UP))
        self.previous_motor_values[name] = rounded
        return rounded

    def process_steering_value(self, value):
        if abs(value) < 0.01:
            return 0.0
        clamped = max(min(value, 1.57), -1.57)
        return float(Decimal(clamped).quantize(Decimal('0.01'), rounding=ROUND_HALF_UP))

    def send_serial_data(self):
        if self.ser is None or self.latest_joint_state is None:
            return

        position_map = {
            name: pos
            for name, pos in zip(self.latest_joint_state.name, self.latest_joint_state.position)
        }
        velocity_map = {
            name: vel
            for name, vel in zip(self.latest_joint_state.name, self.latest_joint_state.velocity)
        }

        servo1 = self.process_steering_value(position_map["steering_front_right"])
        servo2 = self.process_steering_value(position_map["steering_front_left"])
        servo3 = self.process_steering_value(position_map["steering_rear_right"])
        servo4 = self.process_steering_value(position_map["steering_rear_left"])

        motor1 = self.process_wheel_value("wheel_front_right", velocity_map["wheel_front_right"])
        motor2 = self.process_wheel_value("wheel_front_left",  velocity_map["wheel_front_left"])
        motor3 = self.process_wheel_value("wheel_rear_right",  velocity_map["wheel_rear_right"])
        motor4 = self.process_wheel_value("wheel_rear_left",   velocity_map["wheel_rear_left"])

        checksum = self.calculate_checksum([
            motor1, motor2, motor3, motor4,
            servo1, servo2, servo3, servo4
        ])

        data_packet = struct.pack(
            '<8f i',
            motor1, motor2, motor3, motor4,
            servo1, servo2, servo3, servo4,
            checksum
        )

        self.ser.write(data_packet)
        self.ser.flush()

        self.get_logger().info(f'datalen : {len(data_packet)}')
        self.get_logger().info(f'Sent Data (Raw): {data_packet}')
        self.get_logger().info(f'Motor: [{motor1}, {motor2}, {motor3}, {motor4}]')
        self.get_logger().info(f'Servo: [{servo1}, {servo2}, {servo3}, {servo4}]')
        self.get_logger().info(f'Checksum: {checksum}')
        self.get_logger().info('-' * 50)

    def send_emergency_stop(self):
        if self.ser is None:
            return
        stop_packet = struct.pack('<8f i',
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0
        )
        try:
            self.ser.write(stop_packet)
            self.ser.flush()
            self.get_logger().warn("🚫 EMERGENCY STOP SIGNAL SENT!")
        except Exception as e:
            self.get_logger().error(f"Failed to send emergency stop: {e}")

    def destroy_node(self):
        self.send_emergency_stop()
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()




def main(args=None):
    rclpy.init(args=args)
    node = SwerveSerialPublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().warn("📴 Swerve Serial Publisher shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

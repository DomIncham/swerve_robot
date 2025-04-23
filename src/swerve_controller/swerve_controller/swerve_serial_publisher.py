import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
import serial
import struct
from decimal import Decimal, ROUND_HALF_UP

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

        self.send_interval = 0.1  # 500ms
        self.timer = self.create_timer(self.send_interval, self.send_serial_data)

        self.latest_joint_state = None

        self.previous_motor_values = {
            "wheel_front_right": 0.0,
            "wheel_rear_left": 0.0,
            "wheel_rear_right": 0.0,
            "wheel_front_left": 0.0,
        }

        self.deadzone_threshold = 0.4

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def calculate_checksum(self, values):
        int_values = [int(v * 100) for v in values]
        return sum(int_values)

    def process_wheel_value(self, name, value):
        if abs(value) < 0.10:
            return 0.0

        prev_value = self.previous_motor_values[name]
        if abs(value - prev_value) < self.deadzone_threshold:
            return prev_value

        processed_value = float(Decimal(value).quantize(Decimal('0.1'), rounding=ROUND_HALF_UP))
        self.previous_motor_values[name] = processed_value
        return processed_value

    def process_steering_value(self, value):
        if abs(value) < 0.01 or abs(value) > 1.59:
            return 0.0
        return float(Decimal(value).quantize(Decimal('0.01'), rounding=ROUND_HALF_UP))

    def send_serial_data(self):
        if self.ser is None or self.latest_joint_state is None:
            return

        joint_map = {name: pos for name, pos in zip(self.latest_joint_state.name, self.latest_joint_state.position)}
        velocity_map = {name: vel for name, vel in zip(self.latest_joint_state.name, self.latest_joint_state.velocity)}
        
        # ❗ Check: ถ้า joint_states ทั้งหมด = 0 ให้ skip ไม่ต้องส่ง
        all_positions_zero = all(abs(p) < 1e-5 for p in joint_map.values())
        all_velocities_zero = all(abs(v) < 1e-5 for v in velocity_map.values())

        if all_positions_zero and all_velocities_zero:
            self.get_logger().info("🛑 All joint_states are 0 — skipping serial send.")
            return

        # Steering angles
        servo1 = self.process_steering_value(joint_map.get("steering_front_right", 0.0))
        servo2 = self.process_steering_value(joint_map.get("steering_front_left", 0.0))
        servo3 = self.process_steering_value(joint_map.get("steering_rear_left", 0.0))
        servo4 = self.process_steering_value(joint_map.get("steering_rear_right", 0.0))

        # Wheel velocities
        motor1 = self.process_wheel_value("wheel_front_right", velocity_map.get("wheel_front_right", 0.0))
        motor2 = self.process_wheel_value("wheel_rear_left", velocity_map.get("wheel_rear_left", 0.0))
        motor3 = self.process_wheel_value("wheel_rear_right", velocity_map.get("wheel_rear_right", 0.0))
        motor4 = self.process_wheel_value("wheel_front_left", velocity_map.get("wheel_front_left", 0.0))

        checksum = self.calculate_checksum([motor1, motor2, motor3, motor4, servo1, servo2, servo3, servo4])

        data_packet = struct.pack('<8f i', motor1, motor2, motor3, motor4, servo1, servo2, servo3, servo4, checksum)
        lendata =  len(data_packet)
        
        self.get_logger().info(f'datalen : {lendata}')
        self.ser.write(data_packet)

        self.get_logger().info(f'Sent Data (Raw): {data_packet}')
        self.get_logger().info(f'Motor: [{motor1}, {motor2}, {motor3}, {motor4}]')
        self.get_logger().info(f'Servo: [{servo1}, {servo2}, {servo3}, {servo4}]')
        self.get_logger().info(f'Checksum: {checksum}')
        self.get_logger().info('-' * 50)

def main(args=None):
    rclpy.init(args=args)
    node = SwerveSerialPublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

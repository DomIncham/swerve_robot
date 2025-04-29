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

        # Subscribe to /swerve_drive/joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/swerve_drive/joint_states',
            self.joint_state_callback,
            50
        )

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
        self.send_serial_data()

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

        self.get_logger().info(f'datalen : {len(data_packet)}')
        self.ser.write(data_packet)
        self.ser.flush()  # เพิ่มบรรทัดนี้เพื่อบังคับส่งข้อมูลทันที

        self.get_logger().info(f'Sent Data (Raw): {data_packet}')
        self.get_logger().info(f'Motor: [{motor1}, {motor2}, {motor3}, {motor4}]')
        self.get_logger().info(f'Servo: [{servo1}, {servo2}, {servo3}, {servo4}]')
        self.get_logger().info(f'Checksum: {checksum}')
        self.get_logger().info('-' * 50)

    def send_emergency_stop(self):
        """ส่งคำสั่งหยุดมอเตอร์ฉุกเฉิน"""
        if self.ser is None:
            return

        # สร้างแพ็คเก็ตหยุดฉุกเฉิน (ทุกค่าเป็น 0)
        stop_packet = struct.pack('<8f i', 
            0.0, 0.0, 0.0, 0.0,  # motors
            0.0, 0.0, 0.0, 0.0,  # servos
            0                      # checksum
        )
        
        try:
            self.ser.write(stop_packet)
            self.ser.flush()
            self.get_logger().warn("🛑 EMERGENCY STOP SIGNAL SENT!")
        except Exception as e:
            self.get_logger().error(f"Failed to send emergency stop: {e}")

    def destroy_node(self):
        """Override การปิดโหนด"""
        self.send_emergency_stop()
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SwerveSerialPublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Received KeyboardInterrupt, shutting down...")
    except Exception as e:
        node.get_logger().error(f"❌ Node error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
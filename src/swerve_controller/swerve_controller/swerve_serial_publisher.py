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

        # พยายามเปิด Serial Port
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
            self.get_logger().info("✅ Serial port opened successfully.")
        except serial.SerialException:
            self.get_logger().error("❌ Failed to open Serial port! Check connection.")
            self.ser = None

        # Subscribe `/joint_states`
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            50  # เพิ่ม queue size ให้สูงขึ้นเพื่อลดดีเลย์
        )

        # Timer สำหรับส่งข้อมูลทุก 100ms (50Hz)
        self.send_interval = 0.5  # 100ms
        self.timer = self.create_timer(self.send_interval, self.send_serial_data)

        # ตัวแปรเก็บค่าล่าสุดของ joint states
        self.latest_joint_state = None
        
        # ตัวแปรเก็บค่ามอเตอร์ก่อนหน้า (ใช้ Deadzone)
        self.previous_motor_values = {"wheel_front_right_joint": 0.0,
                                      "wheel_rear_left_joint": 0.0,
                                      "wheel_rear_right_joint": 0.0,
                                      "wheel_front_left_joint": 0.0}
        self.deadzone_threshold = 0.4  # กำหนด Deadzone Threshold

    def joint_state_callback(self, msg):
        """ เก็บค่าล่าสุดของ `/joint_states` """
        self.latest_joint_state = msg

    def calculate_checksum(self, values):
        """ คำนวณ Checksum โดยรวมค่าของข้อมูลทั้ง 8 ตัว """
        int_values = [int(v * 100) for v in values]  # คูณ 100 แล้วแปลงเป็น int
        return sum(int_values)

    def process_wheel_value(self, name, value):
        """ ปัดเศษค่า Wheel เป็นทศนิยม 1 ตำแหน่ง และใช้ Deadzone """
        if abs(value) < 0.10:
            return 0.0
        
        prev_value = self.previous_motor_values[name]
        
        # ถ้าค่าที่เปลี่ยนแปลงน้อยกว่า threshold ไม่ต้องอัปเดต
        if abs(value - prev_value) < self.deadzone_threshold:
            return prev_value
        
        processed_value = float(Decimal(value).quantize(Decimal('0.1'), rounding=ROUND_HALF_UP))
        self.previous_motor_values[name] = processed_value
        return processed_value

    def process_steering_value(self, value):
        """ ปัดเศษค่า Steering เป็นทศนิยม 2 ตำแหน่ง และเซ็ตเป็น 0 ถ้าอยู่นอกขอบเขต """
        if abs(value) < 0.01 or abs(value) > 1.59:
            return 0.0
        return float(Decimal(value).quantize(Decimal('0.01'), rounding=ROUND_HALF_UP))

    def send_serial_data(self):
        """ อ่านค่าจาก `latest_joint_state` และส่ง Serial Data """
        if self.ser is None or self.latest_joint_state is None:
            return

        joint_map = {name: pos for name, pos in zip(self.latest_joint_state.name, self.latest_joint_state.position)}
        velocity_map = {name: vel for name, vel in zip(self.latest_joint_state.name, self.latest_joint_state.velocity)}

        # Steering Position
        servo1 = self.process_steering_value(joint_map.get("steering_front_right_joint", 0.0))
        servo2 = self.process_steering_value(joint_map.get("steering_front_left_joint", 0.0))
        servo3 = self.process_steering_value(joint_map.get("steering_rear_left_joint", 0.0))
        servo4 = self.process_steering_value(joint_map.get("steering_rear_right_joint", 0.0))

        # Wheel Velocity (ใช้ Deadzone)
        motor1 = self.process_wheel_value("wheel_front_right_joint", velocity_map.get("wheel_front_right_joint", 0.0))
        motor2 = self.process_wheel_value("wheel_rear_left_joint", velocity_map.get("wheel_rear_left_joint", 0.0))
        motor3 = self.process_wheel_value("wheel_rear_right_joint", velocity_map.get("wheel_rear_right_joint", 0.0))
        motor4 = self.process_wheel_value("wheel_front_left_joint", velocity_map.get("wheel_front_left_joint", 0.0))

         # คำนวณ Checksum ก่อนใช้
        checksum = self.calculate_checksum([motor1, motor2, motor3, motor4, servo1, servo2, servo3, servo4])

        # สร้าง Packet Data
        data_packet = struct.pack('8f i', motor1, motor2, motor3, motor4, servo1, servo2, servo3, servo4, checksum)

        # ส่งข้อมูลไปที่ Arduino Mega
        self.ser.write(data_packet)

        # แสดงค่าที่ส่งออก Terminal
        self.get_logger().info(f'Sent Data (Raw): {data_packet}')
        self.get_logger().info(f'Motor: [{motor1}, {motor2}, {motor3}, {motor4}]')
        self.get_logger().info(f'Servo: [{servo1}, {servo2}, {servo3}, {servo4}]')
        self.get_logger().info(f'Checksum: {checksum}')
        self.get_logger().info('-' * 50)  # แยกผลลัพธ์ให้อ่านง่าย


def main(args=None):
    rclpy.init(args=args)
    
    # ใช้ MultiThreadedExecutor เพื่อให้ Node ทำงานเร็วขึ้น
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

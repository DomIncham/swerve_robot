import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SwerveAutoCommander(Node):
    def __init__(self):
        super().__init__('swerve_auto_commander')
        self.pub = self.create_publisher(Twist, '/swerve_drive/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # เรียกทุก 100ms

        # รายการคำสั่งพิเศษพร้อม speed scaling (scale = ความเร็วสัมพัทธ์) และระยะเวลา (duration)    x,y,z,scale_speed,duration
        self.commands = [
            ("STOP", 1.0, 0.0, 0.0, 0.01, 3),  # duration = 3 วินาที
            ("Forward", 1.0, 0.0, 0.0, 1, 2),  # duration = 1 วินาทีม
            #("Forward", 1.0, 0.0, 0.0, 0.8, 2.5),  # duration = 1 วินาที
            ("STOP", 1.0, 0.0, 0.0, 0.01, 3),  # duration = 3 วินาที
            ("Backward", -1.0, 0.0, 0.0, 1, 2),  # duration = 2 วินาที
            #("Backward", -1.0, 0.0, 0.0, 0.8, 2.5),  # duration = 2 วินาที
            #("STOP", -1.0, 0.0, 0.0, 0.01, 1),  # duration = 2 วินาที
            #("Right", 0.0, -1.0, 0.0, 1, 1),
            #("STOP", 0.0, -1.0, 0.0, 0.01, 1),  # duration = 3 วินาที
            #("Left", 0.0, 1.0, 0.0, 1, 1),
            #("STOP", 0.0, 1.0, 0.0, 0.01, 1),  # duration = 3 วินาที
            #("Forward-Left", 0.7, 0.7, 0.0, 1, 1),
            #("STOP", 0.7, 0.7, 0.0, 0.01, 1),  # duration = 3 วินาที
            #("Forward-Right", 0.7, -0.7, 0.0, 1, 1),
            #("STOP", 0.7, -0.7, 0.0, 0.01, 1),  # duration = 3 วินาที
            #("Backward-Left", -0.7, 0.7, 0.0, 1, 1),
            #("Stop", -0.7, 0.7, 0.0, 0.01, 1),
            #("Backward-Right", -0.7, -0.7, 0.0, 1, 1),
            #("STOP", -0.7, -0.7, 0.0, 0.01,1),
            #("Rotate", 0.0, 0.0, 3.0, 1, 1),
            #("Stop", 0.0, 0.0, 0.01, 1, 1),
            #("Forward-rotate", 0.7, 0.7, 0.7, 1, 5),
            #("Curve-left", 0.7, 0.0, 0.7, 1, 1),
        ]

        self.index = 0
        self.start_time = time.time()  # เวลาที่เริ่มคำสั่งแรก
        self.command_duration = self.commands[0][5]  # ระยะเวลาของคำสั่งแรก
        self.sent_first_command = False  # ติดตามว่าเราส่งคำสั่งไปแล้วหรือยัง

    def timer_callback(self):
        now = time.time()
        if self.index < len(self.commands):
            # ตรวจสอบว่าเวลาผ่านไปตามคำสั่งที่กำหนดแล้ว
            if not self.sent_first_command:
                # ส่งคำสั่งครั้งแรก
                label, x, y, z, scale, duration = self.commands[self.index]
                cmd = Twist()
                cmd.linear.x = x * scale
                cmd.linear.y = y * scale
                cmd.angular.z = z * scale
                self.pub.publish(cmd)
                self.get_logger().info(f'▶ Command: {label} | x={cmd.linear.x:.2f}, y={cmd.linear.y:.2f}, z={cmd.angular.z:.2f} | Duration: {duration} seconds')
                self.sent_first_command = True
                self.start_time = now  # ตั้งเวลาปัจจุบันเมื่อเริ่มส่งคำสั่ง

            # เช็คว่าเวลาผ่านไปตามที่กำหนดแล้ว
            if now - self.start_time > self.command_duration:
                self.index += 1  # เปลี่ยนไปคำสั่งถัดไป
                if self.index < len(self.commands):
                    self.sent_first_command = False  # เตรียมส่งคำสั่งถัดไป
                    self.command_duration = self.commands[self.index][5]  # อัปเดตระยะเวลาใหม่
                    self.start_time = now  # ตั้งเวลาใหม่เมื่อเปลี่ยนคำสั่ง

        else:
            self.stop_robot()  # หยุดหุ่นยนต์เมื่อหมดคำสั่ง

    def stop_robot(self):
        """ส่งคำสั่งหยุดหุ่นยนต์"""
        stop_cmd = Twist()
        self.pub.publish(stop_cmd)
        self.get_logger().info("🛑 หยุดเคลื่อนที่แล้ว (ส่ง cmd_vel = ศูนย์)")
        self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = SwerveAutoCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('👋 ถูกกด Ctrl+C หยุดหุ่นยนต์...')
        node.stop_robot()

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()





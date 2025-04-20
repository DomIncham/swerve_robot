
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SwerveAutoCommander(Node):
    def __init__(self):
        super().__init__('swerve_auto_commander')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # รายการคำสั่งพิเศษพร้อม speed scaling (scale = ความเร็วสัมพัทธ์)
        # ("label", linear.x, linear.y, angular.z, speed_scale)
        self.commands = [
            ("Forward", 1.0, 0.0, 0.0, 1),
            ("Forward-right", 0.7, 0.7, 0.0, 1),
            ("Rotate", 0.0, 0.0, 4.0, 1),
            ("Right", 0.0, 1.0, 0.0, 1),
            ("Forward-rotate", 0.8, 0.0, 0.8, 1),
            ("Backward-left", -0.7, -0.7, 0.0, 1),
        ]

        self.index = 0
        self.start_time = time.time()
        self.command_duration = 4  # วินาทีต่อคำสั่ง

    def timer_callback(self):
        now = time.time()
        if self.index < len(self.commands):
            if now - self.start_time > self.command_duration:
                self.index += 1
                self.start_time = now
            if self.index < len(self.commands):
                label, x, y, z, scale = self.commands[self.index]
                cmd = Twist()
                cmd.linear.x = x * scale
                cmd.linear.y = y * scale
                cmd.angular.z = z * scale
                self.pub.publish(cmd)
                self.get_logger().info(f'▶ Command: {label} | x={cmd.linear.x:.2f}, y={cmd.linear.y:.2f}, z={cmd.angular.z:.2f}')
        else:
            self.pub.publish(Twist())  # หยุด
            self.get_logger().info("🛑 Motion finished.")
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = SwerveAutoCommander()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException

class SwerveJoyController(Node):
    def __init__(self):
        super().__init__('swerve_joy_controller')

        self.cmd_vel_pub = self.create_publisher(Twist, '/swerve_drive/cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.linear_speed = 1.0
        self.angular_speed = 3.0

        self.current_dpad_x = 0.0
        self.current_dpad_y = 0.0
        self.current_rotate = 0.0
        self.lb_pressed = False

        self.last_twist = Twist()
        self.active_command = None

        # ตั้ง timer ทุก 50ms เช็คสถานะ
        self.timer = self.create_timer(0.1, self.update_cmd_vel)

    def joy_callback(self, msg: Joy):
        self.lb_pressed = (msg.buttons[7] == 1)
        self.current_dpad_x = msg.axes[6]
        self.current_dpad_y = msg.axes[7]
        self.current_rotate = msg.axes[2]

    def update_cmd_vel(self):
        if not self.lb_pressed:
            if self.active_command is not None or self.twist_changed(self.last_twist, Twist()):
                self.publish_stop()
                self.last_twist = Twist()
                self.active_command = None
            return

        twist = Twist()
        new_command = None

        # ===== Diagonal Movement =====
        if self.current_dpad_x != 0.0 and self.current_dpad_y != 0.0:
            twist.linear.x = 0.7 * (1.0 if self.current_dpad_y > 0 else -1.0)
            twist.linear.y = 0.7 * (1.0 if self.current_dpad_x > 0 else -1.0)
            new_command = 'diagonal'

        # ===== Straight Movement =====
        elif self.current_dpad_y != 0.0:
            twist.linear.x = 1.0 * (1.0 if self.current_dpad_y > 0 else -1.0)
            twist.linear.y = 0.0
            new_command = 'move_y'

        elif self.current_dpad_x != 0.0:
            twist.linear.x = 0.0
            twist.linear.y = 1.0 * (1.0 if self.current_dpad_x > 0 else -1.0)
            new_command = 'move_x'

        # ===== Rotation =====
        if self.current_rotate == 1.0 or self.current_rotate == -1.0:
            twist.angular.z = self.current_rotate * self.angular_speed
            new_command = 'rotate'

        # ===== ส่งเฉพาะเมื่อค่าจริงเปลี่ยน =====
        if new_command is not None:
            if self.twist_changed(twist, self.last_twist):
                self.cmd_vel_pub.publish(twist)
                self.last_twist = twist
            self.active_command = new_command
        else:
            if self.twist_changed(self.last_twist, Twist()):
                self.publish_stop()
                self.last_twist = Twist()
            self.active_command = None

    def publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    def twist_changed(self, a: Twist, b: Twist) -> bool:
        return (
            abs(a.linear.x - b.linear.x) > 1e-3 or
            abs(a.linear.y - b.linear.y) > 1e-3 or
            abs(a.angular.z - b.angular.z) > 1e-3
        )

def main(args=None):
    rclpy.init(args=args)
    node = SwerveJoyController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().warn("📴 Swerve Commander shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()

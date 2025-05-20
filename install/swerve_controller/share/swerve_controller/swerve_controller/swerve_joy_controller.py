#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class SwerveJoyController(Node):
    def __init__(self):
        super().__init__('swerve_joy_controller')

        self.cmd_vel_pub = self.create_publisher(Twist, '/swerve_drive/cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # ตั้งค่าความเร็วตามโหมด Auto
        self.linear_speed = 1.0  # ใช้ 1.0 แล้วค่อยปรับในโค้ด
        self.angular_speed = 3.0  # ตั้งค่าเหมือนโหมด Auto (Rotate)
        
        # เก็บสถานะก่อนหน้า
        self.prev_dpad_x = 0.0
        self.prev_dpad_y = 0.0
        self.prev_rotate = 0.0
        self.prev_lb_state = 0
        
        # เก็บสถานะปัจจุบัน
        self.active_command = None

    def joy_callback(self, msg: Joy):
        current_lb = msg.buttons[6]  # ปุ่ม LB
        current_dpad_x = msg.axes[6]  # -1=left, 1=right
        current_dpad_y = msg.axes[7]  # 1=up, -1=down
        current_rotate = msg.axes[2]  # แกนหมุน

        # ตรวจสอบการเปลี่ยนแปลง
        lb_changed = (current_lb != self.prev_lb_state)
        dpad_x_changed = (current_dpad_x != self.prev_dpad_x)
        dpad_y_changed = (current_dpad_y != self.prev_dpad_y)
        rotate_changed = (current_rotate != self.prev_rotate)
        
        if not (lb_changed or dpad_x_changed or dpad_y_changed or rotate_changed):
            return
            
        self.prev_lb_state = current_lb
        self.prev_dpad_x = current_dpad_x
        self.prev_dpad_y = current_dpad_y
        self.prev_rotate = current_rotate

        # ถ้าไม่กด LB ให้หยุด
        if current_lb != 1:
            if self.active_command is not None:
                self.publish_stop()
                self.active_command = None
            return

        twist = Twist()
        new_command = None

        # จัดการการเคลื่อนที่แบบต่างๆ ตามโหมด Auto
        if current_dpad_x != 0.0 or current_dpad_y != 0.0:
            # Diagonal movements (เฉียง)
            if current_dpad_x > 0 and current_dpad_y > 0:  # Forward-Left
                twist.linear.x = 0.7  # vx
                twist.linear.y = 0.7  # vy
            elif current_dpad_x > 0 and current_dpad_y < 0:  # Forward-Right
                twist.linear.x = 0.7   # vx
                twist.linear.y = -0.7  # vy
            elif current_dpad_x < 0 and current_dpad_y > 0:  # Backward-Left
                twist.linear.x = -0.7  # vx
                twist.linear.y = 0.7   # vy
            elif current_dpad_x < 0 and current_dpad_y < 0:  # Backward-Right
                twist.linear.x = -0.7  # vx
                twist.linear.y = -0.7  # vy
            # การเคลื่อนที่แนวตรง
            elif current_dpad_y > 0:  # Forward
                twist.linear.x = 1.0
                twist.linear.y = 0.0
            elif current_dpad_y < 0:  # Backward
                twist.linear.x = -1.0
                twist.linear.y = 0.0
            elif current_dpad_x > 0:  # Right
                twist.linear.x = 0.0
                twist.linear.y = -1.0
            elif current_dpad_x < 0:  # Left
                twist.linear.x = 0.0
                twist.linear.y = 1.0
            
            new_command = 'move'
        
        # การหมุน (Rotate)
        if current_rotate == 1.0 or current_rotate == -1.0:
            twist.angular.z = current_rotate * self.angular_speed
            new_command = 'rotate'

        # ส่งคำสั่งเมื่อมีสถานะเปลี่ยนแปลง
        if new_command != self.active_command:
            if new_command is None:
                self.publish_stop()
            else:
                self.cmd_vel_pub.publish(twist)
            self.active_command = new_command

    def publish_stop(self):
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)


def main():
    rclpy.init()
    node = SwerveJoyController()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
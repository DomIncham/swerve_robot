#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import gpiod
import time
import os
import signal
import psutil

CHIP_NAME = 'gpiochip4'
LINE_OFFSET = 17  # GPIO17 = Pin 11

class EmergencyButtonNode(Node):
    def __init__(self):
        super().__init__('emergency_shutdown_node')
        self.chip = gpiod.Chip(CHIP_NAME)
        self.line = self.chip.get_line(LINE_OFFSET)
        self.line.request(
            consumer="emergency_button",
            type=gpiod.LINE_REQ_DIR_IN,
            flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP
        )

        self.get_logger().info(f'🚨 Listening on GPIO{LINE_OFFSET} via {CHIP_NAME}')
        self._triggered = False
        self.timer = self.create_timer(0.1, self.check_button)

    def check_button(self):
        if not self._triggered and self.line.get_value() == 0:
            self._triggered = True
            self.get_logger().warn('🛑 Emergency button pressed! Shutting down all nodes...')
            self.timer.cancel()
            time.sleep(0.1)  # debounce

            # ส่ง SIGINT ไปยังกระบวนการทั้งหมดในกลุ่ม process group
            try:
                current_process = psutil.Process(os.getpid())
                parent_pid = current_process.ppid()
                parent = psutil.Process(parent_pid)
                
                # ส่ง SIGINT ไปยัง parent process (launch system)
                parent.send_signal(signal.SIGINT)
                self.get_logger().info(f'Sent SIGINT to parent PID: {parent_pid}')
                
                # ส่ง SIGINT ไปยังกระบวนการลูกทั้งหมด
                for child in parent.children(recursive=True):
                    try:
                        child.send_signal(signal.SIGINT)
                        self.get_logger().info(f'Sent SIGINT to child PID: {child.pid}')
                    except psutil.NoSuchProcess:
                        pass
                
                # ส่ง SIGINT ไปยังกระบวนการของตัวเอง (กรณีที่จำเป็น)
                os.kill(os.getpid(), signal.SIGINT)
                
            except Exception as e:
                self.get_logger().error(f'Error during shutdown: {str(e)}')
                # ถ้าไม่สำเร็จ ให้ปิด node นี้และเรียก rclpy.shutdown()
                self.destroy_node()
                rclpy.shutdown()

    def destroy_node(self):
        self.line.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyButtonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # ปล่อยให้ launch system shutdown
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import os
import signal
import time

EMERGENCY_PIN = 17  # 🧷 GPIO17 = Pin 11 บน Pi header

class EmergencyButtonNode(Node):
    def __init__(self):
        super().__init__('emergency_button_node')
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(EMERGENCY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.get_logger().info(f'🚨 Emergency button listening on GPIO{EMERGENCY_PIN}')
        self.timer = self.create_timer(0.1, self.check_button)

    def check_button(self):
        if GPIO.input(EMERGENCY_PIN) == GPIO.LOW:
            self.get_logger().warn('🛑 Emergency button pressed! Shutting down...')
            time.sleep(0.2)
            os.kill(os.getppid(), signal.SIGINT)

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyButtonNode()
    try:
        rclpy.spin(node)
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

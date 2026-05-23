#!/usr/bin/env python3
# servo_gripper_control.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import Empty, Trigger
from .motor_control import SERVO_CONTROL  # import your new servo class
import time


class ServoGripperControl(Node):
    def __init__(self):
        super().__init__('gripper_control')

        # === ROS2 Interfaces === #
        # Subscriptions
        self.subscription_angle = self.create_subscription(
            Int32,
            'gripper/servo_angle',
            self.angleCallback,
            10
        )
        

        # Services
        self.home_srv = self.create_service(Empty, 'gripper/home', self.homeCallback)
        self.read_srv = self.create_service(Trigger, 'gripper/read_angle', self.readAngleCallback)
        self.create_service(Empty, '/gripper/open', self.open_callback)
        self.create_service(Empty, '/gripper/open_wide', self.open_wide_callback)
        self.create_service(Empty, '/gripper/close', self.close_callback)



        # === Servo Controller Setup === #
        self.com_port = '/dev/ttyUSB0'
        self.baudrate = 115200
        self.previous_angle = -999
        self.cmd_timeout = 5

        self.servo = SERVO_CONTROL(com_port=self.com_port, baudrate=self.baudrate)

        if not self.servo.open():
            self.get_logger().error('Failed to open serial port for servo controller')
            raise SystemExit

        self.get_logger().info('Connected to servo gripper controller')

        # Optional: send home command on start
        self.servo.home()

    # === ROS2 Callbacks === #
    def angleCallback(self, msg: Int32):
        """Move the gripper to a specified angle (0–180)."""
        angle = msg.data
        self.get_logger().info(f"Received angle command: {angle}")

        # Prevent duplicate commands
        if angle == self.previous_angle:
            self.get_logger().info("Same as previous command. Ignoring.")
            return

        # Safety clamp
        angle = max(0, min(angle, 180))

        # Try sending the command a few times if needed
        done = False
        count = 0
        while not done:
            ret = self.servo.set_angle(angle)
            if ret is not None:
                done = True
            else:
                count += 1
                if count >= self.cmd_timeout:
                    self.get_logger().error("Failed to set angle after retries.")
                    break
                time.sleep(0.2)

        self.previous_angle = angle

    def homeCallback(self, request, response):
        """Move gripper to home position."""
        self.get_logger().info("Returning to home position...")
        self.servo.home()
        response = Empty.Response()
        return response

    def readAngleCallback(self, request, response):
        """Read current angle from servo."""
        angle = self.servo.read_angle()
        if angle:
            response.success = True
            response.message = str(angle)
            self.get_logger().info(f"Servo current angle: {angle}")
        else:
            response.success = False
            response.message = "Failed to read angle"
        return response
    
    def open_callback(self, request, response):
        self.get_logger().info("Opening gripper")
        self.servo.open_gripper()
        return response

    def open_wide_callback(self, request, response):
        self.get_logger().info("Opening gripper wide")
        self.servo.open_wide_gripper()
        return response

    def close_callback(self, request, response):
        self.get_logger().info("Closing gripper")
        self.servo.close_gripper()
        return response

    def process(self):
        """Main ROS2 spin loop."""
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
        except KeyboardInterrupt:
            pass
        finally:
            self.servo.close()
            self.get_logger().info("Serial connection closed")


def main(args=None):
    rclpy.init(args=args)
    node = ServoGripperControl()
    node.process()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

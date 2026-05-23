from gripper_control_pkg.motor_control import SERVO_CONTROL
import time

# Adjust com_port if necessary
servo = SERVO_CONTROL(com_port='/dev/ttyUSB0', baudrate=115200)

try:
    if servo.open():
        print("Connected to servo. Running demo sequence...")
        servo.home()
        time.sleep(1)
        servo.open_gripper()
        time.sleep(1)
        servo.close_gripper()
        time.sleep(1)
        servo.open_gripper_wide()
        time.sleep(1)
        servo.set_angle(90)
        time.sleep(1)
        servo.set_angle(0)
        print("Demo complete!")
    else:
        print("Failed to open serial port")
finally:
    servo.close()

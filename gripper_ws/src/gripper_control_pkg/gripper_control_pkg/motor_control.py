# servo_control.py

import serial
import time


class SERVO_CONTROL:
    def __init__(self, id=0x01, com_port='/dev/ttyUSB0', baudrate=115200):
        """
        Initialize serial connection for servo control.
        :param id: Servo ID (optional, used if multiple servos exist)
        :param com_port: Serial port connected to ESP32 or controller
        :param baudrate: Baud rate for serial communication
        """
        self.id = id
        self.ser = serial.Serial()
        self.ser.baudrate = baudrate
        self.ser.port = com_port
        self.ser.timeout = 1

    # ---------------- Serial Connection ---------------- #
    def open(self):
        """Open serial connection."""
        self.ser.open()
        time.sleep(2)  # Allow ESP32 to reset
        if not self.ser.is_open:
            print("Cannot open serial port")
        else:
            print("Connected to servo controller")
        return self.ser.is_open

    def close(self):
        """Close serial connection."""
        self.ser.close()
        if not self.ser.is_open:
            print("Serial connection closed")
        return not self.ser.is_open

    # ---------------- Checksum (Optional) ---------------- #
    def chkSum(self, data):
        """Compute checksum from list of bytes."""
        return sum(data) & 0xFF

    # ---------------- Communication Helpers ---------------- #
    def write(self, command):
        """Send a string command to ESP32 (with newline)."""
        cmd_str = f"{command}\n"
        self.ser.write(cmd_str.encode('utf-8'))
        print(f"Sent: {command}")

    # def read(self):
    #     """Read line from serial (response from ESP32)."""
    #     response = self.ser.readline().decode('utf-8').strip()
    #     if response:
    #         print(f"Received: {response}")
    #     return response

    def read(self):
        """Safe read from serial without crashing on bad bytes."""
        if not self.ser or not self.ser.is_open:
            print("⚠️ Serial port is closed. Cannot read.")
            return ""

        try:
            raw = self.ser.readline()

            if not raw:
                return ""   # nothing to read

            # Decode safely — skip bad bytes that cause UnicodeDecodeError
            response = raw.decode('utf-8', errors='ignore').strip()

            if response:
                print(f"Received: {response}")

            return response

        except Exception as e:
            print(f"⚠️ Serial read error: {e}")
            return ""


    # ---------------- Servo Commands ---------------- #
    def open_gripper(self):
        """Open the servo gripper."""
        self.write("open")
        return self.read()
    
    def open_wide_gripper(self):
        """Open the servo gripper wide."""
        self.write("open_wide")
        return self.read()

    def close_gripper(self):
        """Close the servo gripper."""
        self.write("close")
        return self.read()

    def home(self):
        """Set current position as home."""
        self.write("home")
        return self.read()

    def set_angle(self, angle):
        """Set servo to a specific angle (0–180)."""
        self.write(f"angle:{angle}")
        return self.read()

    def read_angle(self):
        """Request current servo angle from ESP32."""
        self.write("read_angle")
        return self.read()

    def enable_torque(self, enable=True):
        """Enable or disable servo torque."""
        cmd = "torque_on" if enable else "torque_off"
        self.write(cmd)
        return self.read()

    # ---------------- Example Utility ---------------- #
    def demo_sequence(self):
        """Run a test motion sequence."""
        print("Running demo sequence...")
        self.home()
        time.sleep(1)
        self.open_gripper()
        time.sleep(1)
        self.close_gripper()
        time.sleep(1)
        self.set_angle(90)
        time.sleep(1)
        self.set_angle(0)
        print("Demo complete!")


# ---------------- Run as Main Script ---------------- #
if __name__ == "__main__":
    servo = SERVO_CONTROL(com_port='/dev/ttyUSB0', baudrate=115200)

    try:
        if servo.open():
            servo.demo_sequence()
    finally:
        servo.close()

#!/usr/bin/env python3
"""
set_tcp_offset.py
------------------------------------------------------------
Sets the Tool Center Point (TCP) offset on the xArm hardware
so it matches the MoveIt 'center_tip' frame.
------------------------------------------------------------
"""

import time
from xarm.wrapper import XArmAPI

ROBOT_IP = "192.168.1.213"  # your xArm IP
TCP_OFFSET = [0, 0, 190, 180, -90, 0]  # mm + deg

def main():
    print(f"🔌 Connecting to xArm at {ROBOT_IP} ...")
    arm = XArmAPI(ROBOT_IP)   # constructor already connects

    # Wait a moment to ensure connection stable
    time.sleep(0.5)

    print(f"⚙️ Setting TCP offset to {TCP_OFFSET} ...")
    ret = arm.set_tcp_offset(TCP_OFFSET)
    if ret == 0:
        print("✅ TCP offset set successfully.")
    else:
        print(f"⚠️ set_tcp_offset returned code {ret}")

    # Optional: check current TCP pose
    tcp = arm.get_position(is_radian=False)
    print(f"📏 Current TCP pose: {tcp}")

    arm.disconnect()
    print("🔌 Disconnected.")

if __name__ == "__main__":
    main()

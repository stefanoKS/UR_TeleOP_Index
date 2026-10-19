
import sys
from pathlib import Path

sys.path.insert(
    0,
    str(Path.home() / "Documents/Teleop/ur_teleop_ws/src/ur_bus_servo_gripper")
)

from ur_bus_servo_gripper.gripper_config import Config
from ur_bus_servo_gripper.st3215_driver import Driver, checked

PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7B058089-if00"

driver = Driver(Config(port=PORT, servo_id=1))

try:
    feedback = driver.connect()
    packet = driver.packet

    target = checked(packet.read2ByteTxRx(1, 67))
    positive_deadzone = checked(packet.read1ByteTxRx(1, 26))
    negative_deadzone = checked(packet.read1ByteTxRx(1, 27))

    print("Measured position:", feedback.ticks)
    print("Stored target:", target)
    print("Position error:", target - feedback.ticks, "ticks")
    print("Positive deadzone:", positive_deadzone)
    print("Negative deadzone:", negative_deadzone)
    print("Motor status:", feedback.motor_status)

finally:
    driver.close()


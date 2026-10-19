
import sys
import time
from pathlib import Path
from dataclasses import replace

# Locate your existing ST3215 driver
project = Path.home() / "Documents/Teleop"

sys.path.insert(
    0,
    str(project / "ur_teleop_ws/src/ur_bus_servo_gripper")
)

from ur_bus_servo_gripper.gripper_config import Config
from ur_bus_servo_gripper.st3215_driver import Driver

# Your confirmed Waveshare adapter and ST3215 ID
PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7B058089-if00"
SERVO_ID = 1

# Only a very small test movement
STEP = 500

config = Config(port=PORT, servo_id=SERVO_ID)
driver = Driver(config)

try:
    print("=== ST3215 MOTOR TEST ===")

    feedback = driver.connect()
    start = feedback.ticks

    print("Servo ID:", driver.servo_id)
    print("Position:", start, "ticks")
    print("Voltage:", feedback.voltage_v, "V")
    print("Temperature:", feedback.temperature_c, "C")
    print("Motor status:", feedback.motor_status)

    if feedback.motor_status != 0:
        raise RuntimeError("Motor fault. Test cancelled.")

    print()
    print("Check that the rack has room to move.")
    direction = input("Choose direction (+ or -): ").strip()

    if direction not in ("+", "-"):
        raise SystemExit("Cancelled.")

    delta = STEP if direction == "+" else -STEP
    target = start + delta

    if not (
        driver.identity["firmware_min"]
        <= target
        <= driver.identity["firmware_max"]
    ):
        raise RuntimeError("Target outside servo limits.")

    # Temporary limits: only current position +/- 5 ticks
    driver.config = replace(
        config,
        open_ticks=min(start, target),
        closed_ticks=max(start, target),
        min_ticks=min(start, target),
        max_ticks=max(start, target),
        speed=20,
        acceleration=1,
    )

    print()
    print("Start position:", start)
    print("Target position:", target)
    print("Difference:", delta, "ticks")
    print()

    confirm = input("Type MOVE to continue: ").strip()

    if confirm != "MOVE":
        raise SystemExit("Cancelled.")

    print("Sending movement command...")
    driver.write(target, calibration=True)

    print()
    print("=== MOTOR FEEDBACK ===")

    for i in range(10):
        time.sleep(0.2)
        feedback = driver.read()

        print(
            "Position:", feedback.ticks,
            "| Moving:", feedback.moving,
            "| Load:", feedback.load_raw,
            "| Current:", feedback.current_raw
        )

        if feedback.motor_status != 0:
            print("Motor fault:", feedback.motor_status)
            break

    print()
    print("Test finished.")
    print("No automatic return movement.")

finally:
    driver.close()
    print("Serial port closed.")



import time
from jidenna import JidennaDriver


robot = JidennaDriver(port="COM9",baudrate=115200,startup_delay=10.0)


def move(v, w, duration):
    print(f"\nMoving: v={v} m/s, w={w} rad/s")

    start = time.monotonic()

    # Refresh commands so the 500ms watchdog doesn't expire
    while time.monotonic() - start < duration:
        robot.set_velocity(v, w)
        time.sleep(0.1)

    robot.stop()
    print("Stopped")
    time.sleep(2)


try:
    print("Connecting to Jidenna...")
    robot.connect()

    print("Waiting for telemetry...")
    deadline = time.monotonic() + 5.0

    while time.monotonic() < deadline:
        status = robot.get_status()
        if (status["telemetry_fresh"]
                and status["motor_feedback_fresh"]):
            break
        time.sleep(0.1)
    else:
        raise RuntimeError("Robot telemetry or motor feedback unavailable")

    print("Robot ready!")

    # Test 1: Forward
    input("\nPress ENTER to test FORWARD...")
    move(0.8, 0.0, 2)

    # Test 2: Backward
    input("\nPress ENTER to test BACKWARD...")
    move(-0.8, 0.0, 2)

    # Test 3: Rotate counterclockwise
    input("\nPress ENTER to rotate LEFT...")
    move(0.0, 1.0, 2)

    # Test 4: Rotate clockwise
    input("\nPress ENTER to rotate RIGHT...")
    move(0.0, -1.0, 2)

    print("\nAll motor tests completed!")

except KeyboardInterrupt:
    print("\nTest interrupted!")

finally:
    robot.disconnect()
    print("Jidenna disconnected.")

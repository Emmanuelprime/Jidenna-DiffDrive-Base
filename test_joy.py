
import time
import threading

from jidenna import JidennaDriver
from jidenna import UDPJoystickController


def monitor_robot(robot, stop_event):
    """Print odometry and IMU data while driving."""

    while not stop_event.is_set():

        status = robot.get_status()

        if status["telemetry_fresh"]:

            odom = robot.get_odometry_data()
            imu = robot.get_imu_gyro()

            if odom is not None and imu is not None:

                print(
                    f"\n[ODOMETRY] "
                    f"X: {odom['x']:.3f} m | "
                    f"Y: {odom['y']:.3f} m | "
                    f"Theta: {odom['theta']:.3f} rad"
                )

                print(
                    f"[IMU] "
                    f"Yaw: {imu['yaw']:.3f} rad | "
                    f"Gyro Z: {imu['gyro_z']:.3f} rad/s"
                )

                print("-" * 65)

        else:
            print("[WARNING] Telemetry unavailable or stale")

        stop_event.wait(0.2)  # Print at 5 Hz


robot = JidennaDriver(
    port="/dev/ttyUSB0",
    startup_delay=10.0
)

stop_event = threading.Event()
monitor_thread = None

try:
    robot.connect()

    controller = UDPJoystickController(
        robot,
        port=4210
    )

    print("Waiting for fresh telemetry and hoverboard feedback...")

    deadline = time.monotonic() + 5

    while time.monotonic() < deadline:
        status = robot.get_status()

        if (
            status["telemetry_fresh"]
            and status["motor_feedback_fresh"]
        ):
            break

        time.sleep(0.1)

    else:
        raise RuntimeError("No fresh Arduino/motor feedback")

    print("Robot ready!")
    print("UDP joystick control active.")
    print("Monitoring odometry and IMU data...\n")

    # Start background telemetry monitor
    monitor_thread = threading.Thread(
        target=monitor_robot,
        args=(robot, stop_event),
        daemon=True
    )

    monitor_thread.start()

    # Receive joystick commands and drive robot
    controller.run()

except KeyboardInterrupt:
    print("\nStopping controller...")

finally:
    stop_event.set()

    if monitor_thread is not None:
        monitor_thread.join(timeout=1.0)

    robot.disconnect()

    print("Jidenna disconnected safely.")


import threading
import time

from jidenna import JidennaDriver, UDPJoystickController, Localization


robot = JidennaDriver(
    port="/dev/ttyUSB0",
    startup_delay=10.0
)

stop_event = threading.Event()
monitor_thread = None


def monitor(loc):
    while not stop_event.is_set():

        loc.update()

        # EKF estimated pose
        pose = loc.get_pose()

        # Raw odometry from Arduino
        odom = robot.get_odometry_data()

        # MPU6050 measurements
        imu = robot.get_imu_gyro()

        status = loc.get_status()["status"]

        if (
            pose is not None
            and odom is not None
            and imu is not None
            and status == "ok"
        ):
            print(
                f"\n[RAW ODOM] "
                f"x={odom['x']:.3f} "
                f"y={odom['y']:.3f} "
                f"theta={odom['theta']:.3f}\n"

                f"[EKF POSE] "
                f"x={pose['x']:.3f} "
                f"y={pose['y']:.3f} "
                f"theta={pose['theta']:.3f}\n"

                f"[IMU]      "
                f"yaw={imu['yaw']:.3f} "
                f"gyro_z={imu['gyro_z']:.3f}\n"

                f"{'-' * 65}",
                flush=True
            )

        elif status != "ok":
            print(
                f"Localization: {status}",
                flush=True
            )

        stop_event.wait(0.1)


try:
    robot.connect()

    print("Waiting for fresh Arduino telemetry...")

    deadline = time.monotonic() + 5

    while time.monotonic() < deadline:
        st = robot.get_status()

        if (
            st["telemetry_fresh"]
            and st["motor_feedback_fresh"]
        ):
            break

        time.sleep(0.1)

    else:
        raise RuntimeError(
            "No fresh Arduino/motor feedback"
        )

    loc = Localization(robot)

    monitor_thread = threading.Thread(
        target=monitor,
        args=(loc,),
        daemon=True
    )

    monitor_thread.start()

    print("UDP joystick active.")
    print("Monitoring raw odometry, EKF and IMU.")
    print("Ctrl+C to stop.")

    UDPJoystickController(
        robot,
        port=4210
    ).run()

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    stop_event.set()

    if monitor_thread:
        monitor_thread.join(timeout=1)

    robot.disconnect()

    print("Jidenna disconnected.")

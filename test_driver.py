import time
from jidenna import JidennaDriver

with JidennaDriver(port="/dev/ttyUSB0",startup_delay=10.0) as robot:

    try:
        while True:
            imu = robot.get_imu_gyro()
            status = robot.get_status()

            if imu and status["telemetry_fresh"]:
                print(
                    f"Yaw: {imu['yaw']:.4f} rad | "
                    f"Gyro Z: {imu['gyro_z']:.4f} rad/s"
                )

            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
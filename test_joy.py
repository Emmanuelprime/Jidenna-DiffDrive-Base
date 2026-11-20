
import math
import time

from jidenna import (
    JidennaDriver,
    UDPJoystickController,
)

from jidenna.localization import Odometry


def main():

    with JidennaDriver(
        port="/dev/ttyUSB0",
        baudrate=115200,
        command_rate=20.0,
        command_timeout=0.5,
    ) as robot:

        with UDPJoystickController(
            robot=robot,
            port=4210,
            max_linear=0.65,
            max_angular=2.49,
            invert_angular=True,
            timeout=0.5,
            command_rate=20.0,
        ) as joystick:

            with Odometry(
                robot=robot,
                wheel_separation=0.521,
                update_rate=20.0,
            ) as odom:

                print("\nJIDENNA ODOMETRY MONITOR")
                print("=" * 75)
                print("Press Ctrl+C to stop\n")

                while True:

                    # Odometry state
                    pose = odom.get_state()

                    # Raw Arduino telemetry
                    raw = robot.get_state()

                    # Joystick state
                    joy = joystick.get_status()

                    print("\n" + "=" * 75)

                    print("ROBOT POSE")
                    print(
                        f"X: {pose.x:+.3f} m | "
                        f"Y: {pose.y:+.3f} m | "
                        f"Theta: "
                        f"{math.degrees(pose.theta):+.2f} deg"
                    )

                    print("\nODOMETRY VELOCITY")
                    print(
                        f"Linear: {pose.v:+.3f} m/s | "
                        f"Angular: {pose.w:+.3f} rad/s"
                    )

                    print("\nWHEEL VELOCITIES")
                    print(
                        f"Left: {pose.v_left:+.3f} m/s | "
                        f"Right: {pose.v_right:+.3f} m/s"
                    )

                    if raw is not None:
                        print("\nARDUINO TELEMETRY")
                        print(
                            f"Left: {raw.left_speed:+.3f} m/s | "
                            f"Right: {raw.right_speed:+.3f} m/s"
                        )

                        print(
                            f"IMU Yaw: "
                            f"{raw.imu_yaw_deg:+.2f} deg | "
                            f"Gyro Z: "
                            f"{raw.gyro_z_deg_s:+.2f} deg/s"
                        )

                    print("\nJOYSTICK")
                    print(
                        f"Command: {joy['velocity']} | "
                        f"Active: {joy['active']} | "
                        f"Packets: {joy['packet_count']}"
                    )

                    print("=" * 75)

                    time.sleep(0.5)


if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        print("\nStopping Jidenna...")

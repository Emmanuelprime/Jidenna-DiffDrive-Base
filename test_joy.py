
import math
import time

from jidenna import (
    JidennaDriver,
    UDPJoystickController,
)

from jidenna.localization import Odometry


def print_pose(name, pose):

    x, y, theta = pose

    distance = math.hypot(x, y)

    print(
        f"{name:<15} | "
        f"X: {x:+.3f} m | "
        f"Y: {y:+.3f} m | "
        f"Theta: {math.degrees(theta):+.2f} deg | "
        f"Distance: {distance:.3f} m"
    )


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
        ):

            with Odometry(
                robot=robot,
                wheel_separation=0.521,
                update_rate=20.0,
                mode="wheel",
                alpha=0.5,
            ) as odom:

                print("\nJIDENNA LOCALIZATION TEST")
                print("Wheel vs IMU vs Complementary")
                print("Press Ctrl+C to stop\n")

                while True:

                    poses = odom.get_all_poses()

                    print("\n" + "=" * 95)

                    print_pose(
                        "WHEEL",
                        poses["wheel"],
                    )

                    print_pose(
                        "IMU",
                        poses["imu"],
                    )

                    print_pose(
                        "COMPLEMENTARY",
                        poses["complementary"],
                    )

                    raw = robot.get_state()

                    if raw is not None:

                        wheel_w = (
                            raw.right_speed - raw.left_speed
                        ) / 0.521

                        gyro_w = math.radians(
                            raw.gyro_z_deg_s
                        )

                        print("\nSENSOR COMPARISON")

                        print(
                            f"Left: {raw.left_speed:+.3f} m/s | "
                            f"Right: {raw.right_speed:+.3f} m/s"
                        )

                        print(
                            f"Wheel W: {wheel_w:+.3f} rad/s | "
                            f"Gyro W: {gyro_w:+.3f} rad/s"
                        )

                        print(
                            f"IMU Yaw: "
                            f"{raw.imu_yaw_deg:+.2f} deg"
                        )

                    print("=" * 95)

                    time.sleep(0.5)


if __name__ == "__main__":

    try:
        main()

    except KeyboardInterrupt:
        print("\nStopping Jidenna...")

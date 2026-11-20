
import time

from jidenna import (
    JidennaDriver,
    UDPJoystickController,
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
            max_linear=2.1,
            max_angular=2.49,
            invert_linear=False,
            invert_angular=True,
            timeout=0.5,
            command_rate=20.0,
        ) as joystick:

            print("\nJoystick controller started")
            print("Press Ctrl+C to stop\n")

            while True:
                joy = joystick.get_status()
                state = robot.get_state()

                print(
                    f"JOY: {joy['velocity']} | "
                    f"Active: {joy['active']} | "
                    f"Packets: {joy['packet_count']} | "
                    f"Age: {joy['packet_age_s']}"
                )

                if state:
                    print(
                        f"LEFT: {state.left_speed:.3f} | "
                        f"RIGHT: {state.right_speed:.3f} | "
                        f"YAW: {state.imu_yaw_deg:.2f}"
                    )

                print("-" * 60)

                time.sleep(0.5)


if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        print("\nStopping Jidenna...")

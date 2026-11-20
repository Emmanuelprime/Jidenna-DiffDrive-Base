import time

from jidenna import JidennaDriver, UDPJoystickController


def main():
    with JidennaDriver(port="/dev/ttyUSB0",baudrate=115200,command_timeout=0.3,) as robot:

        with UDPJoystickController(robot=robot,host="0.0.0.0",port=4210,max_linear=2.1,
            max_angular=2.49,invert_linear=False,invert_angular=False,timeout=0.3) as joystick:

            print("\nJidenna joystick control started")
            print("Press Ctrl+C to stop\n")

            while True:
                status = joystick.get_status()
                state = robot.get_state()

                print(
                    f"Joystick: {status['velocity']} | "
                    f"Active: {status['active']}"
                )

                if state is not None:
                    print(
                        f"Left: {state.left_speed:.3f} m/s | "
                        f"Right: {state.right_speed:.3f} m/s | "
                        f"Yaw: {state.imu_yaw_deg:.2f} deg | "
                        f"Gyro: {state.gyro_z_deg_s:.2f} deg/s | "
                        f"Battery: {state.battery_v:.2f} V"
                    )

                print("-" * 65)
                time.sleep(0.5)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopping Jidenna...")
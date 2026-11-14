from jidenna import JidennaDriver
import time

robot = JidennaDriver(port="COM9")

try:
    robot.connect()

    while True:
        odom = robot.get_odometry_data()
        imu = robot.get_imu_gyro()
        battery = robot.get_battery_info()

        print("Odometry:", odom)
        print("IMU:", imu)
        print("Battery:", battery)

        time.sleep(0.5)

finally:
    robot.disconnect()
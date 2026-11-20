import time
from jidenna import JidennaDriver

with JidennaDriver(port="COM9") as robot:

    # Drive forward for 2 seconds
    for _ in range(50):
        robot.set_velocity(1.5, 0.0)
        time.sleep(0.1)

    robot.stop()

    # Read sensors
    state = robot.get_state()

    if state:
        print("Left:", state.left_speed)
        print("Right:", state.right_speed)
        print("Battery:", state.battery_v)
        print("Yaw:", state.imu_yaw_deg)
        print("Gyro:", state.gyro_z_deg_s)
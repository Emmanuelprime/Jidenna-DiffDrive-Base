import threading
import time
from jidenna import JidennaDriver, UDPJoystickController
from jidenna import Localization

robot = JidennaDriver(port='/dev/ttyUSB0', startup_delay=10.0)
stop_event = threading.Event()
monitor_thread = None

def monitor(loc):
    while not stop_event.is_set():
        loc.update()
        pose = loc.get_pose()
        imu = robot.get_imu_gyro()
        status = loc.get_status()['status']
        if pose and status == 'ok' and imu:
            print(f"EKF x={pose['x']:.3f} y={pose['y']:.3f} theta={pose['theta']:.3f} | "
                  f"IMU yaw={imu['yaw']:.3f} gyro_z={imu['gyro_z']:.3f}", flush=True)
        elif status != 'ok':
            print(f'Localization: {status}', flush=True)
        stop_event.wait(0.1)

try:
    robot.connect()
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        st = robot.get_status()
        if st['telemetry_fresh'] and st['motor_feedback_fresh']:
            break
        time.sleep(0.1)
    else:
        raise RuntimeError('No fresh Arduino/motor feedback')
    loc = Localization(robot)
    monitor_thread = threading.Thread(target=monitor, args=(loc,), daemon=True)
    monitor_thread.start()
    print('UDP joystick active. Start with wheels raised. Ctrl+C to stop.')
    UDPJoystickController(robot, port=4210).run()
except KeyboardInterrupt:
    print('Stopping...')
finally:
    stop_event.set()
    if monitor_thread:
        monitor_thread.join(timeout=1)
    robot.disconnect()

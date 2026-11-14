from jidenna import JidennaDriver
from jidenna import UDPJoystickController


robot = JidennaDriver(port='/dev/ttyUSB0', startup_delay=10.0)
try:
    robot.connect()
    controller = UDPJoystickController(robot, port=4210)
    print('Waiting for fresh telemetry and hoverboard feedback...')
    import time
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        status = robot.get_status()
        if status['telemetry_fresh'] and status['motor_feedback_fresh']:
            break
        time.sleep(0.1)
    else:
        raise RuntimeError('No fresh Arduino/motor feedback')
    print('Ready. Keep the robot wheels off the ground. Ctrl+C to exit.')
    controller.run()
except KeyboardInterrupt:
    print('\nStopping controller...')
finally:
    robot.disconnect()

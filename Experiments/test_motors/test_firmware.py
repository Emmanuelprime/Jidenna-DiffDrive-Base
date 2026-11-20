
import serial
import threading
import time
import math

PORT = "COM9"
BAUD = 115200

robot = serial.Serial(PORT, BAUD, timeout=0.2)

print("Waiting for Jidenna...")

deadline = time.monotonic() + 15

while time.monotonic() < deadline:
    line = robot.readline().decode(
        errors="ignore"
    ).strip()

    if line == "# CALIBRATING":
        print("Calibrating MPU6050...")

    elif line == "# READY":
        print("Jidenna is ready!")
        break

else:
    robot.close()
    raise TimeoutError("Jidenna did not send READY")

running = threading.Event()
running.set()

lock = threading.Lock()

velocity = [0.0, 0.0]
latest_telemetry = None


def communicate():
    global latest_telemetry

    next_send = time.monotonic()

    while running.is_set():
        try:
            now = time.monotonic()

            if now >= next_send:
                with lock:
                    v, w = velocity

                robot.write(f"{v:.3f},{w:.3f}\n".encode())
                next_send = now + 0.1

            if robot.in_waiting:
                line = robot.readline().decode(
                    errors="ignore"
                ).strip()

                if line.startswith("#"):
                    continue

                fields = line.split(",")

                if len(fields) == 6:
                    try:
                        values = tuple(map(float, fields))

                        with lock:
                            latest_telemetry = values

                    except ValueError:
                        pass

            time.sleep(0.005)

        except serial.SerialException as exc:
            print(f"\nSerial error: {exc}")
            running.clear()
            break


thread = threading.Thread(
    target=communicate,
    daemon=True
)
thread.start()

print("\nCommands:")
print("  0.3,0   - Forward")
print("  0,1.0   - Rotate")
print("  STOP    - Stop")
print("  STATUS  - Show telemetry")
print("  EXIT    - Quit\n")

try:
    while running.is_set():
        command = input("Jidenna > ").strip()

        if command.upper() == "EXIT":
            break

        if command.upper() == "STOP":
            with lock:
                velocity[:] = [0.0, 0.0]
            robot.write(b"STOP\n")
            print("Stopped")
            continue

        if command.upper() == "STATUS":
            with lock:
                data = latest_telemetry

            if data is None:
                print("No telemetry received yet.")
            else:
                left, right, bat, temp, angle, gyro = data

                print(f"Left speed : {left:.4f} m/s")
                print(f"Right speed: {right:.4f} m/s")
                print(f"Battery    : {bat:.2f} V")
                print(f"Temp       : {temp:.1f} C")
                print(f"Angle Z    : {angle:.3f} deg")
                print(f"Gyro Z     : {gyro:.3f} deg/s")

            continue

        try:
            parts = command.split(",")

            if len(parts) != 2:
                raise ValueError

            v, w = map(float, parts)

            if not math.isfinite(v) or not math.isfinite(w):
                raise ValueError

            if abs(v) > 0.65 or abs(w) > 2.49:
                print("Limits: |v| <= 0.65, |w| <= 2.49")
                continue

            with lock:
                velocity[:] = [v, w]

            print(f"Command set: v={v:.3f}, w={w:.3f}")

        except ValueError:
            print("Invalid command. Use v,w or STOP.")

except (KeyboardInterrupt, EOFError):
    pass

finally:
    running.clear()
    thread.join(timeout=1)

    if robot.is_open:
        try:
            robot.write(b"STOP\n")
            robot.flush()
        except serial.SerialException:
            pass

        robot.close()

    print("\nDisconnected from Jidenna.")

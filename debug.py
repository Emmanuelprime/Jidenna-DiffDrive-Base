
import serial
import socket
import json
import time
import threading

# Configuration
ARDUINO_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

UDP_PORT = 4210

MAX_LINEAR = 0.25
MAX_ANGULAR = 0.8

PACKET_TIMEOUT = 0.3
COMMAND_TIMEOUT = 0.5
STARTUP_DELAY = 10.0

stop_event = threading.Event()

last_command = (0.0, 0.0)
last_packet_time = 0.0

lock = threading.Lock()


# Connect directly to Arduino
arduino = serial.Serial(
    ARDUINO_PORT,
    BAUDRATE,
    timeout=0.1,
    write_timeout=0.1
)

print("Arduino connected.")
print("Waiting for MPU6050 calibration...")

time.sleep(STARTUP_DELAY)
arduino.reset_input_buffer()


def read_arduino():
    """Print exact incoming Arduino serial lines."""

    while not stop_event.is_set():
        try:
            line = arduino.readline().decode(
                "ascii", errors="replace"
            ).rstrip("\r\n")

            if line:
                print(line, flush=True)

        except (serial.SerialException, OSError) as exc:
            print(f"Serial read error: {exc}")
            stop_event.set()
            break


def send_commands():
    """Continuously publish velocity commands to Arduino."""

    while not stop_event.is_set():
        try:
            with lock:
                v, w = last_command
                age = time.monotonic() - last_packet_time

            # Stop if joystick packets stop arriving
            if age > PACKET_TIMEOUT:
                v, w = 0.0, 0.0

            arduino.write(
                f"{v:.4f},{w:.4f}\n".encode("ascii")
            )

        except (serial.SerialException, OSError) as exc:
            print(f"Serial write error: {exc}")
            stop_event.set()
            break

        stop_event.wait(0.1)


def receive_joystick():
    """Receive ESP32 joystick packets over UDP."""

    global last_command, last_packet_time

    udp = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    )

    udp.bind(("0.0.0.0", UDP_PORT))
    udp.settimeout(0.1)

    print(f"Listening for ESP32 joystick on UDP {UDP_PORT}")

    try:
        while not stop_event.is_set():
            try:
                packet, address = udp.recvfrom(1024)

            except socket.timeout:
                continue

            try:
                axes = json.loads(packet.decode("utf-8"))

                ly = float(axes["ly"])
                rx = float(axes["rx"])

                if not (-1 <= ly <= 1 and -1 <= rx <= 1):
                    continue

                v = ly * MAX_LINEAR

                # Preserve your inverted RX mapping
                w = -rx * MAX_ANGULAR

                with lock:
                    last_command = (v, w)
                    last_packet_time = time.monotonic()

            except (ValueError, KeyError, TypeError):
                continue

    finally:
        udp.close()


try:
    rx_thread = threading.Thread(
        target=read_arduino,
        daemon=True
    )

    tx_thread = threading.Thread(
        target=send_commands,
        daemon=True
    )

    rx_thread.start()
    tx_thread.start()

    print("Jidenna joystick control active.")
    print("Printing raw Arduino CSV telemetry...")
    print("Ctrl+C to stop.\n")

    receive_joystick()

except KeyboardInterrupt:
    print("\nStopping Jidenna...")

finally:
    stop_event.set()

    rx_thread.join(timeout=1.0)
    tx_thread.join(timeout=1.0)

    try:
        arduino.write(b"0.0000,0.0000\n")
        arduino.flush()
    finally:
        arduino.close()

    print("Arduino disconnected.")

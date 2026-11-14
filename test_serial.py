
import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200

arduino = serial.Serial(PORT, BAUD, timeout=1)

print("Connected to Arduino")
print("Waiting 10 seconds for MPU6050 calibration...")

time.sleep(10)

arduino.reset_input_buffer()

print("Reading raw Arduino telemetry...\n")

try:
    while True:
        line = arduino.readline().decode(
            "utf-8", errors="replace"
        ).strip()

        if line:
            print("RAW:", repr(line))
            print("FIELDS:", len(line.split(",")))
        else:
            print("No serial data received")

except KeyboardInterrupt:
    pass

finally:
    arduino.close()

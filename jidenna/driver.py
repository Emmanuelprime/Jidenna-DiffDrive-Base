from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import serial


@dataclass(frozen=True)
class RobotState:
    x: float
    y: float
    theta: float
    v_left: float
    v_right: float
    w_left: float
    w_right: float
    battery_v: float
    temperature_c: float
    feedback_age_ms: int
    arduino_watchdog: bool
    odometry_resets: int
    imu_yaw: float 
    gyro_z: float
    received_at: float  # monotonic clock


class JidennaDriver:

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200,
                 publish_hz: float = 10.0, command_timeout: float = 0.5,
                 telemetry_timeout: float = 0.5, startup_delay: float = 10.0):
        if publish_hz <= 0 or command_timeout <= 0 or telemetry_timeout <= 0:
            raise ValueError("Frequencies and timeouts must be positive")
        self.port = port
        self.baudrate = baudrate
        self.publish_hz = publish_hz
        self.command_timeout = command_timeout
        self.telemetry_timeout = telemetry_timeout
        self.startup_delay = startup_delay
        self._serial: Optional[serial.Serial] = None
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_thread: Optional[threading.Thread] = None
        self._state: Optional[RobotState] = None
        self._velocity = (0.0, 0.0)
        self._last_command_at = 0.0
        self._error: Optional[str] = None

    def connect(self) -> None:
        with self._lock:
            if self._serial is not None:
                raise RuntimeError("Driver already connected")

            self._stop_event.clear()
            self._state = None
            self._error = None
            self._velocity = (0.0, 0.0)
            self._last_command_at = 0.0

            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=0.1
            )

        print("[Jidenna] Arduino connected.")
        print("[Jidenna] Waiting 10 seconds for MPU6050 calibration...")

        time.sleep(self.startup_delay)

        with self._lock:
            self._serial.reset_input_buffer()

        print("[Jidenna] Calibration waiting period completed.")
        print("[Jidenna] Starting communication threads...")

        self._rx_thread = threading.Thread(target=self._read_loop,name="jidenna-rx",daemon=True)

        self._tx_thread = threading.Thread(target=self._write_loop,name="jidenna-tx",daemon=True)

        self._rx_thread.start()
        self._tx_thread.start()

        print("[Jidenna] Driver initialized successfully.")

    def _fault(self, exc: Exception) -> None:
        with self._lock:
            self._error = str(exc)
            self._velocity = (0.0, 0.0)
            self._last_command_at = 0.0
            self._stop_event.set()

    def _read_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                with self._lock:
                    port = self._serial
                if port is None:
                    break
                line = port.readline().decode("ascii", errors="ignore").strip()
                parts = line.split(",")
                if len(parts) != 14:
                    continue  
                numbers = [float(part) for part in parts]
                if not all(math.isfinite(n) for n in numbers):
                    continue
                if any(numbers[i] != int(numbers[i]) for i in (9, 10, 11)):
                    continue
                state = RobotState(
                    *numbers[:9],
                    int(numbers[9]),
                    bool(int(numbers[10])),
                    int(numbers[11]),
                    numbers[12],
                    numbers[13], 
                    time.monotonic()
                )
                with self._lock:
                    self._state = state
            except ValueError:
                continue
            except (serial.SerialException, OSError) as exc:
                self._fault(exc)
                break

    def _write_loop(self) -> None:
        period = 1.0 / self.publish_hz
        while not self._stop_event.is_set():
            try:
                with self._lock:
                    now = time.monotonic()
                    fresh = now - self._last_command_at <= self.command_timeout
                    v, w = self._velocity if fresh else (0.0, 0.0)
                    port = self._serial
                    if port is None:
                        break
                    port.write(f"{v:.4f},{w:.4f}\n".encode("ascii"))
            except (serial.SerialException, OSError) as exc:
                self._fault(exc)
                break
            self._stop_event.wait(period)

    def set_velocity(self, v: float, w: float) -> None:
        v, w = float(v), float(w)
        if not (math.isfinite(v) and math.isfinite(w)):
            raise ValueError("Velocities must be finite")
        if abs(v) > 0.8 or abs(w) > 3.0:
            raise ValueError("Limits: |v| <= 0.8 m/s, |w| <= 3.0 rad/s")
        with self._lock:
            if self._serial is None or self._stop_event.is_set():
                raise RuntimeError("Driver not connected")
            self._velocity = (v, w)
            self._last_command_at = time.monotonic()

    def stop(self) -> None:
        with self._lock:
            self._velocity = (0.0, 0.0)
            self._last_command_at = 0.0
            if self._serial is not None and not self._stop_event.is_set():
                self._serial.write(b"0.0000,0.0000\n")

    def reset_odometry(self) -> None:
        with self._lock:
            if self._serial is None or self._stop_event.is_set():
                raise RuntimeError("Driver not connected")
            self._serial.write(b"r\n")

    def get_state(self) -> Optional[RobotState]:
        with self._lock:
            return self._state

    def get_odometry(self) -> Optional[tuple[float, float, float]]:
        state = self.get_state()
        return (state.x, state.y, state.theta) if state else None

    def get_odometry_data(self) -> Optional[dict]:
        """Return named pose coordinates in meters and radians."""
        state = self.get_state()
        return {"x": state.x, "y": state.y, "theta": state.theta} if state else None

    def get_imu_gyro(self) -> Optional[dict]:
        """
        Return MPU6050 orientation and angular velocity.

        yaw: radians
        gyro_z: radians/second
        """
        state = self.get_state()

        if state is None:
            return None

        return {
            "yaw": state.imu_yaw,
            "gyro_z": state.gyro_z
        }

    def get_battery_info(self) -> Optional[dict]:
        """Return measured hoverboard battery voltage in volts."""
        state = self.get_state()
        return {"voltage": state.battery_v} if state else None

    def get_wheel_velocities(self) -> Optional[dict]:
        """Return wheel linear (m/s) and angular (rad/s) velocities."""
        state = self.get_state()
        return ({"left_mps": state.v_left, "right_mps": state.v_right,
                 "left_rad_s": state.w_left, "right_rad_s": state.w_right}
                if state else None)

    def get_motor_temperature(self) -> Optional[float]:
        """Return hoverboard motherboard temperature (degrees Celsius)."""
        state = self.get_state()
        return state.temperature_c if state else None

    def get_imu_yaw(self) -> Optional[float]:
        state = self.get_state()
        return state.imu_yaw if state else None

    def get_status(self) -> dict:
        with self._lock:
            state = self._state
            age = time.monotonic() - state.received_at if state else None
            connected = (self._serial is not None and self._serial.is_open
                         and not self._stop_event.is_set())
            return {
                "connected": connected,
                "telemetry_fresh": bool(connected and age is not None and age <= self.telemetry_timeout),
                "telemetry_age_s": age,
                "motor_feedback_fresh": bool(state and state.feedback_age_ms <= 250),
                "arduino_watchdog": state.arduino_watchdog if state else None,
                "error": self._error,
            }

    def disconnect(self) -> None:
        try:
            self.stop()
        except (serial.SerialException, OSError):
            pass
        self._stop_event.set()
        for thread in (self._rx_thread, self._tx_thread):
            if thread and thread.is_alive() and thread is not threading.current_thread():
                thread.join(timeout=1.0)
        with self._lock:
            if self._serial is not None:
                self._serial.close()
                self._serial = None

    def __enter__(self) -> "JidennaDriver":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

"""Thread-safe serial driver for the Jidenna robot."""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import serial


@dataclass(frozen=True)
class RobotState:
    left_speed: float
    right_speed: float
    battery_v: float
    temperature_c: float
    imu_yaw_deg: float
    gyro_z_deg_s: float
    timestamp: float  # monotonic seconds

    @property
    def imu_yaw_rad(self) -> float:
        return math.radians(self.imu_yaw_deg)

    @property
    def gyro_z_rad_s(self) -> float:
        return math.radians(self.gyro_z_deg_s)


class JidennaDriver:
    MAX_LINEAR = 2.1
    MAX_ANGULAR = 2.49

    def __init__(self, port: str = "COM9", baudrate: int = 115200,
                 command_rate: float = 10.0, command_timeout: float = 0.3,
                 startup_timeout: float = 15.0, autoconnect: bool = True,
                 startup_telemetry_frames: int = 3):
        if command_rate <= 0 or command_timeout <= 0 or startup_timeout <= 0:
            raise ValueError("Rates and timeouts must be positive")
        if startup_telemetry_frames < 1:
            raise ValueError("startup_telemetry_frames must be >= 1")
        self.port = port
        self.baudrate = baudrate
        self.command_rate = command_rate
        self.command_timeout = command_timeout
        self.startup_timeout = startup_timeout
        self.startup_telemetry_frames = startup_telemetry_frames
        self._serial: Optional[serial.Serial] = None
        self._lock = threading.RLock()
        self._write_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._tx_thread: Optional[threading.Thread] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._velocity = (0.0, 0.0)
        self._command_at = 0.0
        self._state: Optional[RobotState] = None
        self._error: Optional[str] = None
        # Startup coordination
        self._ready_event = threading.Event()
        self._startup_frames_seen = 0
        if autoconnect:
            self.connect()

    @property
    def is_connected(self) -> bool:
        return (self._serial is not None and self._serial.is_open
                and not self._stop_event.is_set())

    def connect(self) -> None:
        if self.is_connected:
            return
        self.close()
        self._stop_event.clear()
        self._ready_event.clear()
        with self._lock:
            self._velocity = (0.0, 0.0)
            self._command_at = 0.0
            self._state = None
            self._error = None
            self._startup_frames_seen = 0
        try:
            self._serial = serial.Serial(self.port, self.baudrate,
                                         timeout=0.1, write_timeout=0.2)
            # Start RX first so it can collect the startup telemetry frames.
            self._rx_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._rx_thread.start()
            self._wait_ready()
        except Exception:
            self.close()
            raise
        self._tx_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._tx_thread.start()

    def _wait_ready(self) -> None:
        """Block until enough valid telemetry frames have been received.

        Readiness is defined purely as `startup_telemetry_frames` valid
        telemetry lines arriving from the firmware. The RX thread does all
        the reading and sets `_ready_event` once the threshold is met.
        """
        deadline = time.monotonic() + self.startup_timeout
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                raise RuntimeError("Connection failed during startup")
            if self._ready_event.wait(timeout=0.05):
                print("[Jidenna] Ready (telemetry acquired)")
                return
        raise TimeoutError(
            f"Jidenna did not produce {self.startup_telemetry_frames} "
            f"valid telemetry frames within {self.startup_timeout:.1f}s"
        )

    def _mark_frame(self, state: RobotState) -> None:
        """Called by the RX thread for each valid telemetry line.

        Counts startup frames and, once enough have arrived, publishes the
        first real state and signals readiness.
        """
        with self._lock:
            self._state = state
            if not self._ready_event.is_set():
                self._startup_frames_seen += 1
                if self._startup_frames_seen >= self.startup_telemetry_frames:
                    self._ready_event.set()

    def set_velocity(self, v: float, w: float) -> None:
        v, w = float(v), float(w)
        if not (math.isfinite(v) and math.isfinite(w)):
            raise ValueError("Velocities must be finite")
        if abs(v) > self.MAX_LINEAR or abs(w) > self.MAX_ANGULAR:
            raise ValueError("Velocity outside Jidenna limits")
        if not self.is_connected:
            raise RuntimeError("Jidenna is not connected")
        with self._lock:
            self._velocity = (v, w)
            self._command_at = time.monotonic()

    def stop(self) -> None:
        with self._lock:
            self._velocity = (0.0, 0.0)
            self._command_at = 0.0
        if self.is_connected:
            try:
                with self._write_lock:
                    self._serial.write(b"STOP\n")
            except (serial.SerialException, OSError) as exc:
                self._fail(exc)

    def _fail(self, exc: Exception) -> None:
        with self._lock:
            self._error = str(exc)
        self._stop_event.set()
        # Ensure any waiters unblock.
        self._ready_event.set()

    def _send_loop(self) -> None:
        period = 1.0 / self.command_rate
        while not self._stop_event.is_set():
            started = time.monotonic()
            with self._lock:
                v, w = self._velocity
                if self._command_at == 0 or started - self._command_at >= self.command_timeout:
                    v, w = 0.0, 0.0
                    self._velocity = (0.0, 0.0)
                    self._command_at = 0.0
            try:
                with self._write_lock:
                    self._serial.write(f"{v:.4f},{w:.4f}\n".encode())
            except (serial.SerialException, OSError) as exc:
                self._fail(exc)
                break
            self._stop_event.wait(max(0.0, period - (time.monotonic() - started)))

    def _read_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                line = self._serial.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                fields = line.split(",")
                if len(fields) != 6:
                    continue
                values = tuple(float(field) for field in fields)
                if not all(math.isfinite(value) for value in values):
                    continue
                state = RobotState(*values, timestamp=time.monotonic())
                self._mark_frame(state)
            except ValueError:
                continue
            except (serial.SerialException, OSError) as exc:
                self._fail(exc)
                break

    def get_state(self) -> Optional[RobotState]:
        with self._lock:
            return self._state

    def get_wheel_speeds(self):
        state = self.get_state()
        return None if state is None else (state.left_speed, state.right_speed)

    def get_imu(self):
        state = self.get_state()
        return None if state is None else (state.imu_yaw_rad, state.gyro_z_rad_s)

    def get_battery(self):
        state = self.get_state()
        return None if state is None else state.battery_v

    def get_status(self) -> dict:
        state = self.get_state()
        with self._lock:
            velocity = self._velocity
            error = self._error
        return {"connected": self.is_connected,
                "command": velocity,
                "telemetry_age_s": None if state is None else time.monotonic() - state.timestamp,
                "error": error}

    def close(self) -> None:
        self._stop_event.set()
        self._ready_event.set()  # unblock any _wait_ready caller
        for thread in (self._tx_thread, self._rx_thread):
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=0.5)
        self._tx_thread = None
        self._rx_thread = None
        if self._serial is not None:
            if self._serial.is_open:
                try:
                    with self._write_lock:
                        self._serial.write(b"STOP\n")
                        self._serial.flush()
                except (serial.SerialException, OSError):
                    pass
                self._serial.close()
            self._serial = None

    def __enter__(self):
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
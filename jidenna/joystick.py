
"""UDP joystick controller for the Jidenna robot."""

import json
import math
import socket
import threading
import time

from .driver import JidennaDriver


class UDPJoystickController:
    def __init__(
        self,
        robot: JidennaDriver,
        host: str = "0.0.0.0",
        port: int = 4210,
        max_linear: float = 2.1,
        max_angular: float = 2.49,
        invert_linear: bool = False,
        invert_angular: bool = False,
        timeout: float = 0.3,
        command_rate: float = 10.0,
    ):
        if timeout <= 0 or command_rate <= 0:
            raise ValueError("Timeout and command rate must be positive")

        if not (0 < max_linear <= robot.MAX_LINEAR):
            raise ValueError("Invalid maximum linear velocity")

        if not (0 < max_angular <= robot.MAX_ANGULAR):
            raise ValueError("Invalid maximum angular velocity")

        self.robot = robot
        self.host = host
        self.port = port

        self.max_linear = max_linear
        self.max_angular = max_angular

        self.invert_linear = invert_linear
        self.invert_angular = invert_angular

        self.timeout = timeout
        self.command_rate = command_rate

        self._socket = None
        self._thread = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        self._velocity = (0.0, 0.0)
        self._last_packet = None
        self._active = False
        self._error = None

    @staticmethod
    def _clamp(value):
        return max(-1.0, min(1.0, value))

    def _parse_packet(self, data):
        packet = json.loads(data.decode("utf-8"))

        ly = float(packet["ly"])
        rx = float(packet["rx"])

        if not (math.isfinite(ly) and math.isfinite(rx)):
            raise ValueError("Non-finite joystick value")

        ly = self._clamp(ly)
        rx = self._clamp(rx)

        if self.invert_linear:
            ly = -ly

        if self.invert_angular:
            rx = -rx

        v = ly * self.max_linear
        w = rx * self.max_angular

        return v, w

    @property
    def is_running(self):
        return (
            self._thread is not None
            and self._thread.is_alive()
        )

    def start(self):
        if self.is_running:
            return

        if not self.robot.is_connected:
            raise RuntimeError("Jidenna driver is not connected")

        self._stop_event.clear()

        with self._lock:
            self._velocity = (0.0, 0.0)
            self._last_packet = None
            self._active = False
            self._error = None

        self._socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        try:
            self._socket.bind((self.host, self.port))
            self._socket.settimeout(
                min(0.05, 1.0 / self.command_rate)
            )
        except Exception:
            self._socket.close()
            self._socket = None
            raise

        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="JidennaJoystick"
        )
        self._thread.start()

        print(f"[Joystick] Listening on UDP {self.port}")

    def _run(self):
        period = 1.0 / self.command_rate
        next_command = time.monotonic()

        try:
            while not self._stop_event.is_set():

                try:
                    data, address = self._socket.recvfrom(1024)

                    v, w = self._parse_packet(data)

                    with self._lock:
                        self._velocity = (v, w)
                        self._last_packet = time.monotonic()
                        self._active = True

                except socket.timeout:
                    pass

                except (
                    ValueError,
                    KeyError,
                    TypeError,
                    UnicodeDecodeError,
                ):
                    pass

                now = time.monotonic()

                with self._lock:
                    last_packet = self._last_packet

                    timed_out = (
                        self._active
                        and last_packet is not None
                        and now - last_packet >= self.timeout
                    )

                    if timed_out:
                        self._velocity = (0.0, 0.0)
                        self._active = False

                    active = self._active
                    v, w = self._velocity

                if timed_out:
                    self.robot.stop()
                    print("[Joystick] Signal lost - STOP")

                if now >= next_command:
                    if active:
                        self.robot.set_velocity(v, w)

                    next_command = now + period

                if not self.robot.is_connected:
                    raise RuntimeError("Jidenna disconnected")

        except Exception as exc:
            with self._lock:
                self._error = str(exc)

            print(f"[Joystick] Error: {exc}")

        finally:
            self._stop_event.set()
            self.robot.stop()

    def stop(self):
        self._stop_event.set()

        if self._thread is not None:
            if self._thread is not threading.current_thread():
                self._thread.join(timeout=1.0)

        self.robot.stop()

        if self._socket is not None:
            self._socket.close()
            self._socket = None

        self._thread = None

        with self._lock:
            self._velocity = (0.0, 0.0)
            self._active = False

    def get_velocity(self):
        with self._lock:
            return self._velocity

    def get_status(self):
        with self._lock:
            last_packet = self._last_packet

            return {
                "running": self.is_running,
                "active": self._active,
                "velocity": self._velocity,
                "packet_age_s": (
                    None if last_packet is None
                    else time.monotonic() - last_packet
                ),
                "error": self._error,
            }

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()

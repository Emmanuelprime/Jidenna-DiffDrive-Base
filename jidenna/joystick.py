
"""Thread-safe UDP joystick controller for Jidenna."""

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
        invert_angular: bool = True,
        timeout: float = 0.5,
        command_rate: float = 20.0,
    ):
        if timeout <= 0 or command_rate <= 0:
            raise ValueError(
                "Timeout and command rate must be positive"
            )

        if not (0 < max_linear <= robot.MAX_LINEAR):
            raise ValueError("Invalid max_linear")

        if not (0 < max_angular <= robot.MAX_ANGULAR):
            raise ValueError("Invalid max_angular")

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
        self._rx_thread = None
        self._control_thread = None

        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        self._velocity = (0.0, 0.0)
        self._last_packet = None
        self._active = False
        self._error = None
        self._packet_count = 0

    @staticmethod
    def _clamp(value):
        return max(-1.0, min(1.0, value))

    def _parse_packet(self, data):
        packet = json.loads(data.decode("utf-8"))

        ly = float(packet["ly"])
        rx = float(packet["rx"])

        if not (
            math.isfinite(ly)
            and math.isfinite(rx)
        ):
            raise ValueError("Invalid joystick values")

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
            self._control_thread is not None
            and self._control_thread.is_alive()
            and not self._stop_event.is_set()
        )

    def start(self):
        if self.is_running:
            return

        if not self.robot.is_connected:
            raise RuntimeError(
                "Jidenna driver is not connected"
            )

        self._stop_event.clear()

        with self._lock:
            self._velocity = (0.0, 0.0)
            self._last_packet = None
            self._active = False
            self._error = None
            self._packet_count = 0

        self._socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        try:
            self._socket.bind(
                (self.host, self.port)
            )
            self._socket.settimeout(0.1)
        except Exception:
            self._socket.close()
            self._socket = None
            raise

        self._rx_thread = threading.Thread(
            target=self._receive_loop,
            daemon=True,
            name="JidennaJoystickRX"
        )

        self._control_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name="JidennaJoystickControl"
        )

        self._rx_thread.start()
        self._control_thread.start()

        print(
            f"[Joystick] Listening on "
            f"{self.host}:{self.port}"
        )

    def _receive_loop(self):
        """Continuously receive joystick packets."""

        try:
            while not self._stop_event.is_set():
                try:
                    data, address = (
                        self._socket.recvfrom(1024)
                    )

                    v, w = self._parse_packet(data)

                    now = time.monotonic()

                    with self._lock:
                        self._velocity = (v, w)
                        self._last_packet = now
                        self._active = True
                        self._packet_count += 1

                except socket.timeout:
                    continue

                except (
                    ValueError,
                    KeyError,
                    TypeError,
                    UnicodeDecodeError,
                ):
                    continue

        except (OSError, Exception) as exc:
            if not self._stop_event.is_set():
                with self._lock:
                    self._error = str(exc)

                print(
                    f"[Joystick RX] Error: {exc}"
                )

                self._stop_event.set()

    def _control_loop(self):
        """Send velocity commands at a fixed rate."""

        period = 1.0 / self.command_rate

        try:
            while not self._stop_event.is_set():

                start = time.monotonic()

                with self._lock:
                    last_packet = self._last_packet
                    v, w = self._velocity
                    was_active = self._active

                active = (
                    last_packet is not None
                    and start - last_packet < self.timeout
                )

                if not active:
                    v = 0.0
                    w = 0.0

                    with self._lock:
                        self._velocity = (0.0, 0.0)
                        self._active = False

                    if was_active:
                        self.robot.stop()
                        print(
                            "[Joystick] Signal lost - STOP"
                        )

                else:
                    with self._lock:
                        self._active = True

                    self.robot.set_velocity(v, w)

                if not self.robot.is_connected:
                    raise RuntimeError(
                        "Jidenna disconnected"
                    )

                elapsed = time.monotonic() - start

                self._stop_event.wait(
                    max(0.0, period - elapsed)
                )

        except Exception as exc:
            with self._lock:
                self._error = str(exc)

            print(
                f"[Joystick Control] Error: {exc}"
            )

        finally:
            self._stop_event.set()
            self.robot.stop()

    def stop(self):
        self._stop_event.set()

        self.robot.stop()

        if self._socket is not None:
            self._socket.close()

        for thread in (
            self._rx_thread,
            self._control_thread,
        ):
            if (
                thread is not None
                and thread is not threading.current_thread()
            ):
                thread.join(timeout=1.0)

        self._socket = None
        self._rx_thread = None
        self._control_thread = None

        with self._lock:
            self._velocity = (0.0, 0.0)
            self._active = False

        print("[Joystick] Stopped")

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
                "packet_count": self._packet_count,
                "packet_age_s": (
                    None
                    if last_packet is None
                    else time.monotonic() - last_packet
                ),
                "error": self._error,
            }

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()

import math
import threading
import time
from dataclasses import dataclass

from ..driver import JidennaDriver


@dataclass(frozen=True)
class OdometryState:
    x: float
    y: float
    theta: float
    v: float
    w: float
    v_left: float
    v_right: float
    timestamp: float


class Odometry:

    def __init__(self, robot: JidennaDriver, wheel_separation: float = 0.521, update_rate: float = 20.0, max_sample_gap: float = 1.0):
        if wheel_separation <= 0:
            raise ValueError("Wheel separation must be positive")
        if update_rate <= 0:
            raise ValueError("Update rate must be positive")
        if max_sample_gap <= 0:
            raise ValueError("Max sample gap must be positive")

        self.robot = robot
        self.wheel_separation = wheel_separation
        self.update_rate = update_rate
        self.max_sample_gap = max_sample_gap

        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread = None

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._v = 0.0
        self._w = 0.0
        self._v_left = 0.0
        self._v_right = 0.0
        self._last_sample_time = None
        self._last_state = None

    @staticmethod
    def _wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def update(self):
        state = self.robot.get_state()
        if state is None:
            return False

        sample_time = state.timestamp
        left = float(state.left_speed)
        right = float(state.right_speed)

        if not all(math.isfinite(value) for value in (sample_time, left, right)):
            return False

        with self._lock:
            if self._last_sample_time is not None and sample_time <= self._last_sample_time:
                return False

            if self._last_sample_time is None:
                self._last_sample_time = sample_time
                self._v_left = left
                self._v_right = right
                self._v = (left + right) / 2.0
                self._w = (right - left) / self.wheel_separation
                self._last_state = (left, right)
                return True

            dt = sample_time - self._last_sample_time
            previous_left = self._last_state[0] if self._last_state is not None else left
            previous_right = self._last_state[1] if self._last_state is not None else right

            self._last_sample_time = sample_time
            self._v_left = left
            self._v_right = right
            self._v = (left + right) / 2.0
            self._w = (right - left) / self.wheel_separation

            if dt <= 0 or dt > self.max_sample_gap:
                self._last_state = (left, right)
                return True

            avg_left = (previous_left + left) / 2.0
            avg_right = (previous_right + right) / 2.0

            v = (avg_left + avg_right) / 2.0
            w = (avg_right - avg_left) / self.wheel_separation

            dtheta = w * dt
            theta_mid = self._theta + dtheta / 2.0

            self._x += v * math.cos(theta_mid) * dt
            self._y += v * math.sin(theta_mid) * dt
            self._theta = self._wrap_angle(self._theta + dtheta)
            self._last_state = (left, right)

            return True

    def _run(self):
        period = 1.0 / self.update_rate
        while not self._stop_event.is_set():
            self.update()
            self._stop_event.wait(period)

    def start(self):
        if self.is_running:
            return
        with self._lock:
            self._last_sample_time = None
            self._last_state = None
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="JidennaOdometry")
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        self._thread = None

    @property
    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        with self._lock:
            self._x = float(x)
            self._y = float(y)
            self._theta = self._wrap_angle(theta)
            self._v = 0.0
            self._w = 0.0
            self._v_left = 0.0
            self._v_right = 0.0
            self._last_sample_time = None
            self._last_state = None

    def get_pose(self):
        with self._lock:
            return (self._x, self._y, self._theta)

    def get_velocity(self):
        with self._lock:
            return (self._v, self._w)

    def get_state(self):
        with self._lock:
            return OdometryState(
                x=self._x,
                y=self._y,
                theta=self._theta,
                v=self._v,
                w=self._w,
                v_left=self._v_left,
                v_right=self._v_right,
                timestamp=self._last_sample_time if self._last_sample_time is not None else time.monotonic(),
            )

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
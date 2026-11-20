
import math
import threading
import time
from dataclasses import dataclass


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

    MODES = ("wheel", "imu", "complementary")

    def __init__(
        self,
        robot,
        wheel_separation=0.521,
        update_rate=20.0,
        mode="wheel",
        alpha=0.5,
        max_sample_gap=1.0,
    ):
        if wheel_separation <= 0:
            raise ValueError(
                "wheel_separation must be positive"
            )

        if update_rate <= 0:
            raise ValueError(
                "update_rate must be positive"
            )

        if max_sample_gap <= 0:
            raise ValueError(
                "max_sample_gap must be positive"
            )

        if mode not in self.MODES:
            raise ValueError(
                f"mode must be one of {self.MODES}"
            )

        if not 0.0 <= alpha <= 1.0:
            raise ValueError(
                "alpha must be between 0 and 1"
            )

        self.robot = robot
        self.wheel_separation = wheel_separation
        self.update_rate = update_rate
        self.mode = mode
        self.alpha = alpha
        self.max_sample_gap = max_sample_gap

        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread = None

        self._poses = {
            name: [0.0, 0.0, 0.0]
            for name in self.MODES
        }

        self._v = 0.0
        self._v_left = 0.0
        self._v_right = 0.0

        self._rates = {
            name: 0.0
            for name in self.MODES
        }

        self._last_sample_time = None
        self._previous_measurement = None

    @staticmethod
    def _wrap_angle(angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    @staticmethod
    def _integrate(pose, v, w, dt):
        """
        Exact integration assuming constant
        linear and angular velocity over dt.
        """

        x, y, theta = pose

        dtheta = w * dt

        if abs(w) < 1e-8:
            dx_body = v * dt
            dy_body = 0.0
        else:
            dx_body = (
                v / w
            ) * math.sin(dtheta)

            dy_body = (
                v / w
            ) * (1.0 - math.cos(dtheta))

        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        x += (
            cos_t * dx_body
            - sin_t * dy_body
        )

        y += (
            sin_t * dx_body
            + cos_t * dy_body
        )

        theta = Odometry._wrap_angle(
            theta + dtheta
        )

        return [x, y, theta]

    def update(self):

        state = self.robot.get_state()

        if state is None:
            return False

        # JidennaDriver telemetry fields
        left = float(state.v_left)
        right = float(state.v_right)
        gyro = float(state.gyro_z)

        # Driver timestamp should change only
        # when a fresh telemetry packet arrives.
        sample_time = float(state.received_at)

        values = (
            left,
            right,
            gyro,
            sample_time,
        )

        if not all(
            math.isfinite(value)
            for value in values
        ):
            return False

        v = (left + right) / 2.0

        w_wheel = (
            right - left
        ) / self.wheel_separation

        # MPU6050 gyro Z is in degrees/second
        w_imu = math.radians(gyro)

        # Angular-rate weighted fusion
        w_complementary = (
            self.alpha * w_imu
            + (1.0 - self.alpha) * w_wheel
        )

        rates = {
            "wheel": w_wheel,
            "imu": w_imu,
            "complementary": w_complementary,
        }

        with self._lock:

            if (
                self._last_sample_time is not None
                and sample_time <= self._last_sample_time
            ):
                return False

            measurement = (
                v,
                w_wheel,
                w_imu,
                w_complementary,
            )

            if self._last_sample_time is None:

                self._last_sample_time = sample_time
                self._previous_measurement = measurement

                self._v = v
                self._v_left = left
                self._v_right = right
                self._rates = rates

                return True

            dt = (
                sample_time
                - self._last_sample_time
            )

            previous = self._previous_measurement

            self._last_sample_time = sample_time
            self._previous_measurement = measurement

            self._v = v
            self._v_left = left
            self._v_right = right
            self._rates = rates

            if dt <= 0 or dt > self.max_sample_gap:
                return True

            previous_v = previous[0]

            # Trapezoidal approximation of
            # velocities between telemetry samples.
            avg_v = (
                previous_v + v
            ) / 2.0

            previous_rates = {
                "wheel": previous[1],
                "imu": previous[2],
                "complementary": previous[3],
            }

            for name in self.MODES:

                avg_w = (
                    previous_rates[name]
                    + rates[name]
                ) / 2.0

                self._poses[name] = self._integrate(
                    self._poses[name],
                    avg_v,
                    avg_w,
                    dt,
                )

            return True

    def _run(self):

        period = 1.0 / self.update_rate

        while not self._stop_event.is_set():

            try:
                self.update()
            except Exception as exc:
                print(
                    f"[Odometry] Update error: {exc}"
                )
                self._stop_event.set()
                break

            self._stop_event.wait(period)

    def start(self):

        if self.is_running:
            return

        with self._lock:
            self._last_sample_time = None
            self._previous_measurement = None

        self._stop_event.clear()

        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="JidennaOdometry",
        )

        self._thread.start()

    def stop(self):

        self._stop_event.set()

        thread = self._thread

        if (
            thread is not None
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=2.0)

        self._thread = None

    @property
    def is_running(self):

        return (
            self._thread is not None
            and self._thread.is_alive()
        )

    def set_mode(self, mode):

        if mode not in self.MODES:
            raise ValueError(
                f"Invalid mode: {mode}"
            )

        with self._lock:
            self.mode = mode

    def reset(
        self,
        x=0.0,
        y=0.0,
        theta=0.0,
    ):

        with self._lock:

            theta = self._wrap_angle(theta)

            for name in self.MODES:
                self._poses[name] = [
                    float(x),
                    float(y),
                    theta,
                ]

                self._rates[name] = 0.0

            self._v = 0.0
            self._v_left = 0.0
            self._v_right = 0.0

            self._last_sample_time = None
            self._previous_measurement = None

    def get_pose(self, mode=None):

        with self._lock:

            name = mode or self.mode

            if name not in self.MODES:
                raise ValueError(
                    f"Invalid mode: {name}"
                )

            return tuple(self._poses[name])

    def get_velocity(self, mode=None):

        with self._lock:

            name = mode or self.mode

            if name not in self.MODES:
                raise ValueError(
                    f"Invalid mode: {name}"
                )

            return (
                self._v,
                self._rates[name],
            )

    def get_state(self, mode=None):

        with self._lock:

            name = mode or self.mode

            if name not in self.MODES:
                raise ValueError(
                    f"Invalid mode: {name}"
                )

            x, y, theta = self._poses[name]

            timestamp = (
                self._last_sample_time
                if self._last_sample_time is not None
                else time.monotonic()
            )

            return OdometryState(
                x=x,
                y=y,
                theta=theta,
                v=self._v,
                w=self._rates[name],
                v_left=self._v_left,
                v_right=self._v_right,
                timestamp=timestamp,
            )

    def get_all_poses(self):

        with self._lock:
            return {
                name: tuple(pose)
                for name, pose in self._poses.items()
            }

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()

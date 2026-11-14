"""Jidenna relative localization v1: wheel/gyro EKF, no global position fix."""
from __future__ import annotations
import math
import time
from threading import RLock
import numpy as np


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Localization:
    def __init__(self, robot, telemetry_timeout=0.5, max_dt=0.5,
                 gyro_noise=0.06, speed_noise=0.08,
                 odom_position_noise=0.15, odom_heading_noise=0.12):
        self.robot = robot
        self.telemetry_timeout = float(telemetry_timeout)
        self.max_dt = float(max_dt)
        self.gyro_noise = float(gyro_noise)
        self.speed_noise = float(speed_noise)
        self.R = np.diag([odom_position_noise**2]*2 + [odom_heading_noise**2])
        self._lock = RLock()
        self._x = np.zeros(3)
        self._P = np.diag([0.1**2, 0.1**2, 0.1**2])
        self._last_stamp = None
        self._origin = None
        self._last_reset_counter = None
        self._last_velocity = (0.0, 0.0)
        self._status = 'waiting_for_telemetry'

    def update(self):
        s = self.robot.get_state()
        if s is None or time.monotonic() - s.received_at > self.telemetry_timeout:
            with self._lock:
                self._status = 'stale_telemetry'
            return False
        with self._lock:
            if self._last_stamp is not None and s.received_at <= self._last_stamp:
                return False
            if self._last_reset_counter is not None and s.odometry_resets != self._last_reset_counter:
                self._status = 'odometry_reset_detected'
                self._last_stamp = None
                self._origin = None
                self._last_reset_counter = s.odometry_resets
                return False
            self._last_reset_counter = s.odometry_resets
            if s.feedback_age_ms > 250:
                self._status = 'stale_motor_feedback'
                return False
            if self._origin is None:
                self._origin = (s.x, s.y, s.theta)
                self._last_stamp = s.received_at
                self._status = 'initialized'
                return True
            dt = s.received_at - self._last_stamp
            if dt <= 0 or dt > self.max_dt:
                self._last_stamp = s.received_at
                self._status = 'timing_gap'
                return False
            self._last_stamp = s.received_at
            v = (s.v_left + s.v_right) / 2
            w = s.gyro_z
            if not all(map(math.isfinite, (v,w,s.x,s.y,s.theta))):
                self._status = 'invalid_measurement'
                return False
            theta_mid = self._x[2] + w*dt/2
            dx,dy = v*dt*math.cos(theta_mid),v*dt*math.sin(theta_mid)
            F = np.eye(3)
            F[0,2] = -v*dt*math.sin(theta_mid)
            F[1,2] = v*dt*math.cos(theta_mid)
            G = np.array([[dt*math.cos(theta_mid), -v*dt*dt*math.sin(theta_mid)/2],
                          [dt*math.sin(theta_mid), v*dt*dt*math.cos(theta_mid)/2],
                          [0,dt]])
            Q = G @ np.diag([self.speed_noise**2,self.gyro_noise**2]) @ G.T
            self._x += [dx,dy,w*dt]
            self._x[2] = wrap(self._x[2])
            self._P = F@self._P@F.T + Q + np.diag([1e-6,1e-6,1e-6])
            ox,oy,ot = self._origin
            # Convert odometry displacement to the origin's local heading frame.
            ddx,ddy = s.x-ox,s.y-oy
            c,si = math.cos(ot),math.sin(ot)
            z = np.array([c*ddx+si*ddy, -si*ddx+c*ddy, wrap(s.theta-ot)])
            residual = z-self._x
            residual[2] = wrap(residual[2])
            S = self._P + self.R
            K = np.linalg.solve(S.T,self._P.T).T
            self._x += K@residual
            self._x[2] = wrap(self._x[2])
            I = np.eye(3)
            self._P = (I-K)@self._P@(I-K).T + K@self.R@K.T
            self._last_velocity = (v,w)
            self._status = 'ok'
            return True

    def get_pose(self):
        with self._lock:
            return {'x':float(self._x[0]), 'y':float(self._x[1]), 'theta':float(self._x[2])} if self._origin is not None else None

    def get_heading(self):
        pose = self.get_pose()
        return pose['theta'] if pose else None

    def get_velocity(self):
        with self._lock:
            return {'linear':self._last_velocity[0], 'angular':self._last_velocity[1]}

    def get_status(self):
        with self._lock:
            return {'status':self._status, 'initialized':self._origin is not None,
                    'covariance':self._P.tolist(), 'last_update_at':self._last_stamp}

    def reset_pose(self):
        """Rebase on next incoming telemetry; does not reset Arduino odometry."""
        with self._lock:
            self._x[:] = 0
            self._P = np.diag([0.1**2]*3)
            self._last_stamp = None
            self._origin = None
            self._last_reset_counter = None
            self._last_velocity = (0.0,0.0)
            self._status = 'waiting_for_telemetry'

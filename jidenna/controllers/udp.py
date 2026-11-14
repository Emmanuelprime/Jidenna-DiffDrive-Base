"""UDP joystick bridge for JidennaDriver. Only one process should own the serial port."""
import json
import math
import socket
import time
from typing import Optional


class UDPJoystickController:
    def __init__(self, robot, host='0.0.0.0', port=4210,
                 max_linear=0.8, max_angular=0.8,
                 packet_timeout=0.3, allowed_ip: Optional[str] = None,
                 invert_linear=False, invert_angular=False):
        if not (0 < max_linear <= 0.8 and 0 < max_angular <= 3.0):
            raise ValueError('Speed limits must fit Arduino firmware limits')
        if packet_timeout <= 0:
            raise ValueError('packet_timeout must be positive')
        self.robot = robot
        self.host, self.port = host, port
        self.max_linear, self.max_angular = max_linear, max_angular
        self.packet_timeout = packet_timeout
        self.allowed_ip = allowed_ip
        self.invert_linear, self.invert_angular = invert_linear, invert_angular
        self._running = False
        self._last_packet = None
        self._last_sender = None

    @staticmethod
    def _parse(packet):
        data = json.loads(packet.decode('utf-8'))
        if not isinstance(data, dict) or not all(k in data for k in ('lx', 'ly', 'rx', 'ry', 't')):
            raise ValueError('Missing joystick fields')
        axes = {}
        for key in ('lx', 'ly', 'rx', 'ry'):
            value = data[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError('Invalid axis type')
            value = float(value)
            if not math.isfinite(value) or not -1.0 <= value <= 1.0:
                raise ValueError('Invalid axis value')
            axes[key] = value
        return axes

    def run(self):
        """Blocking receiver; call from the main thread. Ctrl+C stops it."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.settimeout(0.05)
        self._running = True
        self._last_packet = None
        print(f'[UDP] Listening on {self.host}:{self.port}')
        try:
            while self._running:
                try:
                    payload, address = sock.recvfrom(1024)
                    if self.allowed_ip and address[0] != self.allowed_ip:
                        continue
                    axes = self._parse(payload)
                    # Sign conventions: positive ly = forward, positive rx = left.
                    v = axes['ly'] * self.max_linear * (-1 if self.invert_linear else 1)
                    w = axes['rx'] * self.max_angular * (1 if self.invert_angular else -1)
                    self.robot.set_velocity(v, w)
                    self._last_packet = time.monotonic()
                    if self._last_sender != address[0]:
                        print(f'[UDP] Receiving joystick from {address[0]}')
                        self._last_sender = address[0]
                except socket.timeout:
                    pass
                except (ValueError, UnicodeError, json.JSONDecodeError):
                    pass

                if self._last_packet is not None and time.monotonic() - self._last_packet > self.packet_timeout:
                    self.robot.stop()
                    self._last_packet = None
                    print('[SAFETY] Controller timeout: stopped')
        finally:
            self._running = False
            try:
                self.robot.stop()
            finally:
                sock.close()

    def shutdown(self):
        self._running = False

"""VTD 모의 서버 — 같은 TCP 9910 프로토콜로 자전거 모델 차량을 굴린다.

폐루프 검증용 '테스트 시나리오': 지정 시작 pose에서 ego 1대, 교통 없음.
time_scale로 가속 시뮬 가능 (제어기는 수신 패킷에 맞춰 돌므로 그대로 동작).
"""
import socket
import struct
import threading
import time

import numpy as np

from hlvtd_io import VehicleState, pack_data, CTRL_SIZE, unpack_ctrl

WHEELBASE = 2.944
MAX_STEER = 0.48
ACCEL_MIN, ACCEL_MAX = -6.0, 3.0
DT = 0.05  # 시뮬 시간 스텝 (20Hz)


class MockVTD:
    def __init__(self, x0, y0, hdg0, port=9910, time_scale=1.0):
        self.x, self.y, self.h = float(x0), float(y0), float(hdg0)
        self.v = 0.0
        self.steer_cmd = 0.0
        self.accel_cmd = 0.0
        self.port = port
        self.time_scale = time_scale
        self.running = True
        self.sim_time = 0.0
        self.log = []  # (t, x, y, h, v, steer, accel)

    def step(self):
        # 자전거 모델 (후축 기준)
        self.v = max(0.0, self.v + np.clip(self.accel_cmd, ACCEL_MIN, ACCEL_MAX) * DT)
        steer = np.clip(self.steer_cmd, -MAX_STEER, MAX_STEER)
        self.x += self.v * np.cos(self.h) * DT
        self.y += self.v * np.sin(self.h) * DT
        self.h += self.v / WHEELBASE * np.tan(steer) * DT
        self.sim_time += DT
        self.log.append((self.sim_time, self.x, self.y, self.h, self.v, steer, self.accel_cmd))

    def serve(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", self.port))
        srv.listen(1)
        srv.settimeout(10.0)
        conn, _ = srv.accept()
        conn.settimeout(5.0)

        # 제어 수신 스레드
        def rx():
            buf = b""
            while self.running:
                try:
                    chunk = conn.recv(4096)
                except (socket.timeout, OSError):
                    break
                if not chunk:
                    break
                buf += chunk
                while len(buf) >= CTRL_SIZE:
                    steer, accel, sig = unpack_ctrl(buf[:CTRL_SIZE])
                    buf = buf[CTRL_SIZE:]
                    if np.isfinite(steer) and np.isfinite(accel):  # 실기와 동일: NaN 무시
                        self.steer_cmd, self.accel_cmd = steer, accel

        threading.Thread(target=rx, daemon=True).start()
        period = DT / self.time_scale
        try:
            while self.running:
                t0 = time.time()
                self.step()
                state = VehicleState(self.x, self.y, 0.0, self.h, 0.0, 0.0,
                                     objects=[], tl_id=0, tl_state=0)
                conn.sendall(pack_data(state))
                dt_sleep = period - (time.time() - t0)
                if dt_sleep > 0:
                    time.sleep(dt_sleep)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            conn.close()
            srv.close()

    def start(self):
        t = threading.Thread(target=self.serve, daemon=True)
        t.start()
        return t

    def stop(self):
        self.running = False

#!/usr/bin/env python3
"""VTD 모의 서버 — TCP 9910 프로토콜(DataPacket 1109B / CtrlPacket 9B)로 자전거 모델 차량을 굴린다.

용도
  1) 자체 스택 폐루프 회귀 (run_closed_loop.py가 import)
  2) Autoware 실기 모드 회귀: 시뮬 PC 없이 ./start_autoware.sh mock 으로 브리지+Autoware 전체를 검증
     시나리오 옵션: 신호등 state, 시각별 state 변경, 리스폰(좌표 점프), 연결 끊김, ego z, 정지 차량 object

예)  python3 mock_vtd.py --x 4.9 --y -24.6 --hdg 1.70 --tl 1 --tl-at 40:3 --respawn-at 60 --drop-at 90
     (40초에 녹색, 60초에 시작점으로 리스폰, 90초에 연결을 끊고 재접속 허용)
"""
import argparse
import os
import socket
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tools'))
from hlvtd_io import VehicleState, pack_data, CTRL_SIZE, unpack_ctrl  # noqa: E402

WHEELBASE = 2.944
MAX_STEER = 0.48
ACCEL_MIN, ACCEL_MAX = -6.0, 3.0
DT = 0.05  # 시뮬 시간 스텝 (20Hz)


class MockVTD:
    def __init__(self, x0, y0, hdg0, host="127.0.0.1", port=9910, time_scale=1.0, z=0.0,
                 tl_state=0, tl_schedule=(), respawn_at=(), drop_at=(), objects=(), verbose=False, cruise=None):
        self.x0, self.y0, self.h0 = float(x0), float(y0), float(hdg0)
        self.x, self.y, self.h = self.x0, self.y0, self.h0
        self.z = float(z)
        self.v = 0.0
        self.steer_cmd = 0.0
        self.accel_cmd = 0.0
        self.turn_cmd = 0
        self.host, self.port = host, port
        self.time_scale = time_scale
        self.running = True
        self.sim_time = 0.0
        self.log = []  # (t, x, y, h, v, steer, accel)
        self.tl_state = tl_state
        self.tl_schedule = sorted(tl_schedule)   # [(t, state)]
        self.respawn_at = sorted(respawn_at)     # [t]
        self.drop_at = sorted(drop_at)           # [t]
        self.objects = list(objects)             # [(id,x,y,z,heading,speed,length,width,height)]
        self.verbose = verbose
        self.max_v = 0.0
        self.cruise = cruise   # [m/s] 지정 시 제어 명령 무시하고 정속 직진 (브리지 단독 시험용)

    def step(self):
        if self.cruise is not None:
            self.v = float(self.cruise)
        else:
            self.v = max(0.0, self.v + float(np.clip(self.accel_cmd, ACCEL_MIN, ACCEL_MAX)) * DT)
        steer = float(np.clip(self.steer_cmd, -MAX_STEER, MAX_STEER))
        self.x += self.v * np.cos(self.h) * DT
        self.y += self.v * np.sin(self.h) * DT
        self.h += self.v / WHEELBASE * np.tan(steer) * DT
        self.sim_time += DT
        self.max_v = max(self.max_v, self.v)
        self.log.append((self.sim_time, self.x, self.y, self.h, self.v, steer, self.accel_cmd))
        while self.tl_schedule and self.sim_time >= self.tl_schedule[0][0]:
            _, self.tl_state = self.tl_schedule.pop(0)
            print(f"[mock] t={self.sim_time:.1f} 신호등 state → {self.tl_state}", flush=True)
        while self.respawn_at and self.sim_time >= self.respawn_at[0]:
            self.respawn_at.pop(0)
            self.x, self.y, self.h, self.v = self.x0, self.y0, self.h0, 0.0
            print(f"[mock] t={self.sim_time:.1f} 리스폰 → 시작점", flush=True)

    def _serve_conn(self, conn):
        conn.settimeout(5.0)
        alive = True

        def rx():
            nonlocal alive
            buf = b""
            while self.running and alive:
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
                        self.steer_cmd, self.accel_cmd, self.turn_cmd = steer, accel, sig
            alive = False

        threading.Thread(target=rx, daemon=True).start()
        period = DT / self.time_scale
        last_print = 0.0
        try:
            while self.running and alive:
                t0 = time.time()
                self.step()
                if self.drop_at and self.sim_time >= self.drop_at[0]:
                    self.drop_at.pop(0)
                    print(f"[mock] t={self.sim_time:.1f} 연결 끊음 (재접속 허용)", flush=True)
                    break
                state = VehicleState(self.x, self.y, self.z, self.h, 0.0, 0.0,
                                     objects=self.objects, tl_id=1 if self.tl_state else 0,
                                     tl_state=self.tl_state)
                conn.sendall(pack_data(state))
                if self.verbose and self.sim_time - last_print >= 2.0:
                    last_print = self.sim_time
                    print(f"[mock] t={self.sim_time:.0f}s pos=({self.x:.1f},{self.y:.1f}) v={self.v*3.6:.1f}km/h "
                          f"steer={self.steer_cmd:+.3f} accel={self.accel_cmd:+.2f} turn={self.turn_cmd} tl={self.tl_state}",
                          flush=True)
                dt_sleep = period - (time.time() - t0)
                if dt_sleep > 0:
                    time.sleep(dt_sleep)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            alive = False
            conn.close()
            # 실기와 동일: 연결 끊기면 제어 0
            self.steer_cmd, self.accel_cmd = 0.0, 0.0

    def serve(self, accept_timeout=10.0, reconnect=False):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(1)
        srv.settimeout(accept_timeout)
        try:
            while self.running:
                try:
                    conn, addr = srv.accept()
                except socket.timeout:
                    if reconnect:
                        continue
                    break
                print(f"[mock] 접속 {addr}", flush=True)
                self._serve_conn(conn)
                if not reconnect:
                    break
        finally:
            srv.close()

    def start(self):
        t = threading.Thread(target=self.serve, daemon=True)
        t.start()
        return t

    def stop(self):
        self.running = False


def _parse_sched(items):
    out = []
    for s in items or []:
        t, st = s.split(":")
        out.append((float(t), int(st)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9910)
    ap.add_argument("--x", type=float, default=4.9)
    ap.add_argument("--y", type=float, default=-24.6)
    ap.add_argument("--hdg", type=float, default=1.704, help="[rad] route_example 시작점 차선 방향")
    ap.add_argument("--z", type=float, default=36.63, help="ego z (기본: route_example 시작점의 맵 높이. 실기는 40~79m)")
    ap.add_argument("--tl", type=int, default=0, help="초기 신호등 state (0~6)")
    ap.add_argument("--tl-at", action="append", metavar="T:STATE", help="시각 T초에 state 변경 (반복 가능)")
    ap.add_argument("--respawn-at", type=float, action="append", help="시각 T초에 시작점으로 리스폰")
    ap.add_argument("--drop-at", type=float, action="append", help="시각 T초에 연결 끊기 (재접속 허용)")
    ap.add_argument("--car-ahead", type=float, metavar="DIST", help="시작점 전방 DIST m에 정지 차량 object 1대")
    ap.add_argument("--cruise", type=float, help="[m/s] 제어 무시 정속 직진 (브리지 단독 시험용)")
    ap.add_argument("--time-scale", type=float, default=1.0)
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    objs = []
    if a.car_ahead:
        ox = a.x + a.car_ahead * np.cos(a.hdg)
        oy = a.y + a.car_ahead * np.sin(a.hdg)
        objs.append((7, float(ox), float(oy), a.z, a.hdg, 0.0, 4.5, 1.8, 1.5))
    m = MockVTD(a.x, a.y, a.hdg, host=a.host, port=a.port, time_scale=a.time_scale, z=a.z,
                tl_state=a.tl, tl_schedule=_parse_sched(a.tl_at), respawn_at=a.respawn_at or (),
                drop_at=a.drop_at or (), objects=objs, verbose=not a.quiet, cruise=a.cruise)
    print(f"[mock] {a.host}:{a.port} 대기. 시작 ({a.x},{a.y}) hdg {a.hdg} tl={a.tl}", flush=True)
    try:
        m.serve(accept_timeout=30.0, reconnect=True)
    except KeyboardInterrupt:
        pass
    print(f"[mock] 종료. 최고속 {m.max_v*3.6:.1f} km/h, 최종 ({m.x:.1f},{m.y:.1f}), 이동 "
          f"{sum(np.hypot(m.log[i][1]-m.log[i-1][1], m.log[i][2]-m.log[i-1][2]) for i in range(1, len(m.log))):.0f} m")


if __name__ == "__main__":
    main()

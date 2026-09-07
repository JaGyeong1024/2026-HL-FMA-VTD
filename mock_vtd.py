#!/usr/bin/env python3
"""VTD 모의 서버 — TCP 9910 프로토콜(DataPacket 1109B / CtrlPacket 9B)로 자전거 모델 차량을 굴린다.

용도
  1) 자체 스택 폐루프 회귀 (run_closed_loop.py가 import)
  2) Autoware 실기 모드 회귀: 시뮬 PC 없이 ./start_autoware.sh mock 으로 브리지+Autoware 전체를 검증
     시나리오 옵션: 신호등 state, 시각별 state 변경, 리스폰(좌표 점프), 연결 끊김, ego z, 정지 차량 object,
     객체 추가/제거 스케줄, 이동 객체(보행자 급출발 등), 스텝 단위 trace CSV (하네스 측정용)

예)  python3 mock_vtd.py --x 4.9 --y -24.6 --hdg 1.70 --tl 1 --tl-at 40:3 --respawn-at 60 --drop-at 90
     (40초에 녹색, 60초에 시작점으로 리스폰, 90초에 연결을 끊고 재접속 허용)

객체 옵션 (좌표는 시작 pose 기준 상대: DX=전방 m, DY=좌측 m, HDG=시작 방향 기준 상대 각 deg)
  --obj ID,DX,DY,HDG,SPEED,LEN,WID          정지/정속 객체 (SPEED>0 이면 등속 직진)
  --obj-abs ID,X,Y,Z,HDG_RAD,SPEED,LEN,WID  절대 좌표 객체 (tools/harness/place.py 가 만들어 줌)
  --obj-at T:add:ID | T:del:ID              시각 T 에 객체를 보이게/안 보이게
  --mover ID,DX,DY,HDG,SPEED,LEN,WID,T0[,T1]  T0 에 나타나 등속 직진, T1 에 사라짐 (보행자 급출발 재현)
  --trace FILE                              매 스텝 wall,t,x,y,h,v,steer,accel,tl,n_obj 을 CSV 로 기록
  --start-deadband A                        정지 상태(v<0.05)에서 가속 명령이 A m/s² 미만이면 움직이지 않음
                                            (VTD 출발 데드밴드 모사. 실측값 확정 전엔 가정치 — 재출발 교착 D1 재현용)
  --clock move                              위 스케줄(--tl-at/--respawn-at/--obj-at/--mover T0,T1)의 시각 기준을
                                            "접속 시각"이 아니라 "ego 가 처음 움직인 시각(v>0.3)" 으로 (하네스용)
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
                 tl_state=0, tl_schedule=(), respawn_at=(), drop_at=(), objects=(), verbose=False, cruise=None,
                 obj_schedule=(), movers=(), hidden_ids=(), trace=None, clock="connect", start_deadband=0.0):
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
        # 객체: dict id -> [id,x,y,z,heading,speed,length,width,height] (speed>0 이면 매 스텝 등속 직진)
        self.obj_defs = {int(o[0]): list(o) for o in objects}
        self.hidden = set(int(i) for i in hidden_ids)          # --obj-at add 로 나타날 때까지 숨김
        self.obj_schedule = sorted(obj_schedule)                # [(t, 'add'|'del', id)]
        # 이동 객체: (id, x, y, z, heading, speed, length, width, height, t0, t1)  t0 에 등장, t1(None=영구) 에 소멸
        self.movers = [list(mv) for mv in movers]
        self.trace_f = open(trace, 'w') if trace else None
        if self.trace_f:
            self.trace_f.write('wall,t,x,y,h,v,steer,accel,tl,n_obj\n')
        self.start_deadband = float(start_deadband)   # [m/s²] 정지 상태 출발 데드밴드 (0=없음)
        self.clock_mode = clock          # "connect": 접속 후 시뮬 시각 / "move": ego 첫 이동 후 경과 시각
        self.t_move = None               # ego 가 처음 v>0.3 이 된 시뮬 시각
        self.verbose = verbose
        self.max_v = 0.0
        self.cruise = cruise   # [m/s] 지정 시 제어 명령 무시하고 정속 직진 (브리지 단독 시험용)

    def step(self):
        if self.cruise is not None:
            self.v = float(self.cruise)
        else:
            a_cmd = float(np.clip(self.accel_cmd, ACCEL_MIN, ACCEL_MAX))
            if self.v < 0.05 and 0.0 < a_cmd < self.start_deadband:
                a_cmd = 0.0                      # 데드밴드: 약한 가속 명령으론 출발 못 함
            self.v = max(0.0, self.v + a_cmd * DT)
        steer = float(np.clip(self.steer_cmd, -MAX_STEER, MAX_STEER))
        self.x += self.v * np.cos(self.h) * DT
        self.y += self.v * np.sin(self.h) * DT
        self.h += self.v / WHEELBASE * np.tan(steer) * DT
        self.sim_time += DT
        self.max_v = max(self.max_v, self.v)
        if self.t_move is None and self.v > 0.3:
            self.t_move = self.sim_time
            print(f"[mock] t={self.sim_time:.1f} ego 첫 이동 (스케줄 기준 시각{' = 지금' if self.clock_mode == 'move' else ' 무관'})", flush=True)
        tc = self.sched_clock()
        self.log.append((self.sim_time, self.x, self.y, self.h, self.v, steer, self.accel_cmd))
        while tc is not None and self.tl_schedule and tc >= self.tl_schedule[0][0]:
            _, self.tl_state = self.tl_schedule.pop(0)
            print(f"[mock] t={self.sim_time:.1f} 신호등 state → {self.tl_state}", flush=True)
        while tc is not None and self.respawn_at and tc >= self.respawn_at[0]:
            self.respawn_at.pop(0)
            self.x, self.y, self.h, self.v = self.x0, self.y0, self.h0, 0.0
            print(f"[mock] t={self.sim_time:.1f} 리스폰 → 시작점", flush=True)
        while tc is not None and self.obj_schedule and tc >= self.obj_schedule[0][0]:
            _, act, oid = self.obj_schedule.pop(0)
            (self.hidden.discard if act == 'add' else self.hidden.add)(oid)
            print(f"[mock] t={self.sim_time:.1f} 객체 {oid} {act}", flush=True)
        for o in self.obj_defs.values():          # 정속 객체 전진
            if o[5] > 0.0:
                o[1] += o[5] * np.cos(o[4]) * DT
                o[2] += o[5] * np.sin(o[4]) * DT
        for mv in self.movers:                    # 이동 객체 (등장 후 등속 직진)
            if tc is not None and mv[9] <= tc and (mv[10] is None or tc < mv[10]):
                mv[1] += mv[5] * np.cos(mv[4]) * DT
                mv[2] += mv[5] * np.sin(mv[4]) * DT
        if self.trace_f:
            self.trace_f.write(f"{time.time():.3f},{self.sim_time:.2f},{self.x:.3f},{self.y:.3f},{self.h:.4f},{self.v:.3f},"
                               f"{steer:.4f},{self.accel_cmd:.3f},{self.tl_state},{len(self.visible_objects())}\n")
            if int(self.sim_time * 20) % 20 == 0:
                self.trace_f.flush()

    def sched_clock(self):
        """스케줄 비교용 시각. clock=move 면 ego 첫 이동 전엔 None(아무 스케줄도 안 발동)."""
        if self.clock_mode == "move":
            return None if self.t_move is None else self.sim_time - self.t_move
        return self.sim_time

    def visible_objects(self):
        """지금 이 프레임에 보내는 객체 목록 [(id,x,y,z,heading,speed,length,width,height)]."""
        tc = self.sched_clock()
        out = [tuple(o) for oid, o in self.obj_defs.items() if oid not in self.hidden]
        for mv in self.movers:
            if tc is not None and mv[9] <= tc and (mv[10] is None or tc < mv[10]):
                out.append(tuple(mv[:9]))
        return out

    def object_positions(self):
        """하네스 측정용: 현재 보이는 객체의 (id,x,y,heading,length,width)."""
        return [(o[0], o[1], o[2], o[4], o[6], o[7]) for o in self.visible_objects()]

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
                                     objects=self.visible_objects(), tl_id=1 if self.tl_state else 0,
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
    ap.add_argument("--car-ahead", type=float, metavar="DIST", help="시작점 전방 DIST m에 정지 차량 object 1대 (id 7)")
    ap.add_argument("--obj", action="append", metavar="ID,DX,DY,HDG,SPEED,LEN,WID",
                    help="시작 pose 기준 상대 배치 객체 (DX 전방 m, DY 좌측 m, HDG 상대각 deg). 반복 가능")
    ap.add_argument("--obj-abs", action="append", metavar="ID,X,Y,Z,HDG_RAD,SPEED,LEN,WID",
                    help="절대 좌표 객체 (tools/harness/place.py 출력). 반복 가능")
    ap.add_argument("--obj-at", action="append", metavar="T:add|del:ID", help="시각 T 에 객체 표시/숨김. add 대상은 그 전까지 숨김")
    ap.add_argument("--mover", action="append", metavar="ID,DX,DY,HDG,SPEED,LEN,WID,T0[,T1]",
                    help="T0 에 등장해 등속 직진하는 객체 (상대 배치). T1 에 소멸(생략=영구)")
    ap.add_argument("--trace", help="스텝 단위 trace CSV 경로 (하네스 측정용)")
    ap.add_argument("--start-deadband", type=float, default=0.0, help="[m/s²] 정지 출발 데드밴드 (0=없음)")
    ap.add_argument("--clock", choices=["connect", "move"], default="connect",
                    help="스케줄 시각 기준: connect=접속 후 시뮬 시각(기본), move=ego 첫 이동 후 경과 시각")
    ap.add_argument("--cruise", type=float, help="[m/s] 제어 무시 정속 직진 (브리지 단독 시험용)")
    ap.add_argument("--time-scale", type=float, default=1.0)
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()

    def rel(dx, dy, hdg_deg):
        """시작 pose 기준 (전방 dx, 좌측 dy, 상대각 deg) → 절대 (x, y, heading)."""
        c, s_ = np.cos(a.hdg), np.sin(a.hdg)
        return (a.x + dx * c - dy * s_, a.y + dx * s_ + dy * c, a.hdg + np.radians(hdg_deg))

    objs = []
    if a.car_ahead:
        ox, oy, oh = rel(a.car_ahead, 0.0, 0.0)
        objs.append((7, float(ox), float(oy), a.z, float(oh), 0.0, 4.5, 1.8, 1.5))
    for spec in a.obj or []:
        i, dx, dy, hd, sp, ln, wd = [float(v) for v in spec.split(",")]
        ox, oy, oh = rel(dx, dy, hd)
        objs.append((int(i), float(ox), float(oy), a.z, float(oh), sp, ln, wd, 1.5))
    for spec in a.obj_abs or []:
        i, x, y, z, hr, sp, ln, wd = [float(v) for v in spec.split(",")]
        objs.append((int(i), x, y, z, hr, sp, ln, wd, 1.5))
    sched, hidden = [], set()
    for spec in a.obj_at or []:
        t, act, i = spec.split(":")
        sched.append((float(t), act, int(i)))
        if act == "add":
            hidden.add(int(i))
    movers = []
    for spec in a.mover or []:
        f = [float(v) for v in spec.split(",")]
        i, dx, dy, hd, sp, ln, wd, t0 = f[:8]
        t1 = f[8] if len(f) > 8 else None
        ox, oy, oh = rel(dx, dy, hd)
        movers.append((int(i), float(ox), float(oy), a.z, float(oh), sp, ln, wd, 1.7, t0, t1))
    m = MockVTD(a.x, a.y, a.hdg, host=a.host, port=a.port, time_scale=a.time_scale, z=a.z,
                tl_state=a.tl, tl_schedule=_parse_sched(a.tl_at), respawn_at=a.respawn_at or (),
                drop_at=a.drop_at or (), objects=objs, verbose=not a.quiet, cruise=a.cruise,
                obj_schedule=sched, movers=movers, hidden_ids=hidden, trace=a.trace, clock=a.clock,
                start_deadband=a.start_deadband)
    if objs or movers:
        print(f"[mock] 객체 {len(objs)}개, 이동객체 {len(movers)}개, 스케줄 {len(sched)}건", flush=True)
        for o in objs:
            print(f"[mock] 객체초기 id={int(o[0])} x={o[1]:.3f} y={o[2]:.3f} hdg={o[4]:.4f} speed={o[5]:.2f} len={o[6]:.2f} wid={o[7]:.2f}", flush=True)
        for mv in movers:
            print(f"[mock] 이동객체 id={int(mv[0])} x={mv[1]:.3f} y={mv[2]:.3f} hdg={mv[4]:.4f} speed={mv[5]:.2f} len={mv[6]:.2f} wid={mv[7]:.2f} t0={mv[9]} t1={mv[10]}", flush=True)
    print(f"[mock] {a.host}:{a.port} 대기. 시작 ({a.x},{a.y}) hdg {a.hdg} tl={a.tl}", flush=True)
    try:
        m.serve(accept_timeout=30.0, reconnect=True)
    except KeyboardInterrupt:
        pass
    for o in list(m.obj_defs.values()) + [mv[:9] for mv in m.movers]:
        print(f"[mock] 객체정의 id={int(o[0])} x={o[1]:.3f} y={o[2]:.3f} hdg={o[4]:.4f} speed={o[5]:.2f} len={o[6]:.2f} wid={o[7]:.2f}", flush=True)
    print(f"[mock] 종료. 최고속 {m.max_v*3.6:.1f} km/h, 최종 ({m.x:.1f},{m.y:.1f}), 이동 "
          f"{sum(np.hypot(m.log[i][1]-m.log[i-1][1], m.log[i][2]-m.log[i-1][2]) for i in range(1, len(m.log))):.0f} m")


if __name__ == "__main__":
    main()

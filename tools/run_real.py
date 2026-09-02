"""실기 VTD 주행 러너 — 자체 스택(Pure Pursuit)을 실기 VTD에 연결.

대회장용. SCP(48179) 미사용 — 이미 돌고 있는(운영측이 채점 세션을 띄운) VTD에
9910/TCP로 바로 붙어서 주행만 한다. 시나리오 재시작·관전 카메라 등 SCP를 쓰는
기능은 이 파일에 없다 (연구실 전용 도구는 lab_restart_scenario.py 참조).

usage: python3 run_real.py [host] [route.csv] [xodr] [max_sec]

절차: 경로 생성 → 9910 접속(attach) → 20Hz 제어 루프.
종료: 종점 도달 / 시간 초과 / Ctrl+C. 종료 시 정지 명령 송신, 통계·플롯(out/real_run.png) 출력.
리스폰 감지: 프레임 간 위치 점프 > 3m.
"""
import csv
import math
import sys
import time
from pathlib import Path

import numpy as np

from xodr_map import OpenDriveMap
from lane_graph import LaneGraph, build_route
from hlvtd_io import VTDClient
from controller import PathTracker, Controller

ROUTE_CONFIG = Path.home() / "hlfma/route/route_config.yaml"


def route_from_config():
    """route_config.yaml의 csv_path 한 줄을 읽는다 (경로 지정의 단일 기준점)."""
    for line in ROUTE_CONFIG.read_text().splitlines():
        line = line.split("#")[0].strip()
        if line.startswith("csv_path:"):
            return Path(line.split(":", 1)[1].strip())
    raise SystemExit(f"{ROUTE_CONFIG}에 csv_path 없음")


HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.50.11"
ROUTE = Path(sys.argv[2]) if len(sys.argv) > 2 else route_from_config()
XODR = Path(sys.argv[3] if len(sys.argv) > 3 else "HL_FMA_VTD_LivingLab.xodr")
MAX_SEC = float(sys.argv[4]) if len(sys.argv) > 4 else 600.0
print(f"경로 CSV: {ROUTE}")


print("맵 로드...")
m = OpenDriveMap(XODR)
graph = LaneGraph(m)

wps = []
with open(ROUTE) as f:
    for row in csv.DictReader(f):
        wps.append((float(row["x"]), float(row["y"])))
path, node_seq = build_route(graph, wps)
plen = float(np.sum(np.hypot(*np.diff(path, axis=0).T)))
print(f"경로: waypoint {len(wps)} → {len(path)}점, 총 {plen:.0f} m")

print("attach 모드 — SCP 미사용, 이미 도는 시뮬에 9910으로 바로 접속")

# 데이터 수신 대기
client = None
st = None
deadline = time.time() + 30.0
while time.time() < deadline:
    try:
        if client is None:
            client = VTDClient(HOST, timeout=5.0)
        st = client.recv_state()
        break
    except OSError:
        if client is not None:
            client.close()
            client = None
        time.sleep(2.0)
if st is None:
    raise SystemExit("30초 내 VTD 데이터 수신 실패")

tracker = PathTracker(path)
ctrl = Controller(tracker)
d0 = math.hypot(st.x - path[0, 0], st.y - path[0, 1])
print(f"ego 시작 ({st.x:.1f},{st.y:.1f}), 경로 시작점과 {d0:.1f} m")

def reconnect(old):
    """연결 유실 시 재접속 (대회장 대비: 데이터 끊겨도 제어기는 버틴다)."""
    try:
        old.close()
    except OSError:
        pass
    deadline = time.time() + 60.0
    while time.time() < deadline:
        try:
            c = VTDClient(HOST, timeout=5.0)
            c.recv_state()
            print("  ↻ VTD 재접속 성공")
            return c
        except OSError:
            time.sleep(2.0)
    raise SystemExit("재접속 60초 실패")


t0 = time.time()
log, errs, respawns = [], [], 0
prev_xy = (st.x, st.y)
try:
    while True:
        try:
            st = client.recv_state()
        except (OSError, ConnectionError):
            print("  ⚠ 연결 유실 → 재접속 시도")
            client = reconnect(client)
            continue
        jump = math.hypot(st.x - prev_xy[0], st.y - prev_xy[1])
        if jump > 3.0:
            respawns += 1
            # 리스폰은 뒤(구간 시작점)로 돌아가므로 [0, 현재+100] 범위에서 재탐색
            hi = min(len(path), tracker._last_i + 100)
            d_all = np.hypot(path[:hi, 0] - st.x, path[:hi, 1] - st.y)
            tracker._last_i = int(np.argmin(d_all))
            ctrl.v_est = 0.0
            ctrl._prev = None  # pose 점프로 인한 속도 스파이크 방지
            print(f"  ⚠ 리스폰 #{respawns} (점프 {jump:.1f} m, t={time.time()-t0:.0f}s) "
                  f"→ 경로 인덱스 {tracker._last_i}로 재정렬")
        prev_xy = (st.x, st.y)

        steer, accel, sig, dbg = ctrl.update(st.x, st.y, st.heading)
        try:
            client.send_ctrl(steer, accel, sig)
        except OSError:
            pass  # 다음 recv에서 재접속 처리
        errs.append(dbg["lat_err"])
        log.append((time.time() - t0, st.x, st.y, st.heading, dbg.get("v_est", 0.0)))

        if tracker.finished(st.x, st.y):
            print("🏁 종점 도달!")
            break
        if time.time() - t0 > MAX_SEC:
            print("시간 초과 — 중단")
            break
except KeyboardInterrupt:
    print("사용자 중단")
finally:
    for _ in range(20):
        try:
            client.send_ctrl(0.0, -4.0, 0)
            time.sleep(0.05)
        except OSError:
            break
    client.close()

log_a = np.array(log)
errs_a = np.array(errs)
dur = log_a[-1, 0] if len(log_a) else 0.0
print(f"\n=== 결과 ===")
print(f"주행 {dur:.0f}s, 스텝 {len(log_a)}, 리스폰 {respawns}회")
if len(errs_a):
    print(f"횡오차: 평균 {errs_a.mean():.2f} / 최대 {errs_a.max():.2f} / 95% {np.percentile(errs_a, 95):.2f} m")
    print(f"횡오차 > 1.75 m 비율: {(errs_a > 1.75).mean()*100:.1f}%")

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    out = Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.plot(path[:, 0], path[:, 1], "b-", lw=1.2, label="planned")
    if len(log_a):
        ax.plot(log_a[:, 1], log_a[:, 2], "r--", lw=1.0, label="driven")
    ax.plot(*zip(*wps), "ko", ms=5)
    ax.set_aspect("equal"); ax.legend(); ax.set_title(f"real run: {dur:.0f}s, respawn {respawns}")
    fig.savefig(out / "real_run.png", dpi=110, bbox_inches="tight")
    print(f"플롯: {out/'real_run.png'}")
except Exception as e:
    print(f"(플롯 생략: {e})")

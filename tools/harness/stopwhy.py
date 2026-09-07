#!/usr/bin/env python3
"""정지·감속 사유 타임라인 (bag): planning_factors 전 모듈의 STOP(3)/SLOW_DOWN(2) 요인을 시간순으로, 어느 모듈이 먼저 반응했는지.
run_out 은 SLOW_DOWN 단계와 STOP 단계를 구분해 표시. ego 속도·가속 명령을 같이 찍어 감속 시작과 대조.
usage: stopwhy.py <OUT_DIR> [t_from_bag_s] [t_to_bag_s]"""
import sys, os
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
OUT = sys.argv[1]; tf = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0; tt = float(sys.argv[3]) if len(sys.argv) > 3 else 1e9
r = SequentialReader(); r.open(StorageOptions(uri=os.path.join(OUT, 'bag'), storage_id='mcap'), ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
BEH = {0: '-', 1: 'SLOW?', 2: 'SLOW', 3: 'STOP'}
t0 = None; events = []; last = {}; vel = []; cmd = []
while r.has_next():
    topic, data, ts = r.read_next(); t0 = t0 or ts; t = (ts - t0) / 1e9
    if t < tf or t > tt: continue
    if topic == '/vehicle/status/velocity_status':
        m = deserialize_message(data, get_message(types[topic])); vel.append((t, m.longitudinal_velocity))
    elif topic == '/control/command/control_cmd':
        m = deserialize_message(data, get_message(types[topic])); cmd.append((t, m.longitudinal.acceleration))
    elif topic.startswith('/planning/planning_factors/'):
        m = deserialize_message(data, get_message(types[topic])); mod = topic.split('/')[-1]
        if not m.factors:
            st = (0, None, '')
        else:
            f = max(m.factors, key=lambda f: f.behavior)
            d = min((cp.distance for cp in f.control_points), default=None)
            st = (f.behavior, None if d is None else round(d, 1), f.detail)
        prev = last.get(mod, (0, None, ''))
        if st[0] != prev[0] or (st[1] is not None and prev[1] is not None and abs(st[1] - prev[1]) > 5.0) or st[2] != prev[2]:
            events.append((t, mod, st)); last[mod] = st
def v_at(t):
    return next((v for tv, v in vel if tv >= t), None)
def a_at(t):
    return next((a for ta, a in cmd if ta >= t), None)
print("== 요인 타임라인 (bag t | 모듈 | behavior | 거리 m | detail | ego v m/s | accel cmd)")
first = {}
for t, mod, (b, d, det) in events:
    if b in (2, 3) and mod not in first: first[mod] = (round(t, 1), BEH[b])
    va, aa = v_at(t), a_at(t)
    print(f"  {t:7.1f} | {mod:22s} | {str(BEH.get(b, b)):5s} | {'' if d is None else d:>6} | {det[:40]:40s} | {'' if va is None else round(va, 2):>5} | {'' if aa is None else round(aa, 2)}")
print("== 모듈별 첫 반응(SLOW/STOP):", dict(sorted(first.items(), key=lambda kv: kv[1][0])))
if vel:
    zt = [t for t, v in vel if v < 0.05]; print("== 정지 시작(bag t):", round(zt[0], 1) if zt else None, " 최저속:", round(min(v for t, v in vel), 2))

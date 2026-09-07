#!/usr/bin/env python3
"""정지 사유: bag 의 planning_factors 중 behavior=STOP 인 모듈과 그 거리를 시간순으로 요약 (마지막 정지 구간 위주).
usage: stopwhy.py <OUT_DIR> [t_from_bag_s]"""
import sys, os
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
OUT = sys.argv[1]; tf = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
r = SequentialReader(); r.open(StorageOptions(uri=os.path.join(OUT, 'bag'), storage_id='mcap'), ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
t0 = None; seen = {}; vel = []
while r.has_next():
    topic, data, ts = r.read_next(); t0 = t0 or ts; t = (ts - t0) / 1e9
    if t < tf: continue
    if topic == '/vehicle/status/velocity_status':
        m = deserialize_message(data, get_message(types[topic])); vel.append((round(t, 1), round(m.longitudinal_velocity, 2)))
    if topic.startswith('/planning/planning_factors/'):
        m = deserialize_message(data, get_message(types[topic]))
        for f in m.factors:
            if f.behavior == 3:   # STOP
                d = min((cp.distance for cp in f.control_points), default=None)
                mod = topic.split('/')[-1]
                s = seen.setdefault(mod, []); 
                if not s or abs(s[-1][1] - (d or 0)) > 3.0: s.append((round(t, 1), None if d is None else round(d, 1), f.detail))
print("== STOP factor (모듈: [(bag t, 거리 m, detail)…])")
for mod, s in seen.items(): print(f"  {mod}: {s[:8]}")
z = [t for t, v in vel if v < 0.05]
print("== 정지 시작(bag t):", z[0] if z else None, " 속도 샘플 수", len(vel))

#!/usr/bin/env python3
"""감속 경로 분석: 계획 궤적(자차 위치 점의 속도·정지점 거리) vs 실제 속도 vs 제어 명령 가속(PID 출력·gate 출력) 시간열.
usage: decelpath.py <OUT_DIR> t_from t_to  (bag t, s)"""
import sys, os, math
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
OUT, tf, tt = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
r = SequentialReader(); r.open(StorageOptions(uri=os.path.join(OUT, 'bag'), storage_id='mcap'), ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
t0 = None; ego = None; v_act = None; rows = []; last_cmd = None; last_pid = None
while r.has_next():
    topic, data, ts = r.read_next(); t0 = t0 or ts; t = (ts - t0) / 1e9
    if t > tt: break
    if topic == '/localization/kinematic_state':
        m = deserialize_message(data, get_message(types[topic])); ego = m.pose.pose.position; v_act = m.twist.twist.linear.x
    elif topic == '/control/command/control_cmd':
        m = deserialize_message(data, get_message(types[topic])); last_cmd = (m.longitudinal.acceleration, m.longitudinal.velocity)
    elif topic == '/control/trajectory_follower/control_cmd':
        m = deserialize_message(data, get_message(types[topic])); last_pid = m.longitudinal.acceleration
    elif topic == '/planning/trajectory' and t >= tf and ego is not None:
        m = deserialize_message(data, get_message(types[topic]))
        pts = m.points
        if not pts: continue
        k = min(range(len(pts)), key=lambda i: math.hypot(pts[i].pose.position.x - ego.x, pts[i].pose.position.y - ego.y))
        v_plan = pts[k].longitudinal_velocity_mps
        d_stop = None; s = 0.0
        for i in range(k, len(pts) - 1):
            if pts[i].longitudinal_velocity_mps < 0.1:
                d_stop = s; break
            s += math.hypot(pts[i + 1].pose.position.x - pts[i].pose.position.x, pts[i + 1].pose.position.y - pts[i].pose.position.y)
        rows.append((round(t, 1), round(v_act or 0, 2), round(v_plan, 2), None if d_stop is None else round(d_stop, 1),
                     None if last_pid is None else round(last_pid, 2), None if last_cmd is None else round(last_cmd[0], 2)))
print("bag t | v 실제 | v 계획(자차점) | 계획 정지점까지 m | PID 가속 | gate 가속(→VTD)")
for row in rows[::2]:
    print("  " + " | ".join(str(x) for x in row))

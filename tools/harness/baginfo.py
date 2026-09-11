#!/usr/bin/env python3
"""bag 요약: 토픽별 메시지 수·첫/마지막 시각, mrm_state·operation_mode·routing 값 변화, diag 트리 ERROR leaf.
usage: baginfo.py <OUT_DIR or bag dir>"""
import sys, os, glob
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
p = sys.argv[1]
bag = p if os.path.exists(os.path.join(p, 'metadata.yaml')) else os.path.join(p, 'bag')
r = SequentialReader(); r.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
cnt, first, last, hist = {}, {}, {}, {}
watch = {'/api/fail_safe/mrm_state': lambda m: f"state={m.state} behavior={m.behavior}",
         '/api/operation_mode/state': lambda m: f"mode={m.mode} avail={m.is_autonomous_mode_available} in_transition={m.is_in_transition}",
         '/api/routing/state': lambda m: f"route={m.state}",
         '/system/emergency/hazard_status': lambda m: f"level={m.status.level} emergency={m.status.emergency} holding={m.status.emergency_holding}"}
t0 = None
diag_last = None
while r.has_next():
    topic, data, ts = r.read_next()
    t0 = t0 or ts
    cnt[topic] = cnt.get(topic, 0) + 1; first.setdefault(topic, ts); last[topic] = ts
    if topic in watch:
        m = deserialize_message(data, get_message(types[topic]))
        v = watch[topic](m)
        if hist.get(topic, [None])[-1] != v:
            hist.setdefault(topic, []).append(v); hist[topic + '_t'] = hist.get(topic + '_t', []) + [round((ts - t0) / 1e9, 1)]
    if topic == '/diagnostics_graph/status':
        diag_last = (ts, data)
print("== 토픽별 수 (t 는 bag 시작 기준 s)")
for k in sorted(cnt):
    print(f"  {k:70s} n={cnt[k]:6d}  first={(first[k]-t0)/1e9:6.1f} last={(last[k]-t0)/1e9:6.1f}")
print("== 상태 변화")
for k in watch:
    if k in hist:
        print(f"  {k}: " + " | ".join(f"t{t}:{v}" for t, v in zip(hist[k + '_t'], hist[k])))
# diag: struct 없이 status 만으론 이름 매핑 불가 → struct 토픽이 latched 라 bag 에 없을 수 있음. 레벨 분포만.
if diag_last:
    m = deserialize_message(diag_last[1], get_message(types['/diagnostics_graph/status']))
    lv = {}
    for n in m.nodes:
        lv[n.level] = lv.get(n.level, 0) + 1
    print("== 마지막 diag status 레벨 분포 (0 OK,1 WARN,2 ERROR,3 STALE):", lv)

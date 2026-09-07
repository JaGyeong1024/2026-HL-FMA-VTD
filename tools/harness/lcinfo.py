#!/usr/bin/env python3
"""차선변경 분석 (bag + trace): 방향지시등 에피소드(의도) ↔ 실제 이웃 lanelet 전환(실행) 매칭, 미실행(취소·차선 끝 정지) 집계.
usage: lcinfo.py <OUT_DIR>  → OUT_DIR/lc.json + 요약 출력. metrics.py --lanes 가 먼저 돌아 metrics.json 에 lane_changes 가 있어야 함."""
import sys, os, re, json
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
OUT = sys.argv[1]; bag = os.path.join(OUT, 'bag')
# bag t(초) ↔ mock t 변환: bag 시작 벽시계(bag_record.log) 와 trace 의 wall 열
m0 = re.search(r'\[(\d+\.\d+)\].*Starting recording', open(os.path.join(OUT, 'bag_record.log')).read())
bag_wall0 = float(m0.group(1)) if m0 else None
import csv
rows = list(csv.DictReader(open(os.path.join(OUT, 'trace.csv'))))
def mock_t(bag_t):
    w = bag_wall0 + bag_t
    for r in rows:
        if float(r['wall']) >= w:
            return float(r['t'])
    return None
r = SequentialReader(); r.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
t0 = None; ind = []; last = None; lc_factor = []
while r.has_next():
    topic, data, ts = r.read_next(); t0 = t0 or ts; t = (ts - t0) / 1e9
    if topic == '/control/command/turn_indicators_cmd':
        m = deserialize_message(data, get_message(types[topic]))
        if m.command != last:
            ind.append((t, m.command)); last = m.command
    elif topic.startswith('/planning/planning_factors/lane_change') or topic.startswith('/planning/planning_factors/external'):
        m = deserialize_message(data, get_message(types[topic]))
        if m.factors:
            lc_factor.append((round(t, 1), topic.split('/')[-1], m.factors[0].behavior,
                              round(min(cp.distance for f in m.factors for cp in f.control_points))))
# 지시등 에피소드 (LEFT=2/RIGHT=3, 1 s 이상)
eps = []
for i, (t, c) in enumerate(ind):
    if c in (2, 3):
        t_end = ind[i + 1][0] if i + 1 < len(ind) else t + 999
        if t_end - t >= 1.0:
            eps.append({'dir': 'L' if c == 2 else 'R', 'bag_t': round(t, 1), 'mock_t': mock_t(t), 'dur': round(t_end - t, 1)})
met = json.load(open(os.path.join(OUT, 'metrics.json')))
lcs = met.get('lane_changes', [])
stops = met.get('stops', [])
for e in eps:
    e['executed'] = any(e['mock_t'] is not None and e['mock_t'] - 1.0 <= tt <= e['mock_t'] + e['dur'] + 8.0 for tt, a, b in lcs)
    e['stopped_during'] = any(e['mock_t'] is not None and s0 <= e['mock_t'] + e['dur'] + 1.0 and s1 >= e['mock_t'] for s0, s1, x, y in stops)
res = {'episodes': eps, 'n_intent': len(eps), 'n_executed': sum(1 for e in eps if e['executed']),
       'n_unexecuted': sum(1 for e in eps if not e['executed']),
       'n_unexecuted_with_stop': sum(1 for e in eps if not e['executed'] and e['stopped_during']),
       'lc_factor_first': lc_factor[:6]}
json.dump(res, open(os.path.join(OUT, 'lc.json'), 'w'), indent=1)
print(f"차선변경 의도 {res['n_intent']} / 실행 {res['n_executed']} / 미실행 {res['n_unexecuted']} (그중 정지 동반 {res['n_unexecuted_with_stop']})")
for e in eps:
    print(f"  {e['dir']} bag t={e['bag_t']} mock t={e['mock_t']} dur={e['dur']}s executed={e['executed']} stopped={e['stopped_during']}")

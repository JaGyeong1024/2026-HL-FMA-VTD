#!/usr/bin/env python3
"""mock trace(스텝 단위)에서 하네스 판정값을 계산해 metrics.json 으로.

usage: metrics.py <OUT_DIR> [--obj ID] [--mark NAME] [--lanes]
  --obj ID     이 객체(정지차)와의 종방향 간격: 최소값·정지 시 간격(앞범퍼 기준)
  --mark NAME  events.json 의 marks[NAME](벽시계) 또는 mock.log 의 스케줄 시각 이후 재출발 시간(v>1.0 까지)
  --lanes      trace 점을 lanelet 에 매칭해 lanelet 열(차선변경 발생 여부) 계산
항상: 최고속, 최대 감속(명령·실측), 정지 구간, ego 첫 이동 시각, 이동객체 최소거리(있으면), 리스폰 시각(있으면)
"""
import sys, os, json, math, re, csv
OUT = sys.argv[1]
args = sys.argv[2:]
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
EGO_FRONT = 3.808

rows = list(csv.DictReader(open(os.path.join(OUT, 'trace.csv'))))
_ev = json.load(open(os.path.join(OUT, 'events.json'))) if os.path.exists(os.path.join(OUT, 'events.json')) else {}
_stop_wall = _ev.get('marks', {}).get('stop')
if _stop_wall:      # 스택 종료 신호 이후(브리지 죽으며 페일세이프 -3.0 감속 등)는 판정에서 제외
    rows = [r for r in rows if float(r['wall']) <= _stop_wall]
else:               # stop 마크가 없는 옛 실행: 페일세이프 -3.0 명령이 3 s 이상 이어지는 마지막 구간 = 스택 종료로 간주
    a = [float(r['accel']) for r in rows]; t = [float(r['t']) for r in rows]
    i = len(a) - 1
    while i > 0 and abs(a[i] + 3.0) < 1e-6:
        i -= 1
    if len(a) - 1 - i >= 60:
        rows = rows[:i + 1]
T = [float(r['t']) for r in rows]; W = [float(r['wall']) for r in rows]
X = [float(r['x']) for r in rows]; Y = [float(r['y']) for r in rows]; Hd = [float(r['h']) for r in rows]
V = [float(r['v']) for r in rows]; A = [float(r['accel']) for r in rows]
res = {'n': len(rows), 'duration_s': T[-1] - T[0] if rows else 0}
if not rows:
    json.dump(res, open(os.path.join(OUT, 'metrics.json'), 'w')); print(res); sys.exit(0)

mock_log = open(os.path.join(OUT, 'mock.log'), errors='replace').read()
ev = json.load(open(os.path.join(OUT, 'events.json'))) if os.path.exists(os.path.join(OUT, 'events.json')) else {}

def sim_at_wall(w):
    """벽시계 → 시뮬 시각 (trace wall 열 보간)."""
    for i in range(len(W)):
        if W[i] >= w:
            return T[i]
    return None

res['max_speed_kmh'] = round(max(V) * 3.6, 1)
# 실측 감속: 0.5 s 창 미분, **주행 중(v>0.5 m/s) 구간만** — 정지 유지 명령(-1.5/-3.4)·정지 직후 잡음 제외
dec, dec_t = 0.0, None
jump = [False] * len(V)          # 리스폰(순간이동) 스텝: 한 스텝 이동 > 2 m
for i in range(1, len(V)):
    jump[i] = math.hypot(X[i] - X[i - 1], Y[i] - Y[i - 1]) > 2.0
for i in range(10, len(V)):
    if any(jump[i - 10:i + 1]):
        continue
    if V[i - 10] > 0.5 and V[i] > 0.0:
        d = (V[i] - V[i - 10]) / (T[i] - T[i - 10] + 1e-9)
        if d < dec:
            dec, dec_t = d, T[i]
res['max_decel_measured'] = round(dec, 2)
res['max_decel_t'] = dec_t
res['max_decel_cmd_moving'] = round(min([A[i] for i in range(len(A)) if V[i] > 0.5], default=0.0), 2)   # 주행 중 명령 최소값
res['max_decel_cmd'] = round(min(A), 2)              # 전체(정지 유지 명령 포함) — 참고용
res['hold_cmd_at_stop'] = round(A[-1], 2) if V[-1] < 0.05 else None   # 정지 중 유지 명령
m = re.search(r't=([\d.]+) ego 첫 이동', mock_log)
res['t_move'] = float(m.group(1)) if m else None
if 'engage_wall' in ev:
    res['t_engage'] = sim_at_wall(ev['engage_wall'])
# 정지 구간 (v<0.05 가 1 s 이상, 첫 이동 이후)
stops = []
i = 0
while i < len(V):
    if V[i] < 0.05 and res['t_move'] is not None and T[i] > res['t_move']:
        j = i
        while j < len(V) and V[j] < 0.05:
            j += 1
        if T[j - 1] - T[i] >= 1.0:
            stops.append((round(T[i], 1), round(T[j - 1], 1), round(X[i], 1), round(Y[i], 1)))
        i = j
    else:
        i += 1
res['stops'] = stops
# 리스폰
res['respawn_t'] = [float(t) for t in re.findall(r't=([\d.]+) 리스폰', mock_log)]
# 스케줄 이벤트 (신호등 변경, 객체 add/del)
res['tl_changes'] = [(float(t), int(s)) for t, s in re.findall(r't=([\d.]+) 신호등 state → (\d)', mock_log)]
res['obj_events'] = [(float(t), a, int(i)) for t, i, a in re.findall(r't=([\d.]+) 객체 (\d+) (add|del)', mock_log)]

# 객체 정의 (초기 위치)
objs = {}
for oid, x, y, h, sp, ln, wd in re.findall(r'객체초기 id=(\d+) x=([-\d.]+) y=([-\d.]+) hdg=([-\d.]+) speed=([-\d.]+) len=([-\d.]+) wid=([-\d.]+)', mock_log):
    objs[int(oid)] = dict(x=float(x), y=float(y), h=float(h), speed=float(sp), len=float(ln), wid=float(wd))
movers = {}
for oid, x, y, h, sp, ln, wd, t0, t1 in re.findall(r'이동객체 id=(\d+) x=([-\d.]+) y=([-\d.]+) hdg=([-\d.]+) speed=([-\d.]+) len=([-\d.]+) wid=([-\d.]+) t0=([-\d.]+) t1=(\S+)', mock_log):
    movers[int(oid)] = dict(x=float(x), y=float(y), h=float(h), speed=float(sp), len=float(ln), wid=float(wd), t0=float(t0), t1=None if t1 == 'None' else float(t1))

def gap_to(obj, i):
    """i 번째 스텝에서 ego 앞범퍼→객체 뒷범퍼 종방향 간격 [m] (ego 진행방향 기준)."""
    fx, fy = X[i] + EGO_FRONT * math.cos(Hd[i]), Y[i] + EGO_FRONT * math.sin(Hd[i])
    dx, dy = obj['x'] - fx, obj['y'] - fy
    lon = dx * math.cos(Hd[i]) + dy * math.sin(Hd[i])
    lat = -dx * math.sin(Hd[i]) + dy * math.cos(Hd[i])
    return lon - obj['len'] / 2.0, lat

if '--obj' in args:
    oid = int(args[args.index('--obj') + 1])
    if oid in objs:
        gaps = [gap_to(objs[oid], i) for i in range(len(rows))]
        res['obj_min_gap'] = round(min(g for g, l in gaps), 2)
        # 정지 시 간격: 마지막 정지 구간 시작 시점
        if stops:
            ts = stops[-1][0]
            k = next(i for i in range(len(T)) if T[i] >= ts)
            res['obj_gap_at_stop'] = round(gaps[k][0], 2)
            res['obj_lat_at_stop'] = round(gaps[k][1], 2)
        # 객체 옆을 지나갔는지 (lon<0 인 순간이 있으면 통과)
        res['obj_passed'] = any(g < -objs[oid]['len'] for g, l in gaps)
        res['obj_passed_lat'] = round(min((abs(l) for g, l in gaps if -objs[oid]['len'] * 2 < g < objs[oid]['len'] * 2), default=float('nan')), 2)
    else:
        res['obj_min_gap'] = None

# 이동객체 최소 거리 (등장~소멸 사이, 스케줄 시각은 t_move 기준 또는 접속 기준 — mock.log 의 clock 표시로 구분)
clock_move = '스케줄 기준 시각 = 지금' in mock_log
for oid, mv in movers.items():
    base = (res['t_move'] or 0.0) if clock_move else 0.0
    dmin, tmin, v_at = 1e9, None, None
    for i in range(len(rows)):
        tc = T[i] - base
        if tc < mv['t0'] or (mv['t1'] is not None and tc >= mv['t1']):
            continue
        px = mv['x'] + mv['speed'] * math.cos(mv['h']) * (tc - mv['t0'])
        py = mv['y'] + mv['speed'] * math.sin(mv['h']) * (tc - mv['t0'])
        cx, cy = X[i] + 1.4 * math.cos(Hd[i]), Y[i] + 1.4 * math.sin(Hd[i])   # 차체 중심
        d = math.hypot(px - cx, py - cy)
        if d < dmin:
            dmin, tmin, v_at = d, T[i], V[i]
    res[f'mover{oid}_min_dist'] = round(dmin, 2)
    res[f'mover{oid}_t_min'] = tmin
    res[f'mover{oid}_v_at_min'] = round(v_at, 2) if v_at is not None else None
    res[f'mover{oid}_collision'] = dmin < 2.5   # 차체 반폭 0.94 + 보행자 0.3 + 여유

# 재출발: 마크(신호등 녹색 / 객체 del / 리스폰) 이후 v>1.0 까지
def restart_after(t_ev):
    """이벤트 0.5 s 이후에 v>1.0 이 1 s 이상 유지되는 첫 시각 (리스폰 직전 속도가 같은 스텝에 남는 것 배제)."""
    for i in range(len(T)):
        if T[i] >= t_ev + 0.5 and V[i] > 1.0:
            j = i
            while j < len(T) and V[j] > 1.0 and T[j] - T[i] < 1.0:
                j += 1
            if j < len(T) and T[j] - T[i] >= 1.0 or (j == len(T) and T[j - 1] - T[i] >= 1.0):
                return round(T[i] - t_ev, 1)
    return None
if '--mark' in args:
    name = args[args.index('--mark') + 1]
    t_ev = None
    if name == 'green':
        g = [t for t, s in res['tl_changes'] if s == 3]; t_ev = g[0] if g else None
    elif name == 'respawn':
        t_ev = res['respawn_t'][0] if res['respawn_t'] else None
    elif name.startswith('del'):
        d = [t for t, a, i in res['obj_events'] if a == 'del']; t_ev = d[0] if d else None
    elif name in ev.get('marks', {}):
        t_ev = sim_at_wall(ev['marks'][name])
    res['mark'] = name; res['mark_t'] = t_ev
    res['restart_after_mark_s'] = restart_after(t_ev) if t_ev is not None else None
    if t_ev is not None:
        k = next((i for i in range(len(T)) if T[i] >= t_ev), None)
        res['v_at_mark'] = round(V[k], 2) if k is not None else None
        res['stopped_before_mark'] = any(s[0] <= t_ev <= s[1] + 0.5 for s in stops)

# 출발 명령 크기 (데드밴드 대리 지표): 기준 시각 이후 첫 양의 가속 명령과 그 0.5 s 뒤 값. mock 은 데드밴드가 없어
# 어떤 양의 명령에도 움직이므로, "실 VTD 데드밴드보다 큰가"는 이 값과 실측 데드밴드를 비교해 판단한다.
def start_cmd_after(t_ev):
    if t_ev is None:
        return None
    k = next((i for i in range(len(T)) if T[i] >= t_ev and A[i] > 0.0), None)
    if k is None:
        return None
    k2 = min(k + 10, len(T) - 1)
    return {'t_first_pos': round(T[k] - t_ev, 2), 'a_first': round(A[k], 2), 'a_plus0.5s': round(A[k2], 2),
            'a_max_before_v0.3': round(max([A[i] for i in range(k, len(T)) if V[i] < 0.3] + [0.0]), 2)}
res['start_cmd_engage'] = start_cmd_after(res.get('t_engage'))
if res.get('mark_t') is not None:
    res['start_cmd_mark'] = start_cmd_after(res['mark_t'])

# lanelet 열 (차선변경 판정)
if '--lanes' in args:
    sys.path.insert(0, os.path.join(ROOT, 'hlfma_ws/src/hlfma/vtd_autoware_bridge'))
    import logging
    from vtd_autoware_bridge.osm_map import OsmMap
    mp = OsmMap(os.path.join(ROOT, 'map/lanelet2_map.osm'), logging.getLogger())
    seq = []
    for i in range(0, len(rows), 10):   # 0.5 s 간격, 주행 중(v>1)만 — 정지 중 겹친 lanelet 사이 흔들림 배제
        if V[i] < 1.0:
            continue
        lid = mp.nearest_lanelet(X[i], Y[i], heading=Hd[i], avoid_dead_end=False)
        if lid is not None and (not seq or seq[-1][1] != lid):
            seq.append((round(T[i], 1), lid))
    res['lanelet_seq'] = seq
    # 이웃 차선 전환 = 연속 lanelet 이 successor 관계가 아닌 경우
    lcs = []
    for (t1, a), (t2, b) in zip(seq, seq[1:]):
        if b not in mp.successors(a):
            lcs.append((t2, a, b))
    res['lane_changes'] = lcs

json.dump(res, open(os.path.join(OUT, 'metrics.json'), 'w'), indent=1, ensure_ascii=False)
for k, v in res.items():
    print(f"{k}: {v}")

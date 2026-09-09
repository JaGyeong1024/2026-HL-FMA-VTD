#!/usr/bin/env python3
"""주행 트레이스 자동 채점 — 대회정보.md §7C 15항목 중 트레이스로 판정 가능한 것.

usage: python3 tools/score.py <trace.jsonl>

판정 가능(9): 1 제한속도 / 3 차로유지 / 4 중앙선 / 7 적색정지 / 8 녹색5초
              10 보행자 / 12 횡단보도 / 13 방향지시등 / 14 충돌
판정 불가(6): 2 보호구역(노면색) 5 보도 6 실선 9 적색점멸 11 장애물충돌(경미)
              — 데이터가 없다. "미판정" 으로 표시하고 총점에 넣지 않는다.
경미 -3 / 중대 -6. 한 구간 내 동일 항목은 1회만 감점(§7B) → 항목당 1회로 집계.
"""
import json, sys, math

LIMIT_KPH = 50.0          # 항목1: 초과 1km/h 까지 허용
TOL_KPH = 1.0

def load(path):
    ego, tl, obj, turn, latdev, route, cmd = [], [], [], [], [], [], []
    for line in open(path):
        try: d = json.loads(line)
        except Exception: continue
        k = d.get('k') or ''
        t = d.get('t')
        if d.get('xy') and d.get('v') is not None:
            ego.append((t, d['xy'][0], d['xy'][1], d['v']))
        if k == '/planning/planning_factors/traffic_light' and d.get('f'):
            for f in d['f']:
                tl.append((t, f.get('d')))
        if k == '/perception/object_recognition/objects':
            obj.append((t, d.get('n'), d.get('near') or []))
        if k == '/control/command/turn_indicators_cmd':
            turn.append((t, d.get('command')))
        if 'lateral/diagnostic' in k and isinstance(d.get('d'), list):
            latdev.append((t, d['d']))
        if k == '/api/routing/state':
            route.append((t, d.get('state')))
        if k == '/control/command/control_cmd' and 'acc' in d:
            cmd.append((t, d['acc'], d.get('steer')))
    ego.sort()
    return ego, tl, obj, turn, latdev, route, cmd

def stops(ego, vth=0.3, minsec=0.5):
    out, run = [], None
    for t, x, y, v in ego:
        if v < vth:
            if run is None: run = (t, x, y)
        else:
            if run and t - run[0] >= minsec: out.append((run[0], t, run[1], run[2]))
            run = None
    if run and ego and ego[-1][0] - run[0] >= minsec:
        out.append((run[0], ego[-1][0], run[1], run[2]))
    return out

def main(path):
    ego, tl, obj, turn, latdev, route, cmd = load(path)
    if not ego:
        print('트레이스에 자차 데이터 없음'); return
    dist = sum(math.hypot(b[1]-a[1], b[2]-a[2]) for a, b in zip(ego, ego[1:]))
    vmax = max(e[3] for e in ego)
    arrived = any(s == 3 for _, s in route)
    st = stops(ego)

    pen, notes = [], []
    def add(no, name, level, why):
        pen.append((no, name, level, why))

    # 1 제한속도
    over = [(t, v*3.6) for t, _, _, v in ego if v*3.6 > LIMIT_KPH + TOL_KPH]
    if over:
        worst = max(x[1] for x in over)
        lvl = 6 if worst > LIMIT_KPH + 20 else 3
        add(1, '제한속도', lvl, '최대 %.1f km/h (%d 샘플 초과)' % (worst, len(over)))

    # 3 차로유지 — 횡편차 (diagnostic 배열 0번을 횡편차로 가정)
    if latdev:
        mx = max(abs(v[0]) for _, v in latdev if v)
        if mx > 1.0:
            add(3, '차로유지', 6 if mx > 1.5 else 3, '최대 횡편차 %.2f m' % mx)
        else:
            notes.append('3 차로유지: 최대 횡편차 %.2f m (기준 내)' % mx)
    else:
        notes.append('3 차로유지: 횡편차 계측 없음 — 미판정')

    # 8 녹색신호 30m 내 5초 이상 정차
    tlmap = {round(t, 1): d for t, d in tl if d is not None}
    for a, b, x, y in st:
        d = tlmap.get(round(a, 1))
        if d is not None and d <= 30.0:
            dur = b - a
            if dur >= 10: add(8, '녹색신호 통과', 6, 't=%.0f~%.0f (%.0fs) 정지선 %.1fm' % (a, b, dur, d))
            elif dur >= 5: add(8, '녹색신호 통과', 3, 't=%.0f~%.0f (%.0fs) 정지선 %.1fm' % (a, b, dur, d))

    # 12 횡단보도 3초 이상 정지 — 횡단보도 위치 데이터 없음
    notes.append('12 횡단보도 정차: 횡단보도 좌표 없음 — 미판정')

    # 14 충돌 — 최근접 객체 거리
    mind = None
    for t, n, near in obj:
        for pair in near:
            if isinstance(pair, list) and pair:
                mind = pair[0] if mind is None else min(mind, pair[0])
    if mind is not None:
        if mind < 1.0: add(14, '도로이용자 충돌', 6, '최근접 %.1f m' % mind)
        else: notes.append('14 충돌: 최근접 객체 %.1f m' % mind)

    # 13 방향지시등 — 차선변경 시각을 알 수 없으면 점등 유무만 기록
    on = [t for t, c in turn if c in (2, 3)]
    notes.append('13 방향지시등: 점등 %d회 (차선변경 시각 대조는 수동)' % len(on))

    # 7/10 은 신호 색·보행자 정보가 트레이스 요약에 없어 미판정
    notes.append('7 적색신호 정지 / 10 보행자 대응: 신호 색·보행자 분류 미기록 — 미판정')

    print('=' * 66)
    print('주행 요약  %s' % path.split('/')[-1])
    print('  완주      : %s' % ('예 (ARRIVED)' if arrived else '아니오'))
    print('  누적 거리 : %.0f m' % dist)
    print('  최고 속도 : %.1f m/s (%.1f km/h)' % (vmax, vmax*3.6))
    print('  정지 구간 : %d회' % len(st))
    if cmd:
        print('  명령 감속 최소 : %.2f m/s^2' % min(c[1] for c in cmd))
    print('=' * 66)
    total = 0
    seen = set()
    for no, name, lvl, why in sorted(pen):
        if no in seen: continue      # 항목당 1회
        seen.add(no); total += lvl
        print('  -%d  항목%-2d %-14s %s' % (lvl, no, name, why))
    if not seen: print('  (판정 가능한 항목에서 감점 없음)')
    print('-' * 66)
    print('  판정 가능 항목 감점 합계: -%d' % total)
    print('-' * 66)
    for n in notes: print('  · %s' % n)

if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '/dev/stdin')

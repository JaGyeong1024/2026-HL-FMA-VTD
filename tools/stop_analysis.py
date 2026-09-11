#!/usr/bin/env python3
"""적색 정지 접근 판정 — 정지선 과감속(2단 정지) 수정의 검증 도구.

usage: python3 tools/stop_analysis.py test/<판>/            (trace.jsonl 과 logs/autoware.log 사용)

출력: 적색 정지마다
  1차 정지 거리 = 차가 처음 v<0.3 이 된 순간 계획 정지점까지 남은 거리 (채점 7번: 범퍼 기준 2 m 이내가 정상 → 이 값이 2 m 미만이어야 함)
  기어감      = 1차 정지 뒤 다시 움직여 정지점에 맞췄는지
  접근 속도    = 정지점 10 m / 5 m 앞에서의 실제 속도
  최대 제동    = 그 정지 접근 중 명령 가속 최소값
합계로 '감속 너무 셈'(planning_validator) 횟수와 MRM 횟수.
"""
import json, sys, os, re

d = sys.argv[1].rstrip("/")
trace = os.path.join(d, "trace.jsonl"); aw = os.path.join(d, "logs", "autoware.log")
kin, cmd, fac, tl = [], [], [], []
for line in open(trace, errors="ignore"):
    try: r = json.loads(line)
    except Exception: continue
    k = r.get("k", ""); t = r.get("t", 0)
    if k == "/localization/kinematic_state": kin.append((t, r.get("v", 0)))
    elif k == "/control/command/control_cmd": cmd.append((t, r.get("acc", 0)))
    elif k == "/api/planning/velocity_factors":
        f = [x for x in r.get("factors", []) if x.get("behavior") == "traffic-signal"]
        fac.append((t, r.get("v", 0), f[0].get("distance") if f else None))
    elif "traffic_signals" in k:
        g = r.get("traffic_light_groups", [])
        tl.append((t, ",".join({1: "R", 2: "A", 3: "G"}.get(e["color"], "?") for grp in g for e in grp["elements"]) or "-"))

def v_at(t): return min(kin, key=lambda a: abs(a[0] - t))[1] if kin else None
def tl_at(t):
    c = "?"
    for tt, col in tl:
        if tt > t: break
        c = col
    return c

# 접근 구간: traffic-signal 정지점 거리가 30 m 아래로 내려온 뒤 처음 정지할 때까지
events = []; i = 0
while i < len(fac):
    t, v, dist = fac[i]
    if dist is not None and 0 < dist < 30 and v > 1.0:
        t0 = t; v10 = v5 = None; first_stop = None; creep = False; j = i
        while j < len(fac) and fac[j][0] - t0 < 60:
            tj, vj, dj = fac[j]
            if dj is not None:
                if v10 is None and dj <= 10: v10 = vj
                if v5 is None and dj <= 5: v5 = vj
            if first_stop is None and vj < 0.3: first_stop = (tj, dj if dj is not None else float("nan"))
            elif first_stop is not None and vj > 0.5 and (dj is None or dj > 0.3): creep = True
            if first_stop is not None and dj is not None and dj < -0.5: break
            if dj is None and first_stop is not None and fac[j][0] - first_stop[0] > 8: break
            j += 1
        if first_stop:
            acc_min = min([a for tt, a in cmd if t0 <= tt <= first_stop[0]] or [0])
            events.append((t0, tl_at(first_stop[0]), first_stop[1], creep, v10, v5, acc_min))
        i = max(j, i + 1)
    else:
        i += 1

print("적색 정지 접근 %d회 (%s)" % (len(events), d))
print("  %-8s %-4s %-14s %-8s %-10s %-10s %s" % ("t", "TL", "1차정지거리", "기어감", "v@10m", "v@5m", "최대제동"))
bad = 0
for t0, col, dfs, creep, v10, v5, am in events:
    known = dfs == dfs  # nan 이면 False (정지 순간 factor 없음: 녹색 전환·MRM 등)
    flag = "" if (not known or dfs < 2.0) else "  ← 2 m 밖"
    if known and dfs >= 2.0: bad += 1
    print("  %-8.0f %-4s %-14s %-8s %-10s %-10s %.2f%s" % (t0, col, ("%.1f m" % dfs) if known else "? (factor 없음)", "예" if creep else "아니오", "%.1f" % v10 if v10 is not None else "-", "%.1f" % v5 if v5 is not None else "-", am, flag))
inv = mrm = 0
if os.path.exists(aw):
    txt = open(aw, errors="ignore").read()
    inv = txt.count("deceleration is too high"); mrm = txt.count("EMERGENCY_STOP is operated")
print("요약: 1차 정지 2 m 밖 %d/%d(거리 확인된 것 기준), 기어감 %d회, '감속 너무 셈' %d회, 비상정지 %d회" % (bad, len(events), sum(1 for e in events if e[3]), inv, mrm))

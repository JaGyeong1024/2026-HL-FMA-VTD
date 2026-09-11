#!/usr/bin/env python3
"""주행 한 판을 그림으로. 차선 중심선 + 정지차 + 자차 궤적(속도 색) + 차선변경 지점.

usage: python3 tools/plot_run.py <로그디렉터리> [출력.png]
  로그디렉터리: ~/hlfma/logs/harness/<케이스>_<시각>  (trace.csv, mock_args.txt 를 읽는다)
"""
import sys, os, csv, math, re
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

ROOT = os.path.expanduser("~/2026-HL-FMA-VTD-JG")
MAP = os.path.join(ROOT, "map/lanelet2_map.osm")

# 이 경로 구간의 차선 사슬 (좌→우). P=좌회전 포켓
STRANDS = [
    ("P (left-turn pocket)", ["15194", "14855", "14611"], "#7b2d8e"),
    ("A (ego lane)",   ["16144", "15719", "15379", "15207", "14890", "14633"], "#c0392b"),
    ("B",           ["16249", "15750", "15414", "15220", "14925", "14655"], "#2e86c1"),
    ("C",           ["16354", "15781", "15449", "15233", "14960", "14677"], "#16a085"),
    ("D",           ["15484", "15246", "14995", "14699"], "#7f8c8d"),
]
GOAL = ["18858"]   # 좌회전 교차로


def load_map():
    nodes, ways, lls = {}, {}, {}
    for _, el in ET.iterparse(MAP, events=("end",)):
        if el.tag == "node":
            x = y = None
            for t in el.findall("tag"):
                if t.get("k") == "local_x": x = float(t.get("v"))
                elif t.get("k") == "local_y": y = float(t.get("v"))
            if x is not None: nodes[el.get("id")] = (x, y)
            el.clear()
        elif el.tag == "way":
            ways[el.get("id")] = [n.get("ref") for n in el.findall("nd")]
            el.clear()
        elif el.tag == "relation":
            tg = {t.get("k"): t.get("v") for t in el.findall("tag")}
            if tg.get("type") == "lanelet":
                L = R = None
                for m in el.findall("member"):
                    if m.get("role") == "left": L = m.get("ref")
                    elif m.get("role") == "right": R = m.get("ref")
                lls[el.get("id")] = (L, R)
            el.clear()
    return nodes, ways, lls


def centerline(nodes, ways, lls, lid):
    if lid not in lls: return []
    L, R = lls[lid]
    a = [nodes[n] for n in ways.get(L, []) if n in nodes]
    b = [nodes[n] for n in ways.get(R, []) if n in nodes]
    if not a or not b: return []
    n = max(len(a), len(b))
    return [((a[min(i, len(a)-1)][0] + b[min(i, len(b)-1)][0]) / 2,
             (a[min(i, len(a)-1)][1] + b[min(i, len(b)-1)][1]) / 2) for i in range(n)]


def read_trace(d):
    rows = list(csv.DictReader(open(os.path.join(d, "trace.csv"))))
    col = lambda *names: next(c for c in rows[0] if c.lower() in names)
    k, xs, ys, vs = col("t","time"), col("x","pos_x"), col("y","pos_y"), col("v","speed","vx")
    return [(float(r[k]), float(r[xs]), float(r[ys]), float(r[vs])) for r in rows]


def read_objects(d):
    p = os.path.join(d, "mock_args.txt")
    if not os.path.exists(p): return []
    return [(float(m[1]), float(m[2]))
            for m in re.findall(r"--obj-abs (\d+),([-\d.]+),([-\d.]+)", open(p).read())]


def frame(nodes, ways, lls):
    """A 차선 중심선을 기준축으로. 반환: (기준 폴리라인, 누적거리) — 끝점(교차로 정지선)이 s=0"""
    A = [q for l in ["16144","15719","15379","15207","14890","14633"]
         for q in centerline(nodes, ways, lls, l)]
    cum = [0.0]
    for i in range(1, len(A)):
        cum.append(cum[-1] + math.hypot(A[i][0]-A[i-1][0], A[i][1]-A[i-1][1]))
    tot = cum[-1]
    return A, [c - tot for c in cum]


def to_sd(A, sA, x, y):
    """(x,y) → (종방향 s, 횡방향 d). d>0 = 진행방향 기준 왼쪽(포켓 쪽)"""
    j = min(range(len(A)), key=lambda i: (A[i][0]-x)**2 + (A[i][1]-y)**2)
    j2 = min(j+1, len(A)-1); j1 = max(j-1, 0)
    tx, ty = A[j2][0]-A[j1][0], A[j2][1]-A[j1][1]
    n = math.hypot(tx, ty) or 1.0
    tx, ty = tx/n, ty/n
    return sA[j], -(x-A[j][0])*ty + (y-A[j][1])*tx


def main():
    d = sys.argv[1].rstrip("/")
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(d, "run.png")
    nodes, ways, lls = load_map()
    tr = read_trace(d); objs = read_objects(d)
    A, sA = frame(nodes, ways, lls)

    import numpy as np
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 9),
                                   gridspec_kw={"height_ratios": [2, 1]})

    # ── 상단: 종방향-횡방향으로 편 그림 ──────────────────────────────
    for label, ids, color in STRANDS:
        pts = [q for l in ids for q in centerline(nodes, ways, lls, l)]
        if not pts: continue
        sd = [to_sd(A, sA, *q) for q in pts]
        sd = [(s_, d_) for s_, d_ in sd if -230 <= s_ <= 5]
        if not sd: continue
        sd.sort()
        ax1.plot([q[0] for q in sd], [q[1] for q in sd], color=color, lw=1.4, ls="--", alpha=.8)
        ax1.annotate(label, (sd[len(sd)//2][0], sd[len(sd)//2][1] + .35), color=color,
                     fontsize=9, fontweight="bold", ha="center")

    if objs:
        osd = [to_sd(A, sA, *q) for q in objs]
        ax1.scatter([q[0] for q in osd], [q[1] for q in osd], s=230, marker="s",
                    c="#e74c3c", edgecolors="black", lw=1.3, zorder=5,
                    label=f"stopped cars ({len(objs)})")
        for (s_, d_) in osd:
            ax1.annotate(f"{s_:.0f}m", (s_, d_-.9), fontsize=7.5, ha="center", color="#922")

    tsd = [to_sd(A, sA, x, y) for _, x, y, _ in tr]
    seg = [[tsd[i], tsd[i+1]] for i in range(len(tsd)-1)]
    spd = np.array([tr[i][3]*3.6 for i in range(len(tr)-1)])
    lc = LineCollection(seg, cmap="viridis", lw=3.4, zorder=4)
    lc.set_array(spd); lc.set_clim(0, 55)
    ax1.add_collection(lc)
    fig.colorbar(lc, ax=ax1, label="ego speed [km/h]", shrink=.9, pad=.01)

    ax1.scatter([tsd[0][0]], [tsd[0][1]], s=120, marker="o", c="white",
                edgecolors="black", lw=1.6, zorder=7, label="start")
    ax1.scatter([tsd[-1][0]], [tsd[-1][1]], s=210, marker="X", c="black",
                zorder=7, label="final stop")
    ax1.axvline(0, color="#444", lw=1.6, ls="-", alpha=.8)
    ax1.annotate("stop line", (0, 4.2), fontsize=9, ha="center", color="#444")

    nxt = 0.0
    for (t, x, y, v), (s_, d_) in zip(tr, tsd):
        if t >= nxt:
            ax1.annotate(f"{t:.0f}s", (s_, d_+.45), fontsize=7.5, ha="center", color="#333", zorder=8)
            nxt += 10.0

    ax1.set_xlim(-230, 12); ax1.set_ylim(-12, 6)
    ax1.set_ylabel("lateral offset from lane A [m]   (+ = left)")
    ax1.grid(alpha=.25); ax1.legend(loc="upper left", fontsize=9)
    ax1.set_title(f"{os.path.basename(d)}   —   avoid stopped cars, return to lane "
                  f"(pocket NOT entered)", fontsize=11)

    # ── 하단: 속도 ────────────────────────────────────────────────
    ax2.plot([q[0] for q in tsd], [p[3]*3.6 for p in tr], color="#2c3e50", lw=1.8)
    ax2.axhline(50, color="#e74c3c", lw=1.1, ls=":", label="50 km/h limit")
    ax2.axvline(0, color="#444", lw=1.6, alpha=.8)
    if objs:
        for (s_, _) in [to_sd(A, sA, *q) for q in objs]:
            ax2.axvline(s_, color="#e74c3c", lw=.9, alpha=.35)
    ax2.set_xlim(-230, 12); ax2.set_ylim(0, 60)
    ax2.set_xlabel("longitudinal distance along lane A [m]   (0 = intersection stop line)")
    ax2.set_ylabel("speed [km/h]"); ax2.grid(alpha=.25); ax2.legend(loc="upper left", fontsize=9)

    fig.tight_layout(); fig.savefig(out, dpi=140)
    print(f"저장: {out}")


if __name__ == "__main__":
    main()

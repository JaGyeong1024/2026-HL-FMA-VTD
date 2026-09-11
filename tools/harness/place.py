#!/usr/bin/env python3
"""경로 CSV 기준 객체 배치 헬퍼 (mock_vtd.py --obj-abs 용) + 시작 pose·정지선 거리.

usage:
  place.py start   <route.csv>                    → mock 시작 인자  --x X --y Y --hdg H --z Z
  place.py obj     <route.csv> ID D L [SPEED LEN WID]   → --obj-abs 값. D=시작점에서 차선을 따라 전방 m, L=좌측 m(우측은 음수)
                   환경변수 ROUTE_CHAIN="16249,15750,…" 이 있으면 successor 대신 그 lanelet 열을 따라감(자차 경로 차선에 배치)
  place.py chain-stopline <route.csv>            → ROUTE_CHAIN 을 따라 첫 정지선까지 거리
  place.py stopline <route.csv>                   → 시작점에서 첫 신호등 정지선까지 차선 거리 [m] 와 lanelet
  place.py lane    <route.csv> D                  → D 지점의 lanelet, 폭, 이웃(좌/우) 정보
차선 기하는 Autoware 가 읽는 맵(map/lanelet2_map.osm)의 중심선을 따른다 (직선 가정 아님).
"""
import sys, os, math, logging, csv
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'hlfma_ws/src/hlfma/vtd_autoware_bridge'))
from vtd_autoware_bridge.osm_map import OsmMap  # noqa: E402
MAP = os.path.join(ROOT, 'map/lanelet2_map.osm')


def load_route(path):
    pts = []
    for row in csv.reader(open(path)):
        if row and row[0].strip().isdigit():
            pts.append((float(row[1]), float(row[2])))
    return pts


def start_lanelet(m, pts):
    (x, y), (nx, ny) = pts[0], pts[1]
    hint = math.atan2(ny - y, nx - x)
    c = m.match_candidates(x, y, [hint], 8.0, need_pred=False, need_succ=True)
    lid = min(c, key=lambda t: t[2]['d'])[0]
    d, s, h = m.project(lid, x, y)
    return lid, s, h


def along(m, lid, s, D):
    """시작 lanelet 의 s 에서 차선을 따라 D m 앞 점. ROUTE_CHAIN 이 있으면 그 열을(이웃 차선 포함) 따라, 없으면 successor.
    → (x,y,heading,lanelet)"""
    chain = os.environ.get('ROUTE_CHAIN')
    if not chain:
        return m.point_along(lid, s, D)
    ids = [int(v) for v in chain.split(',')]
    rem = D + s
    for l in ids:
        L = m.lanelets[l].length
        if rem <= L:
            x, y, h = m._interp(m.lanelets[l], rem)
            return x, y, h, l
        rem -= L
    x, y, h = m._interp(m.lanelets[ids[-1]], m.lanelets[ids[-1]].length - 0.5)
    return x, y, h, ids[-1]


def main():
    a = sys.argv[1:]
    if not a:
        print(__doc__); sys.exit(1)
    m = OsmMap(MAP, logging.getLogger())
    cmd, route = a[0], a[1]
    pts = load_route(route)
    lid, s, h = start_lanelet(m, pts)
    x0, y0 = pts[0]
    if cmd == 'start':
        print(f"--x {x0:.3f} --y {y0:.3f} --hdg {h:.4f} --z {m.elevation(lid, x0, y0):.2f}")
    elif cmd == 'obj':
        oid, D, L = int(a[2]), float(a[3]), float(a[4])
        speed = float(a[5]) if len(a) > 5 else 0.0
        ln = float(a[6]) if len(a) > 6 else 4.5
        wd = float(a[7]) if len(a) > 7 else 1.8
        px, py, ph, pl = along(m, lid, s, D)
        ox, oy = px - L * math.sin(ph), py + L * math.cos(ph)   # 좌측(+L) = 진행방향 왼쪽
        print(f"{oid},{ox:.3f},{oy:.3f},{m.elevation(pl, ox, oy):.2f},{ph:.4f},{speed},{ln},{wd}")
    elif cmd == 'stopline':
        acc, cur = -s, lid
        for _ in range(60):
            sl = m.stop_line_s(cur)
            if sl is not None:
                print(f"{acc + sl:.1f} {cur}"); return
            acc += m.lanelets[cur].length
            succ = m.successors(cur)
            if not succ:
                print("none"); return
            cur = succ[0]
        print("none")
    elif cmd == 'chain-stopline':
        ids = [int(v) for v in os.environ['ROUTE_CHAIN'].split(',')]
        acc = -s
        for l in ids:
            sl = m.stop_line_s(l)
            if sl is not None:
                print(f"{acc + sl:.1f} {l}"); return
            acc += m.lanelets[l].length
        print("none")
    elif cmd == 'lane':
        D = float(a[2])
        px, py, ph, pl = along(m, lid, s, D)
        c = m.match_candidates(px, py, [ph], 8.0, need_pred=False, need_succ=False)
        print(f"D={D} at ({px:.1f},{py:.1f}) hdg={ph:.3f} lanelet={pl} w={m.width_at(pl, px, py):.1f}")
        for l2, pen, inf in c:
            side = 'L' if inf['d'] > 0.5 else ('R' if inf['d'] < -0.5 else 'C')
            print(f"  {l2} d={inf['d']:+.2f} ({side}) dh={inf['dh_deg']:.0f}deg w={inf['w']:.1f}")
    else:
        print(__doc__); sys.exit(1)


if __name__ == '__main__':
    main()

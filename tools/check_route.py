"""대회 당일 경로 CSV 즉시 점검 — 받자마자 실행하는 원커맨드.

검사 항목:
  1. waypoint가 차선 위에 스냅되는지 (+ 어느 차선인지 → 회전 차선 힌트)
  2. 알려진 xodr 결함 지대와의 최근접 거리 (경로가 지나는지)
  3. lane_graph 글로벌 패스 생성 성공 여부 + 길이
  4. lanelet2 라우팅 성공 여부 (Autoware 스택용)
  5. 경로상 신호등(정지선) 목록 — 교차로 짝과 대응 확인용

usage: .venv/bin/python check_route.py <route.csv> [--no-lanelet2]
"""
import sys
import csv
from pathlib import Path
import numpy as np

# 스크립트와 같은 디렉터리의 xodr을 우선 사용 (제어기 PC tools/에는 이것만 있음),
# 없으면 노트북 원본 경로로 폴백.
_HERE = Path(__file__).parent
_LOCAL_XODR = _HERE / "HL_FMA_VTD_LivingLab.xodr"
XODR = str(_LOCAL_XODR) if _LOCAL_XODR.exists() else "/home/a/HL_FMA/실습파일/HL_FMA_VTD_LivingLab.xodr"
OSM = str(_HERE / "out/livinglab_lanelet2_native.osm")

# 익스포터 감사에서 나온 xodr 원본 불연속 지점 (대회정보.md §13)
DEFECT_ZONES = [  # (x, y, 크기m, 설명)
    (1064, -881, 3.3, "road 1927 차선폭 즉시 점프 (터널 부근)"),
    (966, -539, 0.17, "road 2190→2172 경계"),
    (1027, 74, 0.15, "road 2625 섹션 경계"),
    (905, 188, 0.10, "road 173→1225 경계"),
]


def main():
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    path = sys.argv[1]
    use_l2 = "--no-lanelet2" not in sys.argv

    wps = [(float(r["x"]), float(r["y"])) for r in csv.DictReader(open(path))]
    print(f"waypoint {len(wps)}개 (시작 1 + 교차로 {(len(wps)-2)//2}쌍 + 종료 1)")
    if (len(wps) - 2) % 2 != 0:
        print("⚠ 중간 지점이 짝수가 아님 — 형식 확인 필요!")

    from xodr_map import OpenDriveMap
    from lane_graph import LaneGraph, build_route
    m = OpenDriveMap(XODR)
    g = LaneGraph(m)

    # 1) waypoint 스냅
    print("\n[1] waypoint → 차선 스냅")
    ok = True
    for i, (x, y) in enumerate(wps, start=1):
        c = g.nearest_nodes(x, y, k=3, max_dist=6.0)
        if not c:
            print(f"  P{i} ({x:.1f},{y:.1f}): ⚠ 6m 내 driving 차선 없음!")
            ok = False
        else:
            key, s, d = c[0]
            road_id, si, lane_id = key
            print(f"  P{i} ({x:.1f},{y:.1f}): road {road_id} lane {lane_id} (오차 {d:.2f}m)"
                  + ("  ← 1차선(좌회전 가능성)" if abs(lane_id) == 1 else ""))

    # 2) 결함 지대
    print("\n[2] 알려진 결함 지대 근접도")
    route = None
    try:
        route, node_seq = build_route(g, wps)
    except Exception as e:
        print(f"  (경로 생성 실패로 waypoint 기준으로만 검사: {e})")
    pts = route if route is not None else np.array(wps)
    for dx, dy, size, desc in DEFECT_ZONES:
        d = float(np.min(np.hypot(pts[:, 0] - dx, pts[:, 1] - dy)))
        flag = "🚨 경로가 통과!" if d < 10 else ("⚠ 부근 통과" if d < 50 else "안전")
        print(f"  ({dx},{dy}) {size}m {desc}: 최근접 {d:.0f}m — {flag}")

    # 3) 글로벌 패스
    print("\n[3] lane_graph 글로벌 패스")
    if route is not None:
        seg = np.hypot(*np.diff(route, axis=0).T)
        print(f"  성공: {seg.sum():.0f}m, 점 {len(route)}개")
    else:
        print("  ⚠ 실패 — 수동 확인 필요")
        ok = False

    # 4) lanelet2 라우팅
    if use_l2:
        print("\n[4] lanelet2 라우팅 (Autoware)")
        try:
            import lanelet2
            from lanelet2.projection import UtmProjector
            from lanelet2.io import Origin
            from lanelet2 import traffic_rules, routing
            lmap, errs = lanelet2.io.loadRobust(OSM, UtmProjector(Origin(37.2, 126.8)))
            tr = traffic_rules.create(traffic_rules.Locations.Germany,
                                      traffic_rules.Participants.Vehicle)
            graph = routing.RoutingGraph(lmap, tr)

            def local_pts(ls):
                return np.array([[float(p.attributes["local_x"]),
                                  float(p.attributes["local_y"])] for p in ls])
            lls, centers = [], []
            for ll in lmap.laneletLayer:
                lb, rb = local_pts(ll.leftBound), local_pts(ll.rightBound)
                n = max(len(lb), len(rb))
                def rs(a, n):
                    dd = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(a, axis=0).T))])
                    u = np.linspace(0, dd[-1], n)
                    return np.column_stack([np.interp(u, dd, a[:, 0]),
                                            np.interp(u, dd, a[:, 1])])
                lls.append(ll); centers.append((rs(lb, n) + rs(rb, n)) / 2)

            def cands(x, y, k=4):
                ds = sorted(((float(np.min(np.hypot(c[:, 0] - x, c[:, 1] - y))), i)
                             for i, c in enumerate(centers)))
                return [lls[i] for d, i in ds[:k] if d < 6.0]

            prev = cands(*wps[0])
            for i, (x, y) in enumerate(wps[1:], start=2):
                cur, hit = cands(x, y), None
                for p in prev:
                    for c in cur:
                        if graph.getRoute(p, c) is not None:
                            hit = c; break
                    if hit: break
                if hit is None:
                    print(f"  ⚠ P{i-1}→P{i} 라우팅 실패!")
                    ok = False
                    prev = cur
                else:
                    prev = [hit]
            else:
                print("  전 구간 라우팅 성공" if ok else "  (위 실패 구간 확인)")
        except ImportError:
            print("  (lanelet2 미설치 — 건너뜀)")

    # 5) 경로상 신호등
    print("\n[5] 경로 부근 신호등 (반경 20m)")
    if route is not None:
        from scipy.spatial import cKDTree
        tree = cKDTree(route)
        seen = set()
        sigs = []
        for r in m.roads.values():
            for sig in r.signals:
                if sig.dynamic and (sig.road_id, sig.id) not in seen:
                    d, j = tree.query([sig.x, sig.y])
                    if d < 20:
                        sigs.append((float(np.hypot(*(route[j] - route[0]))), sig, d))
                        seen.add((sig.road_id, sig.id))
        sigs.sort(key=lambda t: t[0])
        for _, sig, d in sigs:
            print(f"  road {sig.road_id} signal id={sig.id} at ({sig.x:.0f},{sig.y:.0f}) 경로거리 {d:.1f}m")
        print(f"  총 {len(sigs)}개 — 교차로 쌍 수({(len(wps)-2)//2})와 대응 확인")

    print("\n결론:", "✅ 이상 없음" if ok else "🚨 위 경고 확인 필요")


if __name__ == "__main__":
    main()

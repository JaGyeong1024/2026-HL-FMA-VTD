"""VTD 시나리오 XML의 Path(웨이포인트)를 경로 CSV(seq,x,y)로 변환.

usage: .venv/bin/python extract_scenario_route.py <scenario.xml> [PathId] [out.csv]

Waypoint(TrackId, s) → xodr 기준선 ref_at(s) 좌표. t=0(도로 기준선)이므로
차선 스냅은 lane_graph.build_route가 처리한다 (route CSV와 동일한 사용법).
"""
import csv
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from xodr_map import OpenDriveMap

XODR = Path("/home/a/HL_FMA/실습파일/HL_FMA_VTD_LivingLab.xodr")

scen = Path(sys.argv[1])
path_id = sys.argv[2] if len(sys.argv) > 2 else "1"
out = Path(sys.argv[3]) if len(sys.argv) > 3 else Path(f"real_route_path{path_id}.csv")

root = ET.parse(scen).getroot()
wps = None
for p in root.iter("Path"):
    if p.get("PathId") == path_id:
        wps = [(w.get("TrackId"), float(w.get("s"))) for w in p.findall("Waypoint")]
        break
if not wps:
    raise SystemExit(f"PathId={path_id} 없음")

m = OpenDriveMap(XODR)
roads = {str(r.id): r for r in m.roads.values()} if hasattr(m, "roads") else None
if roads is None:
    raise SystemExit("OpenDriveMap.roads 접근 실패")

rows = []
for i, (tid, s) in enumerate(wps, 1):
    r = roads.get(tid)
    if r is None:
        raise SystemExit(f"road {tid} 없음")
    x, y, _ = r.ref_at(s)
    rows.append((i, float(x), float(y)))
    print(f"{i:2d}: road {tid:>5} s={s:8.2f} → ({x:9.2f}, {y:9.2f})")

with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["seq", "x", "y"])
    w.writerows(rows)
print(f"\n저장: {out} ({len(rows)}점)")

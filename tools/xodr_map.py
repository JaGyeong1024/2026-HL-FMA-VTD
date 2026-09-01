"""OpenDRIVE(.xodr) 파서 — HL FMA LivingLab 맵용 세계 모델의 정적 계층.

지원: planView(line/arc/spiral/poly3), laneOffset, laneSection(left/right),
lane width 다항식, roadMark, signal, road link.
spiral은 수치 적분(내부 0.1m 스텝)으로 계산 — Fresnel 특수처리 불필요.
"""
from dataclasses import dataclass, field
from lxml import etree
import numpy as np

INTERNAL_STEP = 0.1  # 기하 수치적분 내부 스텝 [m]


@dataclass
class RoadMark:
    s_offset: float  # laneSection 기준
    type: str        # solid / broken / none / solid solid ...
    color: str = "standard"       # standard(백색) / yellow(중앙선)
    lane_change: str = "both"     # both / none — 이 선을 넘는 차로변경 허용 여부


@dataclass
class Width:
    s_offset: float
    a: float; b: float; c: float; d: float

    def eval(self, ds):
        x = ds - self.s_offset
        return self.a + self.b * x + self.c * x * x + self.d * x ** 3


@dataclass
class Lane:
    id: int
    type: str
    widths: list = field(default_factory=list)
    road_marks: list = field(default_factory=list)
    pred: int | None = None
    succ: int | None = None

    def width_at(self, ds):
        w = None
        for rec in self.widths:
            if rec.s_offset <= ds + 1e-9:
                w = rec
        return max(0.0, w.eval(ds)) if w else 0.0

    def mark_at(self, ds):
        m = "none"
        for rec in self.road_marks:
            if rec.s_offset <= ds + 1e-9:
                m = rec.type
        return m

    def mark_rec_at(self, ds):
        rec = None
        for r in self.road_marks:
            if r.s_offset <= ds + 1e-9:
                rec = r
        return rec


@dataclass
class LaneSection:
    s: float
    s_end: float
    left: list = field(default_factory=list)   # id 오름차순(1,2,..)
    right: list = field(default_factory=list)  # id 내림차순(-1,-2,..)
    center: 'Lane' = None                      # id 0 — roadMark(중앙선)만 의미 있음


@dataclass
class Signal:
    id: int
    road_id: int
    s: float
    t: float
    dynamic: bool
    type: str
    x: float = 0.0
    y: float = 0.0
    orientation: str = "+"   # '+' = +s 주행 차량 대상(우측 차선), '-' = -s
    z_offset: float = 0.0


@dataclass
class Road:
    id: int
    length: float
    junction: int
    link: dict = field(default_factory=dict)  # {'predecessor': (type,id), 'successor': ...}
    ref_s: np.ndarray = None   # 기준선 station
    ref_x: np.ndarray = None
    ref_y: np.ndarray = None
    ref_hdg: np.ndarray = None
    lane_offset: list = field(default_factory=list)  # (s, a,b,c,d) 절대 s
    sections: list = field(default_factory=list)
    signals: list = field(default_factory=list)

    def lane_offset_at(self, s):
        rec = None
        for r in self.lane_offset:
            if r[0] <= s + 1e-9:
                rec = r
        if rec is None:
            return 0.0
        x = s - rec[0]
        return rec[1] + rec[2] * x + rec[3] * x * x + rec[4] * x ** 3

    def ref_at(self, s):
        """기준선 위 (x, y, hdg) 보간."""
        s = np.clip(s, self.ref_s[0], self.ref_s[-1])
        x = np.interp(s, self.ref_s, self.ref_x)
        y = np.interp(s, self.ref_s, self.ref_y)
        # heading은 unwrap된 배열로 보간
        h = np.interp(s, self.ref_s, self.ref_hdg)
        return x, y, h


def _sample_geometry(g, step=INTERNAL_STEP):
    """geometry 요소 하나 → 로컬 (s_local, x, y, hdg) 배열."""
    s0 = float(g.get("s")); x0 = float(g.get("x")); y0 = float(g.get("y"))
    hdg = float(g.get("hdg")); L = float(g.get("length"))
    n = max(2, int(np.ceil(L / step)) + 1)
    sl = np.linspace(0.0, L, n)
    child = g[0] if len(g) else None
    tag = child.tag if child is not None else "line"

    if tag == "line":
        x = x0 + sl * np.cos(hdg)
        y = y0 + sl * np.sin(hdg)
        h = np.full_like(sl, hdg)
    elif tag == "arc":
        k = float(child.get("curvature"))
        h = hdg + k * sl
        x = x0 + (np.sin(h) - np.sin(hdg)) / k
        y = y0 - (np.cos(h) - np.cos(hdg)) / k
    elif tag == "spiral":
        k0 = float(child.get("curvStart")); k1 = float(child.get("curvEnd"))
        cdot = (k1 - k0) / L if L > 0 else 0.0
        h = hdg + k0 * sl + 0.5 * cdot * sl ** 2
        # 수치 적분 (누적 사다리꼴)
        dx = np.cos(h); dy = np.sin(h)
        ds = np.diff(sl)
        x = x0 + np.concatenate([[0], np.cumsum(0.5 * (dx[1:] + dx[:-1]) * ds)])
        y = y0 + np.concatenate([[0], np.cumsum(0.5 * (dy[1:] + dy[:-1]) * ds)])
    elif tag == "poly3":
        a = float(child.get("a")); b = float(child.get("b"))
        c = float(child.get("c")); d = float(child.get("d"))
        # 로컬 u를 밀도 있게 샘플 → 호길이 재매개화
        u = np.linspace(0.0, L * 1.5, n * 2)
        v = a + b * u + c * u ** 2 + d * u ** 3
        xg = x0 + u * np.cos(hdg) - v * np.sin(hdg)
        yg = y0 + u * np.sin(hdg) + v * np.cos(hdg)
        seg = np.concatenate([[0], np.cumsum(np.hypot(np.diff(xg), np.diff(yg)))])
        x = np.interp(sl, seg, xg)
        y = np.interp(sl, seg, yg)
        dv = b + 2 * c * u + 3 * d * u ** 2
        hg = hdg + np.arctan(dv)  # 근사: u축 대비 기울기
        h = np.interp(sl, seg, hg)
    else:
        raise ValueError(f"unsupported geometry: {tag}")
    return s0 + sl, x, y, h


class OpenDriveMap:
    def __init__(self, path):
        self.roads: dict[int, Road] = {}
        self.junctions: dict[int, list] = {}  # junction_id -> [(incoming, connecting, contact)]
        self._parse(path)

    def _parse(self, path):
        root = etree.parse(str(path)).getroot()
        for jel in root.findall("junction"):
            conns = []
            for c in jel.findall("connection"):
                lane_links = [(int(l.get("from")), int(l.get("to")))
                              for l in c.findall("laneLink")]
                conns.append((int(c.get("incomingRoad")), int(c.get("connectingRoad")),
                              c.get("contactPoint"), lane_links))
            self.junctions[int(jel.get("id"))] = conns

        for rel in root.findall("road"):
            road = Road(id=int(rel.get("id")), length=float(rel.get("length")),
                        junction=int(rel.get("junction")))
            link = rel.find("link")
            if link is not None:
                for le in link:
                    road.link[le.tag] = (le.get("elementType"), int(le.get("elementId")),
                                         le.get("contactPoint"))

            # 기준선
            segs = [_sample_geometry(g) for g in rel.find("planView")]
            road.ref_s = np.concatenate([s[0] for s in segs])
            road.ref_x = np.concatenate([s[1] for s in segs])
            road.ref_y = np.concatenate([s[2] for s in segs])
            road.ref_hdg = np.unwrap(np.concatenate([s[3] for s in segs]))

            lanes_el = rel.find("lanes")
            for off in lanes_el.findall("laneOffset"):
                road.lane_offset.append(tuple(float(off.get(k)) for k in "sabcd"))

            secs = lanes_el.findall("laneSection")
            for i, sec_el in enumerate(secs):
                s = float(sec_el.get("s"))
                s_end = float(secs[i + 1].get("s")) if i + 1 < len(secs) else road.length
                sec = LaneSection(s=s, s_end=s_end)
                center_el = sec_el.find("center")
                if center_el is not None:
                    clel = center_el.find("lane")
                    if clel is not None:
                        sec.center = Lane(id=0, type="none")
                        for m in clel.findall("roadMark"):
                            sec.center.road_marks.append(RoadMark(
                                float(m.get("sOffset")), m.get("type") or "none",
                                m.get("color") or "standard", m.get("laneChange") or "both"))
                for side, target, order in (("left", sec.left, 1), ("right", sec.right, -1)):
                    side_el = sec_el.find(side)
                    if side_el is None:
                        continue
                    lanes = []
                    for lel in side_el.findall("lane"):
                        lane = Lane(id=int(lel.get("id")), type=lel.get("type"))
                        for w in lel.findall("width"):
                            lane.widths.append(Width(float(w.get("sOffset")),
                                *(float(w.get(k)) for k in "abcd")))
                        for m in lel.findall("roadMark"):
                            lane.road_marks.append(RoadMark(
                                float(m.get("sOffset")), m.get("type") or "none",
                                m.get("color") or "standard", m.get("laneChange") or "both"))
                        ln = lel.find("link")
                        if ln is not None:
                            p = ln.find("predecessor"); sc = ln.find("successor")
                            lane.pred = int(p.get("id")) if p is not None else None
                            lane.succ = int(sc.get("id")) if sc is not None else None
                        lanes.append(lane)
                    lanes.sort(key=lambda l: abs(l.id))  # 중심에서 바깥 순
                    target.extend(lanes)
                road.sections.append(sec)

            # laneOffset 정규화 — 저작 도구 오차 보정 (VTD가 보는 연속 기하에 맞춤)
            # ① 섹션 경계와 0.05m 이내로 어긋난 레코드 s는 경계에 스냅
            #    (예: road 429, 섹션 s=92.00 vs 레코드 s=92.01 → 1cm 틈에서 가짜 3.8m 불연속)
            # ② s 오름차순 정렬(원본에 비정렬 road 13개) ③ 같은 s 중복은 뒤 레코드 우선
            if road.lane_offset:
                bounds = [s.s for s in road.sections] + [road.length]
                snapped = []
                for rec in road.lane_offset:
                    s0 = rec[0]
                    for b in bounds:
                        if 0 < abs(s0 - b) <= 0.05:
                            s0 = b
                            break
                    snapped.append((s0,) + rec[1:])
                snapped.sort(key=lambda r: r[0])  # 안정 정렬 → 같은 s는 파일 순서 유지
                dedup = {}
                for rec in snapped:
                    dedup[round(rec[0], 6)] = rec  # 같은 s는 마지막 레코드가 승리
                road.lane_offset = sorted(dedup.values(), key=lambda r: r[0])

            sig_el = rel.find("signals")
            if sig_el is not None:
                for s_el in sig_el.findall("signal"):
                    sig = Signal(id=int(s_el.get("id")), road_id=road.id,
                                 s=float(s_el.get("s")), t=float(s_el.get("t")),
                                 dynamic=s_el.get("dynamic") == "yes",
                                 type=s_el.get("type") or "",
                                 orientation=s_el.get("orientation") or "+",
                                 z_offset=float(s_el.get("zOffset") or 0.0))
                    x, y, h = road.ref_at(sig.s)
                    sig.x = x - sig.t * np.sin(h)   # t>0 = 왼쪽
                    sig.y = y + sig.t * np.cos(h)
                    road.signals.append(sig)

            self.roads[road.id] = road

    # ---- 차선 기하 ----
    def lane_polylines(self, road: Road, sec: LaneSection, step=1.0):
        """섹션의 각 차선 → (lane, centerline Nx2, outer_boundary Nx2)."""
        n = max(2, int(np.ceil((sec.s_end - sec.s) / step)) + 1)
        ss = np.linspace(sec.s, max(sec.s, sec.s_end - 1e-3), n)
        xs = np.interp(ss, road.ref_s, road.ref_x)
        ys = np.interp(ss, road.ref_s, road.ref_y)
        hs = np.interp(ss, road.ref_s, road.ref_hdg)
        nx, ny = -np.sin(hs), np.cos(hs)  # 왼쪽 법선
        t0 = np.array([road.lane_offset_at(s) for s in ss])

        out = []
        for lanes, sign in ((sec.left, 1.0), (sec.right, -1.0)):
            t_inner = t0.copy()
            for lane in lanes:
                w = np.array([lane.width_at(s - sec.s) for s in ss])
                t_outer = t_inner + sign * w
                t_center = 0.5 * (t_inner + t_outer)
                center = np.column_stack([xs + t_center * nx, ys + t_center * ny])
                outer = np.column_stack([xs + t_outer * nx, ys + t_outer * ny])
                out.append((lane, center, outer, ss))
                t_inner = t_outer
        return out

    def all_lane_polylines(self, types=("driving",), step=1.0):
        for road in self.roads.values():
            for sec in road.sections:
                for lane, center, outer, ss in self.lane_polylines(road, sec, step):
                    if types is None or lane.type in types:
                        yield road, sec, lane, center, outer, ss

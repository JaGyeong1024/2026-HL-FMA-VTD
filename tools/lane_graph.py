"""차선 연결 그래프 + 경로 생성.

노드 = (road_id, section_idx, lane_id) 단위의 '주행 방향 차선'.
RHT 기준 right(-) 차선은 +s, left(+) 차선은 -s 방향으로 주행.
경로: waypoint 열 → 후보 차선 매칭 → 순차 Dijkstra → 조밀 중심선 폴리라인.
"""
import heapq
from dataclasses import dataclass, field

import numpy as np

from xodr_map import OpenDriveMap

LANE_CHANGE_COST = 30.0  # [m] 차로변경 페널티


@dataclass
class LaneNode:
    key: tuple  # (road_id, sec_idx, lane_id)
    pts: np.ndarray      # 주행 방향 순 중심선 Nx2
    s: np.ndarray        # 폴리라인 누적거리 (0..length)
    length: float
    marks: list = field(default_factory=list)  # 진행방향 기준 (구간 시작 s비율, 바깥경계 roadMark)
    edges: list = field(default_factory=list)  # (next_key, cost, kind) kind: 'succ'|'lc'


class LaneGraph:
    def __init__(self, m: OpenDriveMap, step=1.0):
        self.m = m
        self.nodes: dict[tuple, LaneNode] = {}
        self._build(step)

    def _build(self, step):
        # 1) 노드 생성 (driving 차선만)
        for road in self.m.roads.values():
            for si, sec in enumerate(road.sections):
                for lane, center, outer, ss in self.m.lane_polylines(road, sec, step):
                    if lane.type != "driving":
                        continue
                    pts = center if lane.id < 0 else center[::-1]  # 좌측 차선은 -s 주행
                    d = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
                    self.nodes[(road.id, si, lane.id)] = LaneNode(
                        key=(road.id, si, lane.id), pts=pts, s=d, length=d[-1])

        # 2) 도로 내부 섹션 연결
        for road in self.m.roads.values():
            for si, sec in enumerate(road.sections):
                for lane in sec.right + sec.left:
                    key = (road.id, si, lane.id)
                    if key not in self.nodes:
                        continue
                    node = self.nodes[key]
                    if lane.id < 0:  # +s 주행 → 다음 섹션의 succ
                        if si + 1 < len(road.sections) and lane.succ is not None:
                            nk = (road.id, si + 1, lane.succ)
                            if nk in self.nodes:
                                node.edges.append((nk, node.length, "succ"))
                    else:            # -s 주행 → 이전 섹션의 pred
                        if si - 1 >= 0 and lane.pred is not None:
                            nk = (road.id, si - 1, lane.pred)
                            if nk in self.nodes:
                                node.edges.append((nk, node.length, "succ"))
                    # 차로변경: 같은 섹션, 같은 부호, |id|±1
                    for other in sec.right + sec.left:
                        if other.type == "driving" and other.id * lane.id > 0 \
                           and abs(other.id - lane.id) == 1:
                            ok = (road.id, si, other.id)
                            if ok in self.nodes:
                                node.edges.append((ok, LANE_CHANGE_COST, "lc"))

        # 3) 도로 경계 연결 (successor/predecessor + junction)
        for road in self.m.roads.values():
            last = len(road.sections) - 1
            for si, sec in enumerate(road.sections):
                for lane in sec.right + sec.left:
                    key = (road.id, si, lane.id)
                    if key not in self.nodes:
                        continue
                    node = self.nodes[key]
                    exit_end = (lane.id < 0 and si == last) or (lane.id > 0 and si == 0)
                    if not exit_end:
                        continue
                    link_tag = "successor" if lane.id < 0 else "predecessor"
                    link = road.link.get(link_tag)
                    lane_link = lane.succ if lane.id < 0 else lane.pred
                    if link is None:
                        continue
                    etype, eid, contact = link
                    if etype == "road":
                        if lane_link is None:
                            continue
                        self._connect(node, eid, contact, lane_link)
                    elif etype == "junction":
                        for inc, conn_road, conn_contact, lls in self.m.junctions.get(eid, []):
                            if inc != road.id:
                                continue
                            for frm, to in lls:
                                if frm == lane.id:
                                    self._connect(node, conn_road, conn_contact, to)

    def _connect(self, node, next_road_id, contact, next_lane_id):
        nroad = self.m.roads.get(next_road_id)
        if nroad is None:
            return
        # contact start → 섹션0에서 +s 주행(우측차선) 또는 첫 섹션의 해당 차선
        # contact end → 마지막 섹션에서 -s 주행(좌측차선)
        si = 0 if contact == "start" else len(nroad.sections) - 1
        nk = (next_road_id, si, next_lane_id)
        if nk in self.nodes:
            node.edges.append((nk, node.length, "succ"))

    # ---- 질의 ----
    def nearest_nodes(self, x, y, k=6, max_dist=6.0):
        """(x,y)에 가까운 후보 (key, 투영 s, 거리) 목록."""
        cands = []
        for key, node in self.nodes.items():
            d2 = np.hypot(node.pts[:, 0] - x, node.pts[:, 1] - y)
            i = int(np.argmin(d2))
            if d2[i] <= max_dist:
                cands.append((d2[i], key, node.s[i]))
        cands.sort()
        return [(key, s, d) for d, key, s in cands[:k]]

    def shortest_path(self, start_keys, goal_keys):
        """start 후보들 → goal 후보들 최단 노드 열. start/goal: {key: 진입비용}."""
        dist = dict(start_keys)
        prev = {}
        pq = [(c, k) for k, c in start_keys.items()]
        heapq.heapify(pq)
        goal_set = set(goal_keys)
        best_goal, best_cost = None, np.inf
        while pq:
            d, key = heapq.heappop(pq)
            if d > dist.get(key, np.inf):
                continue
            if key in goal_set and d + goal_keys[key] < best_cost:
                best_goal, best_cost = key, d + goal_keys[key]
            if d > best_cost:
                break
            for nk, cost, kind in self.nodes[key].edges:
                nd = d + cost
                if nd < dist.get(nk, np.inf):
                    dist[nk] = nd
                    prev[nk] = key
                    heapq.heappush(pq, (nd, nk))
        if best_goal is None:
            return None
        path = [best_goal]
        while path[-1] in prev:
            path.append(prev[path[-1]])
        return path[::-1]


def build_route(graph: LaneGraph, waypoints, blend=25.0):
    """waypoint 열(seq 순 (x,y)) → 노드 경로 → 조밀 중심선 Nx2.

    반환: (pts Nx2, node_keys)
    """
    # 각 waypoint의 후보 노드
    cand = []
    for x, y in waypoints:
        c = graph.nearest_nodes(x, y)
        if not c:
            raise ValueError(f"waypoint ({x},{y}) 근처에 driving 차선 없음")
        cand.append({key: d for key, s, d in c})

    # 순차 구간 최단경로 이어붙이기
    node_seq = []
    for i in range(len(cand) - 1):
        leg = graph.shortest_path(cand[i], cand[i + 1])
        if leg is None:
            raise ValueError(f"waypoint {i+1}→{i+2} 경로 없음")
        if node_seq and node_seq[-1] == leg[0]:
            leg = leg[1:]
        node_seq.extend(leg)

    # 기하 조립 (차로변경 'lc' 엣지는 선형 블렌드)
    pts = []
    for i, key in enumerate(node_seq):
        node = graph.nodes[key]
        seg = node.pts
        if pts:
            prev_end = pts[-1][-1]
            gap = np.hypot(*(seg[0] - prev_end))
            if gap > 1.0:  # 차로변경 블렌드: 이전 끝 → 현재 차선 blend 지점
                n_into = np.searchsorted(node.s, min(blend, node.length * 0.5))
                seg = seg[max(n_into, 1):]
                w = np.linspace(0, 1, max(n_into, 2))[:, None]
                bridge = prev_end * (1 - w) + node.pts[max(n_into - 1, 0)] * w
                pts.append(bridge)
        pts.append(seg)
    route = np.vstack(pts)

    # waypoint 첫/끝 지점에서 트림 — 순환 경로(시작≈끝) 대응:
    # 시작점은 경로 앞쪽 절반에서, 끝점은 뒤쪽 절반에서 최근접을 찾는다
    d0 = np.hypot(route[:, 0] - waypoints[0][0], route[:, 1] - waypoints[0][1])
    d1 = np.hypot(route[:, 0] - waypoints[-1][0], route[:, 1] - waypoints[-1][1])
    half = max(1, len(route) // 2)
    i0 = int(np.argmin(d0[:half]))
    i1 = half + int(np.argmin(d1[half:]))
    if i0 < i1:
        route = route[i0:i1 + 1]

    # 접합부 등에서 생기는 완전/거의 동일 점 제거 (ds<1e-3 → np.gradient 0나눗셈 → 곡률 NaN/∞)
    if len(route) > 1:
        ds = np.hypot(*np.diff(route, axis=0).T)
        keep = np.concatenate([[True], ds > 1e-3])
        route = route[keep]

    return route, node_seq

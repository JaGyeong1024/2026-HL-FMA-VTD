"""경로 기반 신호등 배정 (9/1 공식 Q&A: trafficLightId는 맵과 무매핑, state만 유효).

VTD는 "Ego 진행방향에 대한 신호 상태" 하나를 준다. Autoware는 규제요소(TrafficLightGroup) 단위로
상태를 받는다. 이 모듈은 Autoware 경로(LaneletRoute)와 ego 위치로
  "경로상 다음 정지선"과 그 정지선을 참조하는 **모든** 신호등 규제요소 id
를 찾는다. 상태 해석(가라/서라)은 bridge_node가 한다.

설계 원칙
- 현재 위치는 매 프레임 경로 lanelet 폴리곤 포함 판정으로 찾는다 (직전 인덱스 주변 → 실패 시 경로 전체).
  리스폰으로 수십 lanelet 뒤로 가도 고착되지 않는다.
- "다음 정지선"은 lanelet 인덱스가 아니라 경로 누적거리 s 기준. 정지선을 지나면(범퍼 기준) 그 다음으로 넘어간다.
- set_route는 새 상태를 지역변수로 만든 뒤 한 번에 교체한다 (rx 스레드와 경합 방지).
"""
import math
import threading

from .osm_map import OsmMap


class StopEntry:
    __slots__ = ('s_route', 'lanelet_id', 'groups', 'route_idx', 'xy')

    def __init__(self, s_route, lanelet_id, groups, route_idx, xy):
        self.s_route = s_route
        self.lanelet_id = lanelet_id
        self.groups = groups
        self.route_idx = route_idx
        self.xy = xy


class TrafficLightRouter:
    def __init__(self, osm_map: OsmMap, logger=None):
        self.map = osm_map
        self.log = logger
        self._lock = threading.Lock()
        self.route_ids = []
        self.offsets = []       # route_idx -> 경로 누적 s (lanelet 시작)
        self.stops = []         # [StopEntry] s_route 오름차순
        self.cur_idx = 0
        self.total_len = 0.0

    def _info(self, msg):
        if self.log:
            self.log.info(msg)

    def set_route(self, lanelet_ids):
        ids = [i for i in lanelet_ids if i in self.map.lanelets]
        offsets, stops = [], []
        s = 0.0
        for idx, lid in enumerate(ids):
            ll = self.map.lanelets[lid]
            offsets.append(s)
            if ll.tl_groups:
                s_stop = self.map.stop_line_s(lid)
                if s_stop is None:
                    s_stop = ll.length
                mx = (ll.stop_line[0][0] + ll.stop_line[1][0]) / 2.0 if ll.stop_line else None
                my = (ll.stop_line[0][1] + ll.stop_line[1][1]) / 2.0 if ll.stop_line else None
                stops.append(StopEntry(s + s_stop, lid, list(ll.tl_groups), idx, (mx, my)))
            s += ll.length
        stops.sort(key=lambda e: e.s_route)
        with self._lock:
            self.route_ids = ids
            self.offsets = offsets
            self.stops = stops
            self.total_len = s
            self.cur_idx = 0
        dropped = len(lanelet_ids) - len(ids)
        self._info(f'TlRouter: 경로 lanelet {len(ids)}개({s:.0f}m), 정지선 {len(stops)}개'
                   + (f', 맵에 없는 lanelet {dropped}개' if dropped else ''))

    def has_route(self):
        return bool(self.route_ids)

    def locate(self, x, y):
        """ego → (route_idx, s_route). 경로 밖이면 최근접 lanelet으로 추정. 경로 없으면 None."""
        with self._lock:
            ids, offsets, cur = self.route_ids, self.offsets, self.cur_idx
        if not ids:
            return None
        n = len(ids)
        # 1) 직전 인덱스 주변 (뒤 2 ~ 앞 6)
        order = list(range(max(0, cur - 2), min(n, cur + 7)))
        found = next((i for i in order if self.map.contains(ids[i], x, y)), None)
        # 2) 실패 시 경로 전체 (리스폰·초기)
        if found is None:
            found = next((i for i in range(n) if self.map.contains(ids[i], x, y)), None)
        # 3) 폴리곤 밖(차로 이탈 등): 경로 lanelet 중 최근접
        if found is None:
            best = float('inf')
            for i in range(n):
                d, _, _ = self.map.project(ids[i], x, y)
                if d < best:
                    best, found = d, i
        _, s_in, _ = self.map.project(ids[found], x, y)
        with self._lock:
            self.cur_idx = found
        return found, offsets[found] + s_in

    def next_stop(self, s_ego, passed_margin=1.0):
        """s_ego 앞의 첫 정지선 (범퍼가 정지선을 passed_margin 이상 지났으면 다음)."""
        with self._lock:
            stops = self.stops
        for e in stops:
            if e.s_route + passed_margin >= s_ego:
                return e
        return None

    def stops_snapshot(self):
        with self._lock:
            return list(self.stops)

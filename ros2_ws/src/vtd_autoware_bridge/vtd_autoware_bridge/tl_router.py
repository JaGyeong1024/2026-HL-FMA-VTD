"""경로 기반 신호등 배정 — 9/1 공식 Q&A 반영.

trafficLightId는 맵 signal id와 매핑되지 않으므로 (Ego 진행방향 대표 상태만 옴),
Autoware 경로(LaneletRoute)의 lanelet 순서에서 "다음 신호등 그룹"을 찾아
TCP state를 그 그룹에 적용한다.

OSM 구조 (자체 익스포터 산출물):
  <relation type=lanelet>  ── member role=left/right (way) + role=regulatory_element (relation)
  <relation type=regulatory_element subtype=traffic_light xodr_signal_id=...>
"""
import math
import xml.etree.ElementTree as ET


class TlRouter:
    def __init__(self, osm_path: str, logger=None):
        self.osm_path = osm_path
        self.log = logger
        self.lanelet_tls = {}     # lanelet_id -> [tl_group_relation_id]
        self.lanelet_ways = {}    # lanelet_id -> (left_way, right_way)  (경로 lanelet 지오메트리용)
        self._parse_relations()
        # 경로 상태
        self.route_ids = []       # 순서대로 lanelet id
        self.route_tls = []       # [(route_idx, group_id)] 오름차순
        self.centroids = []       # route_idx -> (x, y) or None
        self.cur_idx = 0

    def _info(self, msg):
        if self.log:
            self.log.info(msg)

    def _parse_relations(self):
        tl_groups = set()
        for _, e in ET.iterparse(self.osm_path):
            if e.tag != 'relation':
                continue
            tags = {t.get('k'): t.get('v') for t in e.findall('tag')}
            rid = int(e.get('id'))
            if tags.get('type') == 'regulatory_element' and tags.get('subtype') == 'traffic_light':
                tl_groups.add(rid)
            elif tags.get('type') == 'lanelet':
                regs, left, right = [], None, None
                for m in e.findall('member'):
                    role = m.get('role')
                    if m.get('type') == 'relation' and role == 'regulatory_element':
                        regs.append(int(m.get('ref')))
                    elif role == 'left':
                        left = int(m.get('ref'))
                    elif role == 'right':
                        right = int(m.get('ref'))
                if regs:
                    self.lanelet_tls[rid] = regs
                self.lanelet_ways[rid] = (left, right)
            e.clear()
        # 신호등 그룹만 남기기 (speed_limit 등 다른 reg elem 제거)
        for lid in list(self.lanelet_tls):
            tls = [r for r in self.lanelet_tls[lid] if r in tl_groups]
            if tls:
                self.lanelet_tls[lid] = tls
            else:
                del self.lanelet_tls[lid]
        self._tl_groups = tl_groups
        self._info(f'TlRouter: 신호등 그룹 {len(tl_groups)}, 신호등 있는 lanelet {len(self.lanelet_tls)}')

    def set_route(self, lanelet_ids):
        """LaneletRoute의 preferred lanelet id 열 → 경로 신호등 목록·지오메트리 구축."""
        self.route_ids = list(lanelet_ids)
        self.cur_idx = 0
        self.route_tls = []
        for i, lid in enumerate(self.route_ids):
            for g in self.lanelet_tls.get(lid, []):
                self.route_tls.append((i, g))
        self.centroids = self._load_centroids(self.route_ids)
        self._info(f'TlRouter: 경로 lanelet {len(self.route_ids)}개, 경로상 신호등 {len(self.route_tls)}개')

    def _load_centroids(self, lanelet_ids):
        """경로 lanelet들의 대략 중심점 — 대상 way/node만 골라 3-pass 파싱."""
        need_ways = set()
        for lid in lanelet_ids:
            lw = self.lanelet_ways.get(lid)
            if lw:
                need_ways.update(w for w in lw if w is not None)
        way_nodes = {}   # way_id -> [node_id] (양끝+중간만)
        for _, e in ET.iterparse(self.osm_path):
            if e.tag == 'way':
                wid = int(e.get('id'))
                if wid in need_ways:
                    nds = [int(n.get('ref')) for n in e.findall('nd')]
                    if nds:
                        way_nodes[wid] = [nds[0], nds[len(nds) // 2], nds[-1]]
            e.clear()
        need_nodes = {n for nds in way_nodes.values() for n in nds}
        coords = {}
        for _, e in ET.iterparse(self.osm_path):
            if e.tag == 'node':
                nid = int(e.get('id'))
                if nid in need_nodes:
                    tags = {t.get('k'): t.get('v') for t in e.findall('tag')}
                    try:
                        coords[nid] = (float(tags['local_x']), float(tags['local_y']))
                    except (KeyError, ValueError):
                        pass
            e.clear()
        cents = []
        for lid in lanelet_ids:
            pts = []
            for w in (self.lanelet_ways.get(lid) or ()):  # left, right
                for n in way_nodes.get(w, []):
                    if n in coords:
                        pts.append(coords[n])
            if pts:
                cents.append((sum(p[0] for p in pts) / len(pts),
                              sum(p[1] for p in pts) / len(pts)))
            else:
                cents.append(None)
        return cents

    def next_group(self, x, y):
        """ego 위치 → 경로상 다음 신호등 그룹 id (없으면 None)."""
        if not self.route_ids or not self.route_tls:
            return None
        # 현재 lanelet 인덱스 추정: 직전 인덱스 주변(뒤 2 ~ 앞 8)에서 최근접 centroid
        lo = max(0, self.cur_idx - 2)
        hi = min(len(self.route_ids), self.cur_idx + 9)
        best, best_d = self.cur_idx, float('inf')
        for i in range(lo, hi):
            c = self.centroids[i]
            if c is None:
                continue
            d = math.hypot(c[0] - x, c[1] - y)
            if d < best_d:
                best, best_d = i, d
        self.cur_idx = best
        for i, g in self.route_tls:
            if i >= self.cur_idx:
                return g
        return None

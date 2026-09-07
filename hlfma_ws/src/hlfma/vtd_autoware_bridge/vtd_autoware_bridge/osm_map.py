"""경량 Lanelet2 OSM 로더 — 브리지·route_node 공용.

Autoware가 읽는 것과 같은 OSM 파일(자체 익스포터 산출물)에서 다음만 읽는다:
  node: local_x / local_y (VTD 월드좌표 = Autoware map 프레임)
  way:  노드 열 (차선 경계, 정지선)
  relation type=lanelet: left/right way, regulatory_element 참조, 태그(turn_direction 등)
  relation type=regulatory_element subtype=traffic_light: ref_line(정지선 way)

lanelet2 파이썬 바인딩을 쓰지 않는 이유: 이 파일은 lat/lon이 더미이고 local_x/local_y가 진짜 좌표라
Autoware의 local projector와 같은 방식으로 읽어야 한다. 필요한 기하는 중심선·폴리곤·정지선뿐이다.

주의: ET.iterparse는 자식(<tag>, <nd>, <member>)의 end 이벤트를 부모보다 먼저 낸다.
      부모를 처리하기 전에 자식을 clear()하면 속성이 사라진다 → clear()는 부모 처리 후에만.
"""
import math
import xml.etree.ElementTree as ET


def _seg_dist(px, py, ax, ay, bx, by):
    """점 P와 선분 AB 거리, 선분상 투영 파라미터 t(0~1)."""
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(px - ax, py - ay), 0.0
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    qx, qy = ax + t * dx, ay + t * dy
    return math.hypot(px - qx, py - qy), t


def _point_in_polygon(px, py, poly):
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > py) != (yj > py):
            x_cross = (xj - xi) * (py - yi) / (yj - yi) + xi
            if px < x_cross:
                inside = not inside
        j = i
    return inside


class Lanelet:
    __slots__ = ('id', 'left', 'right', 'regelems', 'tags', 'center', 'polygon',
                 'tl_groups', 'stop_line', 'length', 'cum_s')

    def __init__(self, lid):
        self.id = lid
        self.left = None
        self.right = None
        self.regelems = []
        self.tags = {}
        self.center = []      # [(x, y)]
        self.polygon = []     # left + reversed(right)
        self.tl_groups = []   # 이 lanelet에 붙은 traffic_light 규제요소 id
        self.stop_line = None  # [(x, y), (x, y)] 대표 정지선 (첫 tl 규제요소의 ref_line)
        self.length = 0.0
        self.cum_s = []


class OsmMap:
    def __init__(self, path, logger=None):
        self.path = path
        self.log = logger
        self.nodes = {}      # id -> (x, y)
        self.node_z = {}     # id -> z (ele 태그, 없으면 0)
        self.ways = {}       # id -> [node id]
        self.way_tags = {}   # id -> {k: v} (subtype/lane_change/color — 실선·차선변경 허용 판정용)
        self._by_left_way = {}   # way id -> 그 way 를 왼쪽 경계로 쓰는 lanelet id (이웃 탐색)
        self._by_right_way = {}  # way id -> 그 way 를 오른쪽 경계로 쓰는 lanelet id
        self.lanelets = {}   # id -> Lanelet
        self.tl_regelems = {}  # regelem id -> {'ref_line': way id, 'refers': [way id]}
        self._grid = {}      # (gx, gy) -> [lanelet id]
        self._cell = 30.0
        self._load()
        self._build()

    def _info(self, msg):
        if self.log:
            self.log.info(msg)

    # ---------- 파싱 ----------
    def _load(self):
        for _, e in ET.iterparse(self.path, events=('end',)):
            tag = e.tag
            if tag == 'node':
                x = y = None
                z = 0.0
                for t in e.findall('tag'):
                    k = t.get('k')
                    if k == 'local_x':
                        x = float(t.get('v'))
                    elif k == 'local_y':
                        y = float(t.get('v'))
                    elif k == 'ele':
                        z = float(t.get('v'))
                if x is not None and y is not None:
                    self.nodes[int(e.get('id'))] = (x, y)
                    self.node_z[int(e.get('id'))] = z
                e.clear()
            elif tag == 'way':
                self.ways[int(e.get('id'))] = [int(n.get('ref')) for n in e.findall('nd')]
                self.way_tags[int(e.get('id'))] = {t.get('k'): t.get('v') for t in e.findall('tag')}
                e.clear()
            elif tag == 'relation':
                tags = {t.get('k'): t.get('v') for t in e.findall('tag')}
                rid = int(e.get('id'))
                typ = tags.get('type')
                if typ == 'lanelet':
                    ll = Lanelet(rid)
                    ll.tags = tags
                    for m in e.findall('member'):
                        role, ref = m.get('role'), int(m.get('ref'))
                        if role == 'left':
                            ll.left = ref
                        elif role == 'right':
                            ll.right = ref
                        elif role == 'regulatory_element':
                            ll.regelems.append(ref)
                    self.lanelets[rid] = ll
                elif typ == 'regulatory_element' and tags.get('subtype') == 'traffic_light':
                    info = {'ref_line': None, 'refers': []}
                    for m in e.findall('member'):
                        role, ref = m.get('role'), int(m.get('ref'))
                        if role == 'ref_line':
                            info['ref_line'] = ref
                        elif role == 'refers':
                            info['refers'].append(ref)
                    self.tl_regelems[rid] = info
                e.clear()
            # <tag>/<nd>/<member>는 부모가 처리·clear 하므로 여기서 건드리지 않는다

    def _way_pts(self, wid):
        return [self.nodes[n] for n in self.ways.get(wid, []) if n in self.nodes]

    def _build(self):
        for ll in self.lanelets.values():
            L = self._way_pts(ll.left) if ll.left is not None else []
            R = self._way_pts(ll.right) if ll.right is not None else []
            if len(L) < 2 or len(R) < 2:
                continue
            # 중심선: 더 촘촘한 쪽을 기준으로 상대 경계에 투영해 평균
            base, other = (L, R) if len(L) >= len(R) else (R, L)
            center = []
            for (x, y) in base:
                best, bt, bi = float('inf'), 0.0, 0
                for i in range(len(other) - 1):
                    d, t = _seg_dist(x, y, *other[i], *other[i + 1])
                    if d < best:
                        best, bt, bi = d, t, i
                ax, ay = other[bi]
                bx, by = other[bi + 1]
                qx, qy = ax + bt * (bx - ax), ay + bt * (by - ay)
                center.append(((x + qx) / 2.0, (y + qy) / 2.0))
            ll.center = center
            ll.polygon = L + list(reversed(R))
            cum = [0.0]
            for i in range(1, len(center)):
                cum.append(cum[-1] + math.hypot(center[i][0] - center[i - 1][0],
                                                center[i][1] - center[i - 1][1]))
            ll.cum_s = cum
            ll.length = cum[-1]
            ll.tl_groups = [r for r in ll.regelems if r in self.tl_regelems]
            self._by_left_way.setdefault(ll.left, ll.id)
            self._by_right_way.setdefault(ll.right, ll.id)
            if ll.tl_groups:
                ref = self.tl_regelems[ll.tl_groups[0]]['ref_line']
                pts = self._way_pts(ref) if ref is not None else []
                if len(pts) >= 2:
                    ll.stop_line = [pts[0], pts[-1]]
            # 공간 격자 등록
            xs = [p[0] for p in ll.polygon]
            ys = [p[1] for p in ll.polygon]
            for gx in range(int(min(xs) // self._cell), int(max(xs) // self._cell) + 1):
                for gy in range(int(min(ys) // self._cell), int(max(ys) // self._cell) + 1):
                    self._grid.setdefault((gx, gy), []).append(ll.id)
        n_tl = sum(1 for ll in self.lanelets.values() if ll.tl_groups)
        zs = list(self.node_z.values())
        self._info(f'OsmMap: z 범위 {min(zs):.1f}~{max(zs):.1f}m, ' if zs else '')
        self._info(f'OsmMap: node {len(self.nodes)}, lanelet {len(self.lanelets)}, '
                   f'신호등 규제요소 {len(self.tl_regelems)}, 신호등 lanelet {n_tl}')

    # ---------- 질의 ----------
    def candidates(self, x, y):
        gx, gy = int(x // self._cell), int(y // self._cell)
        out = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                out.extend(self._grid.get((gx + dx, gy + dy), []))
        return out

    def contains(self, lid, x, y):
        ll = self.lanelets.get(lid)
        return bool(ll and ll.polygon and _point_in_polygon(x, y, ll.polygon))

    def project(self, lid, x, y):
        """lanelet 중심선에 투영: (거리, 중심선 s, 진행방향 heading)."""
        ll = self.lanelets[lid]
        c = ll.center
        best, bs, bh = float('inf'), 0.0, 0.0
        for i in range(len(c) - 1):
            d, t = _seg_dist(x, y, *c[i], *c[i + 1])
            if d < best:
                seg = math.hypot(c[i + 1][0] - c[i][0], c[i + 1][1] - c[i][1])
                best = d
                bs = ll.cum_s[i] + t * seg
                bh = math.atan2(c[i + 1][1] - c[i][1], c[i + 1][0] - c[i][0])
        return best, bs, bh

    def nearest_lanelet(self, x, y, heading=None, max_dist=6.0, avoid_dead_end=True):
        """가장 가까운 lanelet id (heading 주어지면 ±90° 안쪽만). 없으면 None.
        avoid_dead_end: 최근접 lanelet에 후속 lanelet이 없고(맵 경계·차선 소멸·교차로 잉여 차선)
        1.5m 이내에 후속이 있는 이웃 lanelet이 있으면 그쪽을 택한다 (xodr 원본의 막다른 차선 90개 회피)."""
        cands = []
        for lid in self.candidates(x, y):
            d, _, h = self.project(lid, x, y)
            if heading is not None:
                dh = (h - heading + math.pi) % (2 * math.pi) - math.pi
                if abs(dh) > math.pi / 2:
                    continue
            if d <= max_dist:
                cands.append((d, lid))
        if not cands:
            return None
        cands.sort()
        best_d, best = cands[0]
        if avoid_dead_end and not self.successors(best):
            for d, lid in cands[1:]:
                if d - best_d <= 1.5 and self.successors(lid):
                    return lid
        return best

    def is_dead_end(self, lid):
        return not self.successors(lid)

    def has_predecessor(self, lid):
        ll = self.lanelets[lid]
        l0, r0 = self.ways[ll.left][0], self.ways[ll.right][0]
        for o in self.lanelets.values():
            if o.id != lid and o.left is not None and self.ways[o.left][-1] == l0 and self.ways[o.right][-1] == r0:
                return True
        return False

    def width_at(self, lid, x, y):
        ll = self.lanelets[lid]
        L = [self.nodes[n] for n in self.ways[ll.left]]
        R = [self.nodes[n] for n in self.ways[ll.right]]
        i = min(range(len(L)), key=lambda k: math.hypot(L[k][0] - x, L[k][1] - y))
        j = min(range(len(R)), key=lambda k: math.hypot(R[k][0] - x, R[k][1] - y))
        return math.hypot(L[i][0] - R[j][0], L[i][1] - R[j][1])

    def match_candidates(self, x, y, headings, max_dist=8.0, need_pred=True, need_succ=True, min_width=2.0):
        """경로 CSV 점의 lanelet 후보 목록 [(lanelet id, 벌점 m, 진단)]. 벌점은 라우팅 총길이에 더해 비교한다.
        - headings: 허용 진행방향 힌트들(진입 방향·진출 방향). 어느 하나와 ±90° 안이면 후보
        - 벌점: 거리 + 방향차(45°당 3m) + 선행 없음 6m(시작점 제외) + 후속 없음 6m(종료점 제외) + 폭<min_width 6m
          → 폭 0에서 생기는 확폭 차선·막다른 차선보다 본선을 선호 (검토보고 D-1·E-2)"""
        out = []
        seen = set()
        for lid in self.candidates(x, y):
            if lid in seen:
                continue
            seen.add(lid)
            d, s, h = self.project(lid, x, y)
            if d > max_dist:
                continue
            dh = min(abs((h - hd + math.pi) % (2 * math.pi) - math.pi) for hd in headings)
            if dh > math.pi / 2:
                continue
            w = self.width_at(lid, x, y)
            flags = []
            pen = d + 3.0 * dh / (math.pi / 4)
            if need_pred and not self.has_predecessor(lid):
                pen += 6.0; flags.append('선행없음')
            if need_succ and not self.successors(lid):
                pen += 6.0; flags.append('후속없음')
            if w < min_width:
                pen += 6.0; flags.append(f'폭{w:.1f}')
            out.append((lid, pen, {'d': d, 'dh_deg': math.degrees(dh), 'w': w, 'flags': flags, 's': s, 'h': h}))
        out.sort(key=lambda t: t[1])
        return out[:5]

    def heading_at(self, lid, x, y):
        return self.project(lid, x, y)[2]

    def elevation(self, lid, x, y):
        """lanelet 경계 노드 중 (x,y)에 가장 가까운 노드의 z (맵 높이)."""
        ll = self.lanelets[lid]
        best, bz = float('inf'), 0.0
        for wid in (ll.left, ll.right):
            for n in self.ways.get(wid, []):
                if n in self.nodes:
                    d = math.hypot(self.nodes[n][0] - x, self.nodes[n][1] - y)
                    if d < best:
                        best, bz = d, self.node_z.get(n, 0.0)
        return bz

    def neighbors(self, lid):
        """(왼쪽 이웃 lanelet id, 오른쪽 이웃 lanelet id). 경계 way 를 공유하는 같은 방향 lanelet 만 (없으면 None).
        익스포터 산출 맵은 이웃 차선이 경계 way 를 공유한다 (2480 중 1205 lanelet 이 왼쪽 경계 공유 확인, 9/7)."""
        ll = self.lanelets.get(lid)
        if ll is None:
            return None, None
        left = self._by_right_way.get(ll.left)    # 내 왼쪽 경계를 오른쪽 경계로 쓰는 lanelet = 왼쪽 이웃
        right = self._by_left_way.get(ll.right)   # 내 오른쪽 경계를 왼쪽 경계로 쓰는 lanelet = 오른쪽 이웃
        return (left if left != lid else None), (right if right != lid else None)

    def boundary_solid(self, lid, side):
        """side('left'|'right') 경계가 차선변경 금지(실선)인가. lane_change 태그 우선, 없으면 subtype.
        주의: 익스포터 lane_change 태그는 섹션 중점 기준 근사 (todo0906) — 오탐 시 익스포터 정밀화가 선결."""
        ll = self.lanelets.get(lid)
        if ll is None:
            return True
        tags = self.way_tags.get(ll.left if side == 'left' else ll.right, {})
        if 'lane_change' in tags:
            return tags['lane_change'] != 'yes'
        return tags.get('subtype', '').startswith('solid')

    def successors(self, lid):
        """왼쪽·오른쪽 경계의 끝 노드를 시작 노드로 갖는 lanelet들."""
        ll = self.lanelets[lid]
        l_end = self.ways[ll.left][-1]
        r_end = self.ways[ll.right][-1]
        out = []
        for o in self.lanelets.values():
            if o.id == lid or o.left is None or o.right is None:
                continue
            if self.ways[o.left][0] == l_end and self.ways[o.right][0] == r_end:
                out.append(o.id)
        return out

    def point_along(self, lid, s_from, dist, prefer=None):
        """lanelet 중심선 s_from에서 dist만큼 앞의 점과 heading. lanelet 끝을 넘으면 후속 lanelet으로.
        prefer: 후속 후보가 여럿일 때 우선할 lanelet id 집합(경로). 반환 (x, y, heading, lanelet id)."""
        cur, s = lid, s_from + dist
        for _ in range(20):
            ll = self.lanelets[cur]
            if s <= ll.length or ll.length == 0.0:
                return self._interp(ll, s) + (cur,)
            succ = self.successors(cur)
            if not succ:
                return self._interp(ll, ll.length) + (cur,)
            nxt = next((c for c in succ if prefer and c in prefer), succ[0])
            s -= ll.length
            cur = nxt
        ll = self.lanelets[cur]
        return self._interp(ll, min(s, ll.length)) + (cur,)

    def _interp(self, ll, s):
        c, cum = ll.center, ll.cum_s
        s = max(0.0, min(s, ll.length))
        for i in range(len(c) - 1):
            if cum[i + 1] >= s:
                seg = cum[i + 1] - cum[i]
                t = (s - cum[i]) / seg if seg > 1e-9 else 0.0
                x = c[i][0] + t * (c[i + 1][0] - c[i][0])
                y = c[i][1] + t * (c[i + 1][1] - c[i][1])
                h = math.atan2(c[i + 1][1] - c[i][1], c[i + 1][0] - c[i][0])
                return x, y, h
        h = math.atan2(c[-1][1] - c[-2][1], c[-1][0] - c[-2][0])
        return c[-1][0], c[-1][1], h

    def stop_line_s(self, lid):
        """lanelet 중심선 기준 정지선 위치 s (없으면 None)."""
        ll = self.lanelets[lid]
        if not ll.stop_line:
            return None
        mx = (ll.stop_line[0][0] + ll.stop_line[1][0]) / 2.0
        my = (ll.stop_line[0][1] + ll.stop_line[1][1]) / 2.0
        return self.project(lid, mx, my)[1]

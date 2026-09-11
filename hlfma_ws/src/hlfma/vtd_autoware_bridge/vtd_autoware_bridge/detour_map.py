"""Fail-closed map policy for the project's local-coordinate Lanelet OSM.

This is an additional veto, not a replacement for Lanelet2 traffic rules and
Autoware's regulatory/collision checks. Unknown boundary permissions are denied.
"""
import heapq
import math
from .detour_geometry import footprint
from .tl_router import TrafficLightRouter


class DetourMap:
    def __init__(self, osm):
        self.map = osm
        self.router = TrafficLightRouter(osm)
        self.route = []
        self.successors = {}
        self.adjacent = {}
        self.predecessors = {}
        starts, lefts, rights = {}, {}, {}
        for lid, ll in osm.lanelets.items():
            if not ll.center or ll.left not in osm.ways or ll.right not in osm.ways:
                continue
            starts.setdefault((osm.ways[ll.left][0], osm.ways[ll.right][0]), []).append(lid)
            lefts.setdefault(ll.left, []).append(lid)
            rights.setdefault(ll.right, []).append(lid)
        for lid, ll in osm.lanelets.items():
            if not ll.center:
                continue
            self.successors[lid] = starts.get((osm.ways[ll.left][-1], osm.ways[ll.right][-1]), [])
            self.adjacent[lid] = []
            for boundary, others in ((ll.left, rights), (ll.right, lefts)):
                tags = osm.way_tags.get(boundary, {})
                if tags.get('lane_change') != 'yes' or tags.get('subtype') not in ('dashed', 'dashed_dashed'):
                    continue
                for other in others.get(boundary, []):
                    if other != lid and self.heading_agrees(lid, other):
                        self.adjacent[lid].append(other)

        for lid, successors in self.successors.items():
            for nxt in successors:
                self.predecessors.setdefault(nxt, []).append(lid)

    def on_preferred(self, ego, heading):
        return bool(self.lanes_at(ego.x, ego.y, heading) & set(self.route))

    def heading_agrees(self, a, b):
        aa, bb = self.map.lanelets[a].center, self.map.lanelets[b].center
        u = (aa[-1][0]-aa[0][0], aa[-1][1]-aa[0][1])
        v = (bb[-1][0]-bb[0][0], bb[-1][1]-bb[0][1])
        return u[0]*v[0]+u[1]*v[1] > 0

    def set_route(self, ids):
        if not ids or any(i not in self.map.lanelets for i in ids):
            self.route = []
            self.router.set_route([])
            return
        self.route = list(ids)
        self.router.set_route(ids)

    def lanes_at(self, x, y, heading):
        out = set()
        for lid in set(self.map.candidates(x, y)):
            ll = self.map.lanelets[lid]
            if ll.tags.get('subtype') != 'road' or ll.tags.get('one_way') != 'yes':
                continue
            if not self.map.contains(lid, x, y):
                continue
            _, _, h = self.map.project(lid, x, y)
            if abs((h-heading+math.pi) % (2*math.pi)-math.pi) < math.pi/3:
                out.add(lid)
        return out

    def next_signal(self, ego):
        if not self.route:
            return None
        loc = self.router.locate(ego.x, ego.y)
        if loc is None:
            return None
        entry = self.router.next_stop(loc[1])
        if entry is None:
            return None
        turn = self.map.lanelets[entry.lanelet_id].tags.get('turn_direction')
        if turn is None:
            # Upstream stop-line lanelets may omit turn_direction.
            forward = 0.0
            for lid in self.route[entry.route_idx:]:
                ll = self.map.lanelets[lid]
                turn = ll.tags.get('turn_direction')
                forward += ll.length
                if turn or forward > 40:
                    break
        return entry.s_route-loc[1], entry.groups, turn or 'unknown', entry.lanelet_id

    def validate(self, path, ego, finish, required, front, rear, width, return_reserve):
        if not self.route:
            return 'MAP_UNAVAILABLE'
        _, origin = path.project(ego.x, ego.y)
        previous = None
        finish_lanes = set()
        # Check candidate center transitions and sampled footprint coverage.
        for s, pose in path.samples(origin, min(path.length, origin+required), 0.5):
            lanes = self.lanes_at(*pose)
            if not lanes:
                return 'REGULATION'
            if previous is not None:
                lanes = {lid for lid in lanes if any(
                    lid == prev or lid in self.successors.get(prev, []) or
                    lid in self.adjacent.get(prev, []) for prev in previous)}
                if not lanes:
                    return 'REGULATION'
            lanes = {lid for lid in lanes if self.map.lanelets[lid].tags.get('turn_direction')
                     not in ('left', 'right') or lid in self.route}
            if not lanes:
                return 'REGULATION'
            previous = lanes
            allowed = set(lanes)
            for lid in lanes:
                allowed.update(self.successors.get(lid, []))
                allowed.update(self.adjacent.get(lid, []))
                # The rear can still occupy a predecessor at a lanelet seam.
                allowed.update(self.predecessors.get(lid, []))
            for lid in tuple(allowed):
                allowed.update(self.successors.get(lid, []))
                allowed.update(self.predecessors.get(lid, []))
            # Vehicle corners and edge midpoints must remain on permitted road lanes.
            corners = footprint(*pose, front, rear, width)
            probes = corners + [((a[0]+b[0])/2, (a[1]+b[1])/2)
                                for a, b in zip(corners, corners[1:]+corners[:1])]
            if any(not any(self.map.contains(lid, x, y) for lid in allowed) for x, y in probes):
                return 'REGULATION'
            if s-origin >= finish and not finish_lanes:
                finish_lanes = lanes
        if not finish_lanes:
            return 'NO_RETURN_SPACE'
        loc = self.router.locate(ego.x, ego.y)
        preferred = set(self.route[loc[0]:])
        pose = path.at(origin+finish)
        for lid in finish_lanes:
            if lid in preferred:
                return None
            remaining = self.map.lanelets[lid].length-self.map.project(lid, pose[0], pose[1])[1]
            if self.can_return(lid, preferred, remaining, return_reserve):
                return None
        return 'NO_RETURN_SPACE'

    def can_return(self, lid, preferred, remaining, reserve):
        # Reach a preferred lane before any off-route turn; reserve is consumed
        # for every lateral transition. No guessed connections across solid lines.
        queue = [(0., lid, remaining)]
        visited = set()
        while queue:
            distance, cur, available = heapq.heappop(queue)
            if cur in visited or distance > 150:
                continue
            visited.add(cur)
            if cur in preferred:
                return True
            ll = self.map.lanelets[cur]
            if ll.tags.get('turn_direction') in ('left', 'right'):
                continue
            for nxt in self.adjacent.get(cur, []):
                completion = self.lateral_completion(cur, nxt, available, reserve)
                if completion:
                    target, remaining = completion
                    heapq.heappush(queue, (distance+reserve, target, remaining))
            for nxt in self.successors.get(cur, []):
                heapq.heappush(queue, (distance+available, nxt, self.map.lanelets[nxt].length))
        return False

    def lateral_completion(self, source, target, available, reserve):
        """Reserve may span short lanelets only while both lanes keep permission."""
        x, y, _ = self.map._interp(self.map.lanelets[source], self.map.lanelets[source].length-available)
        target_remaining = self.map.lanelets[target].length-self.map.project(target,x,y)[1]
        for _ in range(100):
            if target not in self.adjacent.get(source, []):
                return None
            if any(self.map.lanelets[l].tags.get('turn_direction') in ('left','right') for l in (source,target)):
                return None
            progress = min(available, target_remaining, reserve)
            available -= progress
            target_remaining -= progress
            reserve -= progress
            if reserve <= 1e-6:
                return target, target_remaining
            # Accept only an unambiguous paired continuation; junction branches
            # cannot be guessed as a legal return corridor.
            sources = self.successors.get(source, []) if available <= 1e-6 else [source]
            targets = self.successors.get(target, []) if target_remaining <= 1e-6 else [target]
            pairs = [(a,b) for a in sources for b in targets if b in self.adjacent.get(a, [])]
            if len(pairs) != 1:
                return None
            a,b = pairs[0]
            if a != source:
                available = self.map.lanelets[a].length
            if b != target:
                target_remaining = self.map.lanelets[b].length
            source,target = a,b
        return None

    def candidate_signals(self, path, ego, horizon):
        """Signals whose actual stop line intersects the candidate center path."""
        _, origin = path.project(ego.x, ego.y)
        contexts = []
        seen = set()
        for s, pose in path.samples(origin, min(path.length, origin+horizon), 2.0):
            for lid in self.lanes_at(*pose):
                ll = self.map.lanelets[lid]
                if lid in seen or not ll.tl_groups or not ll.stop_line:
                    continue
                seen.add(lid)
                c,d = ll.stop_line
                for i,(a,b) in enumerate(zip(path.xy,path.xy[1:])):
                    u=(b[0]-a[0],b[1]-a[1]); v=(d[0]-c[0],d[1]-c[1])
                    den=u[0]*v[1]-u[1]*v[0]
                    if abs(den)<1e-9:
                        continue
                    w=(c[0]-a[0],c[1]-a[1])
                    t=(w[0]*v[1]-w[1]*v[0])/den
                    q=(w[0]*u[1]-w[1]*u[0])/den
                    distance=path.s[i]+t*(path.s[i+1]-path.s[i])-origin
                    if 0<=t<=1 and 0<=q<=1 and -1<=distance<=horizon:
                        contexts.append((distance,ll.tl_groups,ll.tags.get('turn_direction','unknown'),lid))
                        break
        return contexts

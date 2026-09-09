#!/usr/bin/env python3
"""차선 그래프 A* 계획기 — 전체 기동 시퀀스를 한 번에 낸다 (1단계: 계획만, 발행 없음).

배경 (2026-09-08 실주행 측정):
  Autoware 는 "바로 옆 한 차선"만 보고, "앞으로 어느 차선이 통행 가능한가"를 만드는
  구성요소가 없다. 그래서 1·2·3차선이 모두 막히고 4·5차선이 빈 상황을 아무도 모른다.
  또 좌회전 포켓은 선행 lanelet 이 없어 차선변경으로만 진입 가능한데,
  preferred_primitive 는 후속 링크로 이어지는 차선만 표현할 수 있어 원리적으로 지정 불가다.

이 노드가 하는 일:
  - 경로 세그먼트로 차선 회랑을 만들고 종방향으로 잘게 나눈다
  - 각 객체의 예측 경로를 (lanelet, 종방향, 시각) 점유로 펼친다
  - 목표(= 경로의 다음 필수 회전에 도달 가능한 차선 집합)까지 A* 로 전체 시퀀스를 찾는다
  - 결정은 이벤트에서만. 매 주기는 검증만 하고 깨질 때만 다시 찾는다
"""
import math
import heapq
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from autoware_planning_msgs.msg import LaneletRoute
from autoware_perception_msgs.msg import PredictedObjects
from nav_msgs.msg import Odometry
from autoware_internal_planning_msgs.msg import VelocityLimit
from autoware_planning_msgs.srv import SetPreferredPrimitive
from autoware_planning_msgs.msg import LaneletPrimitive
from tier4_planning_msgs.msg import RerouteAvailability
from std_msgs.msg import String
from autoware_internal_debug_msgs.msg import StringStamped

from .osm_map import OsmMap

CELL = 2.0              # [m] 종방향 격자
LC_PENALTY = 8.0        # 차선변경 1회의 추가 비용 [m 환산]
LAT_SHIFT = 3.2         # [m] 차선 간격 (실측)
OBJ_MARGIN = 6.0        # [m] 객체 앞뒤 여유 (차체 + 정지 여유)
# 경로 차선을 벗어나 있는 동안 셀마다 무는 비용 [m 환산].
# 이게 없으면 '거리 + 차선변경 횟수'만 보므로 언제 복귀하든 비용이 같아, A* 가 복귀를
# 회랑 끝까지 미룬다. 그러면 마지막 세그먼트(17.9 m)에서 포켓 진입까지 하려다 자리가 없다
# (실측 2026-09-08: 14925(B,seg4) → 14633(A,seg5) → 14611(P,seg5) 로 계획해 실패).
# 막힌 셀은 애초에 통행 불가라 이 비용이 우회 자체를 막지는 않는다. 복귀 시점만 앞당긴다.
OFF_ROUTE = 0.6


class LanePlanner(Node):
    def __init__(self):
        super().__init__('lane_planner')
        self.declare_parameter('map_osm', '')
        self.declare_parameter('lat_accel', 2.0)      # 차선변경 횡가속 [m/s^2]
        self.declare_parameter('plan_horizon', 250.0) # [m]
        osm = self.get_parameter('map_osm').value
        self.omap = OsmMap(osm, self.get_logger()) if osm else None
        self.lat_accel = float(self.get_parameter('lat_accel').value)
        # 모듈의 준비시간 가정 [s]. 깜빡이 누적 전 1.0 s 로 보수적으로 본다(누적되면 0.5 s).
        self.declare_parameter('lc_prepare_s', 1.0)
        self.lc_prepare_s = float(self.get_parameter('lc_prepare_s').value)
        self.horizon = float(self.get_parameter('plan_horizon').value)

        self.route = None
        self.ego = None
        self.objs = None
        self.last_key = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LaneletRoute, '/planning/mission_planning/route', self.on_route, latched)
        self.create_subscription(Odometry, '/localization/kinematic_state', self.on_odom, 1)
        self.create_subscription(PredictedObjects, '/perception/object_recognition/objects',
                                 self.on_objs, 1)
        # 단계 재라우팅: 다음 '한 칸'만 루트 preferred 로 요구하고, 완료되면 다음 단계로 넘어간다.
        # 모듈은 남은 변경 전부가 종점 안에 들어가야 첫 변경을 시작하므로(calc_lc_length_and_dist_buffer),
        # 두 칸을 한꺼번에 요구하면 첫 칸조차 시작하지 않는다(2026-09-08 실측).
        # set_preferred_primitive 는 preferred 만 바꾸고 primitives·uuid 는 그대로 둔다.
        # change_route 와 달리 check_reroute_safety 를 호출하지 않아
        # "New route is not safe" 로 거부되지 않는다(mission_planner.cpp:384-420).
        # reset=true 로 원래 경로 복원까지 지원한다(mission_planner 가 original_route_ 를 보관).
        self.cli_pref = self.create_client(
            SetPreferredPrimitive,
            '/planning/mission_planning/mission_planner/set_preferred_primitive')
        self.create_subscription(
            RerouteAvailability,
            '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/'
            'output/is_reroute_available',
            lambda m: setattr(self, 'reroute_ok', m.availability), 1)
        self.reroute_ok = False
        # 차선변경이 실행 중인 동안에는 루트를 바꾸지 않는다.
        # is_reroute_available 만으로는 틈이 있다 — 실측 2026-09-08: lane_change_right 가
        # RUNNING 인 중에 복귀 요구가 통과했고, 실행 중이던 모듈의 목표 차선이 preferred 에서
        # 빠지면서 그 모듈이 승인 슬롯을 15.7초 붙들었다. 그동안 lane_change_left 는 후보로만
        # 떠 있다가 승인을 못 받았고, 자차는 33 m 를 그냥 흘려보냈다.
        self.lc_running = False
        self.create_subscription(
            StringStamped,
            '/planning/scenario_planning/lane_driving/behavior_planning/'
            'behavior_path_planner/debug/internal_state',
            self.on_internal_state, 1)
        self.orig_preferred = None   # 현재 루트의 원래 preferred (복귀 기준)
        self.route_uuid = None       # 루트가 바뀌면 보관본을 새로 잡는다
        self.demand_now = None       # 지금 적용 중인 요구
        self.demand_pending = None   # 재라우팅 가용해질 때까지 대기 중인 요구
        # 진행 중인 한 수. 이걸 잡으면 끝날 때까지 요구를 바꾸지 않는다.
        # 안 그러면 객체 속도 잔떨림·자차 가속으로 A* 해가 매 주기 바뀌고(A→B→C),
        # 그때마다 change_route 를 새로 밀어 모듈이 하던 일을 리셋한다
        # (2026-09-08 VTD 실주행: 같은 자차 위치에서 계획 4회·요구 3회·속도상한 2회).
        self.committed = None        # (목표 lanelet, 그 demand)
        # 약속할 당시 '비어 있다고 보고' 계획에 넣은 차선들. GT 인지 범위(80 m) 밖의
        # 정지차는 계획 시점에 안 보인다 — 이 중 하나가 나중에 막히면 약속을 깬다.
        # 차량들은 순차적으로 시야에 들어오므로, 다음 한 수의 목표 차선만 보면
        # 계획 뒤쪽 차선이 늦게 드러나는 경우를 놓친다. 계획 전체를 본다.
        self.committed_free_lanes = frozenset()
        self.inflight = False        # 서비스 응답 대기 중 — 겹쳐 쏘지 않는다
        self.pub_state = self.create_publisher(String, '/decision/state', 10)

        # 속도 상한: 현재 속도로 해가 없고 더 느리면 있으면, 그 속도를 걸어 계획을 성립시킨다.
        self.declare_parameter('speed_candidates', [8.0, 6.0, 4.0, 3.0, 2.0])
        self.declare_parameter('speed_margin', 0.8)   # 계획 속도에 곱해 상한으로 건다
        self.v_cands = [float(x) for x in self.get_parameter('speed_candidates').value]
        self.v_margin = float(self.get_parameter('speed_margin').value)
        # 선택기가 transient_local 로 구독한다(external_velocity_limit_selector_node.cpp:127).
        # 기본 volatile 로 발행하면 전달되지 않는다.
        self.pub_vlim = self.create_publisher(
            VelocityLimit, '/planning/scenario_planning/max_velocity_candidates',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.vlim_now = None

        self.ev_key = None      # 마지막으로 계획한 시점의 이벤트 지문
        self.create_timer(0.1, self.tick)   # 판단 지연은 한 주기(100ms). 계산은 이벤트일 때만 한다.
        self.get_logger().info('lane_planner 시작 (A* + 단계 재라우팅)')

    def on_internal_state(self, m):
        """승인 풀에 lane_change 계열이 있으면 실행 중으로 본다."""
        line = next((l for l in m.data.splitlines() if 'approved modules' in l), '')
        self.lc_running = 'lane_change' in line

    def on_route(self, m):
        """루트가 바뀌면(리스폰 재주입 포함) 보관본과 진행 상태를 전부 새로 잡는다.

        리스폰이 나면 route_node 가 change_route_points 로 경로를 다시 넣는데, 자차 위치가
        달라져 세그먼트 수도 달라진다. 예전에는 orig_preferred 를 최초 한 번만 저장해서,
        재주입 뒤 모든 요구가
          "The size of preferred_primitives (21) is different from that of the current route"
        로 거부됐다 — 판단이 통째로 죽는다(2026-09-08 VTD 실주행에서 확인).
        """
        new_uuid = bytes(m.uuid.uuid)
        changed = (self.route is None) or (new_uuid != self.route_uuid)
        self.route = m
        self.last_key = None
        if changed:
            self.route_uuid = new_uuid
            self.orig_preferred = [sg.preferred_primitive.id for sg in m.segments]
            self.demand_now = None
            self.demand_pending = None
            self.committed = None
            self.committed_free_lanes = frozenset()
            self.get_logger().info(
                f'경로 갱신 → 보관본 재설정: preferred {len(self.orig_preferred)} 세그먼트')
            # A* 회랑은 corridor() 가 route primitives 로만 만든다. 우측 차선이
            # 여기 없으면 우회 해가 원리적으로 없다 — 실제 범위를 한 번 찍어 확정한다.
            for i, sg in enumerate(m.segments):
                ids = [p.id for p in sg.primitives]
                self.get_logger().info(
                    f'  세그먼트 primitives seg{i}: {ids} (preferred {sg.preferred_primitive.id})')
    def on_objs(self, m): self.objs = m
    def on_odom(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.ego = (p.x, p.y, yaw, m.twist.twist.linear.x)

    def seg_index_of(self, lid):
        for i, seg in enumerate(self.route.segments):
            if any(p.id == lid for p in seg.primitives):
                return i
        return None

    # ---------------------------------------------------------------- 회랑
    def corridor(self, ego_seg):
        """[(seg_index, [lanelet ...왼→오], 세그먼트 길이)] 를 horizon 까지."""
        out, total = [], 0.0
        for i in range(ego_seg, len(self.route.segments)):
            lanes = [p.id for p in self.route.segments[i].primitives]
            lanes = [l for l in lanes if l in self.omap.lanelets]
            if not lanes:
                break
            ln = self.omap.lanelets[lanes[0]].length
            out.append((i, lanes, ln))
            total += ln
            if total > self.horizon:
                break
        return out

    def must_reach(self, corr):
        """뒤에서부터: 후속 링크만으로 회랑 끝에 닿는 lanelet 집합 (세그먼트별)."""
        sets = [set() for _ in corr]
        sets[-1] = set(corr[-1][1])
        for k in range(len(corr) - 2, -1, -1):
            nxt = sets[k + 1]
            sets[k] = {l for l in corr[k][1]
                       if any(s in nxt for s in self.omap.successors(l))}
            if not sets[k]:
                sets[k] = set(corr[k][1])   # 끊기면 제약을 걸지 않는다
        return sets

    # ---------------------------------------------------------------- 점유
    def blocked(self, corr=None):
        """{(lanelet, cell): 가장 이른 점유 시각}. 정지 객체는 t=0."""
        occ = {}
        if self.objs is None:
            return occ
        for o in self.objs.objects:
            k = o.kinematics
            paths = list(getattr(k, 'predicted_paths', []))
            pts = [(0.0, k.initial_pose_with_covariance.pose.position)]
            if paths:
                step = paths[0].time_step.sec + paths[0].time_step.nanosec * 1e-9
                pts = [(i * step, q.position) for i, q in enumerate(paths[0].path)]
            for t, p in pts:
                lid = self.omap.nearest_lanelet(p.x, p.y)
                if lid is None:
                    continue
                _, s, _ = self.omap.project(lid, p.x, p.y)
                lo = int(max(0.0, s - OBJ_MARGIN) // CELL)
                hi = int((s + OBJ_MARGIN) // CELL)
                for c in range(lo, hi + 1):
                    key = (lid, c)
                    if key not in occ or t < occ[key]:
                        occ[key] = t
        return occ

    def blocked_lane_ids(self):
        """정지 객체(t<=0)가 점유한 lanelet 집합. 약속 파기 판단용."""
        if self.objs is None or self.omap is None:
            return frozenset()
        return frozenset(l for (l, _c), t in self.blocked().items() if t <= 0.0)

    # ---------------------------------------------------------------- A*
    def plan(self, corr, start_lane, start_s, v):
        reach = self.must_reach(corr)
        occ = self.blocked(corr)
        seg_of = {}
        for idx, (i, lanes, ln) in enumerate(corr):
            for j, l in enumerate(lanes):
                seg_of[l] = (idx, j, ln)
        # 차선변경 1회에 필요한 종방향 길이. **모듈이 실제로 쓰는 값과 맞춰야 한다.**
        #   모듈: 준비거리(prepare_duration x v) + 횡이동거리
        #   준비시간은 깜빡이 누적으로 max_prepare_duration(4.0s) 에서 min(0.5s) 까지 줄어들고,
        #   횡이동은 실측 7.6 m 로 일정했다([HLFMA-A], 2026-09-08).
        #   이전 모델 v*2.0*sqrt(LAT_SHIFT/lat_accel) = 2.53v 는 v=13.9 에서 35 m 를 요구했는데
        #   모듈의 실제 요구는 12 m 였다 — 3배 과대평가다. 그 탓에 "이 속도로는 안 된다"고
        #   잘못 판단해 4.8 m/s 상한을 걸었고, 저속이 다시 기동을 늦추는 악순환이 됐다.
        #   깜빡이가 아직 안 쌓인 초반을 감안해 준비시간을 1.0 s 로 보고 여유 3 m 를 둔다.
        lc_len = max(9.0, self.lc_prepare_s * v + LAT_SHIFT * 2.4 + 3.0)
        lc_cells = max(1, int(math.ceil(lc_len / CELL)))

        def free(lid, c, t):
            e = occ.get((lid, c))
            return e is None or e > t + 1.0

        # 목표: '통과'가 아니라 '올바른 차선 사슬에 진입'. 대기열이 정지선까지 차 있으면
        # 끝까지 통과하는 경로는 원래 없다. 뒤에서부터 좁혀진 첫 세그먼트에 들어가면 성공이다.
        target_idx = None
        for k, (i, lanes, ln) in enumerate(corr):
            if reach[k] and set(reach[k]) != set(lanes):
                target_idx = k
                break
        if target_idx is None:
            target_idx = len(corr) - 1

        # 경로가 원래 가리키던 차선 집합. 여기를 벗어나 있으면 셀마다 OFF_ROUTE 를 문다.
        on_route = set(self.orig_preferred or
                       [sg.preferred_primitive.id for sg in self.route.segments])

        start = (start_lane, int(start_s // CELL))
        openq = [(0.0, 0.0, start, None)]
        best, came = {start: 0.0}, {}
        goal = None
        while openq:
            f, g, node, prev = heapq.heappop(openq)
            if node in came and best.get(node, 1e18) < g:
                continue
            came[node] = prev
            lid, c = node
            idx, j, ln = seg_of[lid]
            t_here = g / max(1.0, v)
            # 목표는 '완주'다. 대기열 뒤에 서는 것은 우회가 아니므로 회랑 끝 도달을 요구한다.
            if idx == len(corr) - 1 and (not reach[idx] or lid in reach[idx]):
                goal = node; break
            ncell = int(ln // CELL)
            step = CELL + (0.0 if lid in on_route else OFF_ROUTE)
            # 직진
            if c + 1 <= ncell:
                nn = (lid, c + 1)
                if free(lid, c + 1, t_here) and g + step < best.get(nn, 1e18):
                    best[nn] = g + step
                    heapq.heappush(openq, (g + step, g + step, nn, node))
            elif idx + 1 < len(corr):
                for s2 in self.omap.successors(lid):
                    if s2 in seg_of:
                        nn = (s2, 0)
                        s2step = CELL + (0.0 if s2 in on_route else OFF_ROUTE)
                        if free(s2, 0, t_here) and g + s2step < best.get(nn, 1e18):
                            best[nn] = g + s2step
                            heapq.heappush(openq, (g + s2step, g + s2step, nn, node))
            # 차선변경. 이 맵은 lanelet 이 10~32m 로 짧아 변경 구간이 거의 항상 경계를 넘는다.
            # 원·목표 차선을 각각 후속을 따라 lc_cells 만큼 걸어가며 양쪽 모두 비어 있는지 본다.
            lanes = corr[idx][1]
            for dj in (-1, 1):
                if not (0 <= j + dj < len(lanes)):
                    continue
                a = self._walk(seg_of, lid, c, lc_cells, t_here, free)
                b = self._walk(seg_of, lanes[j + dj], c, lc_cells, t_here, free)
                if a is None or b is None:
                    continue
                nn = b[0]
                cost = g + lc_cells * CELL + LC_PENALTY
                if cost < best.get(nn, 1e18):
                    best[nn] = cost
                    heapq.heappush(openq, (cost, cost, nn, node))
        if goal is None:
            # 진단: 회랑의 어느 차선 어느 구간이 정지 객체로 막혀 사슬이 끊겼는가.
            # 목표 조건이 '회랑 끝 도달'이므로, 한 군데만 끊겨도 전체가 실패한다.
            rows = []
            for k, (i, lanes, ln) in enumerate(corr[:6]):
                cells = []
                for l in lanes:
                    cs = sorted(c for (ll, c), tt in occ.items() if ll == l and tt <= 0.0)
                    cells.append('%d%s' % (l, '' if not cs else
                                 '<%.0f~%.0f막힘>' % (cs[0]*CELL, cs[-1]*CELL)))
                rows.append('seg%d[%s]reach=%s' % (i, ' '.join(cells), sorted(reach[k])))
            self.get_logger().warning('[계획진단] ' + ' | '.join(rows))
            self.get_logger().warning(
                '[계획진단] 회랑 %d세그(horizon %.0fm), 탐색노드 %d개, LC소요 %.0fm, '
                '목표=마지막세그(idx %d) / 미사용 target_idx=%d'
                % (len(corr), self.horizon, len(best), lc_len, len(corr)-1, target_idx))
            return None, lc_len
        path, n = [], goal
        while n is not None:
            path.append(n); n = came.get(n)
        return list(reversed(path)), lc_len

    def _walk(self, seg_of, lid, c, ncells, t, free):
        """lid 의 셀 c 에서 후속을 따라 ncells 만큼 전진. 지나는 셀이 모두 비면 (끝 노드,) 반환."""
        cur, cc, left = lid, c, ncells
        if not free(cur, cc, t):
            return None
        while left > 0:
            if cur not in seg_of:
                return None
            ln = seg_of[cur][2]
            ncell = int(ln // CELL)
            step = min(left, ncell - cc)
            for k in range(1, step + 1):
                if not free(cur, cc + k, t):
                    return None
            cc += step
            left -= step
            if left == 0:
                break
            nxt = [x for x in self.omap.successors(cur) if x in seg_of]
            if not nxt:
                return None
            cur, cc = nxt[0], 0
            if not free(cur, 0, t):
                return None
        return ((cur, cc),)

    def set_vlim(self, vlim, plan_v):
        if self.vlim_now is not None and abs(self.vlim_now - vlim) < 0.3:
            return
        m = VelocityLimit()
        m.stamp = self.get_clock().now().to_msg()
        m.sender = 'lane_planner'
        m.max_velocity = float(vlim)
        self.pub_vlim.publish(m)
        self.vlim_now = vlim
        self.get_logger().info(
            f'속도 상한 {vlim:.1f} m/s 설정 (계획은 {plan_v:.1f} m/s 에서 성립)')

    def clear_vlim(self):
        if self.vlim_now is None:
            return
        m = VelocityLimit()
        m.stamp = self.get_clock().now().to_msg()
        m.sender = 'lane_planner'
        m.max_velocity = 100.0
        self.pub_vlim.publish(m)
        self.vlim_now = None
        self.get_logger().info('속도 상한 해제')

    # ---------------------------------------------------------------- 단계 재라우팅
    def build_demand(self, path, changes):
        """'다음 한 칸만' 요구하는 preferred 배열. 변경이 없으면 원래 preferred 그대로.

        changes[0] = (from, to) 가 다음 한 칸이다. to 가 속한 세그먼트부터는 to 의 후속 체인을
        유지해 그 뒤로는 아무 변경도 요구하지 않는다. 모듈은 남은 변경 전부가 종점 안에 들어가야
        첫 변경을 시작하므로(calc_lc_length_and_dist_buffer), 두 칸을 한꺼번에 요구하면 안 된다.

        세그먼트 안에서 일어나는 변경(예: 15379->15414 는 둘 다 seg2)도 잡아야 하므로
        세그먼트당 '마지막' lanelet 을 쓴다.
        """
        segs = self.route.segments
        demand = list(self.orig_preferred)
        if not changes:
            return demand, None
        f0, t0, _ = changes[0]

        # 변경 지점까지: path 를 따라가며 세그먼트별 lanelet (마지막 것이 이긴다)
        for lid, _ in path:
            if lid == t0:
                break
            i = self.seg_index_of(lid)
            if i is not None:
                demand[i] = lid

        # 변경 이후: to 의 후속 체인을 유지
        start = self.seg_index_of(t0)
        if start is None:
            return demand, (f0, t0)
        self.follow_from(demand, start, t0)
        return demand, (f0, t0)

    def apply_demand(self, demand, reset=False):
        """preferred 만 교체한다. primitives 는 건드리지 않아 차선변경 가능성이 유지된다."""
        if not reset and demand == self.demand_now:
            return True
        if self.inflight:
            return False                 # 앞선 요청의 응답을 기다린다 (덮어쓰기 방지)
        if not self.reroute_ok or self.lc_running:
            # 실행 중인 차선변경이 끝난 뒤에 넣는다. 보류분은 매 주기 재시도한다.
            self.demand_pending = demand
            return False
        if not self.cli_pref.service_is_ready():
            self.get_logger().warning('set_preferred_primitive 서비스 미준비',
                                      throttle_duration_sec=5.0)
            return False
        req = SetPreferredPrimitive.Request()
        req.uuid = self.route.uuid
        req.reset = reset
        if not reset:
            req.preferred_primitives = [
                LaneletPrimitive(id=int(v), primitive_type='lane') for v in demand]
        changed = [(i, a, b) for i, (a, b) in enumerate(zip(
            [sg.preferred_primitive.id for sg in self.route.segments], demand)) if a != b]
        self.inflight = True
        fut = self.cli_pref.call_async(req)

        def done(f, changed=changed, reset=reset):
            # 응답을 반드시 확인한다. RTC 때 조용히 실패해 하루를 썼다(2026-09-08).
            self.inflight = False
            try:
                r = f.result()
            except Exception as e:
                self.get_logger().error(f'set_preferred_primitive 예외: {e!r}')
                self.demand_now = None
                self.committed = None
                return
            if r.status.success:
                if reset:
                    self.get_logger().info('원래 경로로 복원')
                else:
                    self.get_logger().info(
                        f'단계 요구 반영: {len(changed)}개 세그먼트 '
                        + ', '.join(f'seg{i}:{a}→{b}' for i, a, b in changed[:4]))
            else:
                self.get_logger().warning(
                    f'set_preferred_primitive 거부 code={r.status.code} "{r.status.message}"')
                self.demand_now = None      # 조건이 바뀌는 폴링이므로 재시도는 정당
                self.committed = None

        fut.add_done_callback(done)
        self.demand_now = demand
        self.demand_pending = None
        return True

    def follow_from(self, demand, seg_idx, lane):
        """seg_idx 부터 lane 의 후속 체인을 demand 에 채운다. 그 뒤로는 아무 변경도 요구하지 않게 된다."""
        segs = self.route.segments
        cur = lane
        for i in range(seg_idx, len(segs)):
            ids = [p.id for p in segs[i].primitives]
            if cur not in ids:
                break
            demand[i] = cur
            if i + 1 >= len(segs):
                break
            nxt = [x for x in self.omap.successors(cur)
                   if x in [p.id for p in segs[i + 1].primitives]]
            if not nxt:
                break
            cur = nxt[0]
        return demand

    def step_toward(self, cur, goal, seg_idx):
        """같은 세그먼트 안에서 cur 에서 goal 쪽으로 '한 칸'. 없으면 None."""
        ids = [p.id for p in self.route.segments[seg_idx].primitives]
        if cur == goal or cur not in ids or goal not in ids:
            return None
        for side in (0, 1):                      # 0=왼쪽, 1=오른쪽
            first = self.omap.neighbors(cur)[side]
            n, guard = cur, 0
            while guard < 8:
                nb = self.omap.neighbors(n)[side]
                if nb is None or nb not in ids:
                    break
                if nb == goal:
                    return first
                n = nb
                guard += 1
        return None

    def restore_route(self):
        """우회 요구를 걷고 원래 경로로 되돌린다 — 단, **한 칸씩**.

        원래 경로를 통째로 되돌리면, 자차가 두 칸 벗어나 있을 때(예: B 에 있는데 목표가
        좌회전 포켓) 모듈에 두 칸을 한꺼번에 요구하게 된다. 그러면 남은 변경 전부가 종점 안에
        들어가야 첫 변경을 시작하는 성질(calc_lc_length_and_dist_buffer) 때문에 첫 칸조차
        시작하지 않는다 — 2026-09-08 실측으로 확인한 문제다.
        그래서 자차 위치에서 원래 차선 쪽으로 한 칸만 요구하고, 그 뒤로는 그 차선의 후속을 따른다.
        다음 주기에 다시 불려 한 칸 더 좁힌다.
        """
        if self.orig_preferred is None or self.demand_now is None or self.ego is None:
            return
        orig = list(self.orig_preferred)
        if self.demand_now == orig:
            return
        x, y, yaw, _ = self.ego
        lid = self.omap.nearest_lanelet(x, y, yaw)
        i = self.seg_index_of(lid) if lid is not None else None
        if lid is None or i is None:
            self.apply_demand(orig)          # 자차 위치를 못 잡으면 통째로 (기존 동작)
            return
        goal = orig[i]
        if lid == goal:
            self.apply_demand(orig)          # 이미 원래 차선 위 — 나머지도 원래대로
            return
        step = self.step_toward(lid, goal, i)
        if step is None:
            self.apply_demand(orig)          # 이웃 관계로 판단 불가 → 통째로
            return
        demand = list(orig)
        self.follow_from(demand, i, step)
        self.apply_demand(demand)

    def publish_state(self, **kw):
        try:
            self.pub_state.publish(String(data='; '.join(f'{k}={v}' for k, v in kw.items())))
        except Exception:
            pass

    # ---------------------------------------------------------------- tick
    def event_key(self):
        """이벤트 지문. 이 값이 바뀔 때만 A* 를 다시 돈다.
        - 객체: 등장·소멸, 속도가 0.5 m/s 이상 변화, 소속 차선 변화(끼어들기)
        - 자차: 점유 차선 변화
        위치 변화는 넣지 않는다. 움직이는 객체마다 매 주기 발동해 진동을 만든다."""
        if self.objs is None or self.ego is None:
            return None
        x, y, yaw, _ = self.ego
        ego_lane = self.omap.nearest_lanelet(x, y, yaw)
        items = []
        for o in self.objs.objects:
            k = o.kinematics
            p = k.initial_pose_with_covariance.pose.position
            vv = k.initial_twist_with_covariance.twist.linear.x
            lid = self.omap.nearest_lanelet(p.x, p.y)
            items.append((bytes(o.object_id.uuid)[:4], lid, round(vv / 0.5)))
        return (ego_lane, tuple(sorted(items)))

    def tick(self):
        if self.route is None or self.ego is None or self.omap is None:
            return
        # 보류된 요구 재시도. 저장만 하고 재시도하지 않으면, 첫 계획 때 재라우팅이 불가능한 경우
        # (engage 전 등) 그 요구가 영영 버려진다 — 실제로 그래서 요구가 늦게 적용됐다(2026-09-08).
        if self.demand_pending is not None and self.reroute_ok and not self.lc_running:
            self.apply_demand(self.demand_pending)
        key = self.event_key()
        if key is not None and key == self.ev_key:
            return          # 이벤트 없음 → 재계획하지 않는다
        self.ev_key = key
        x, y, yaw, v = self.ego
        lid = self.omap.nearest_lanelet(x, y, yaw)
        if lid is None:
            return
        # 진행 중인 한 수가 있으면 도달할 때까지 요구를 바꾸지 않는다.
        # 도달 판정은 자차 lanelet 이 목표이거나 그 후속인지로 본다.
        if self.committed is not None:
            goal_lane, held = self.committed
            if lid == goal_lane or lid in self.omap.successors(goal_lane):
                self.get_logger().info(f'한 수 완료: {goal_lane} 도달')
                self.committed = None
            elif self.committed_free_lanes & self.blocked_lane_ids():
                # 계획에 넣을 당시 비어 있던 차선이 새로 보인 정지 객체로 막혔다.
                # 그대로 지키면 막힌 차선으로 들어간다 — 즉시 재계획한다.
                # (실측 2026-09-09: 계획 3회가 전부 출발 lanelet 16249 에서 났고,
                #  정지차는 200 m 앞이라 안 보였다. 보인 뒤에도 약속 때문에 재계획이
                #  막혀 막힌 차선으로 계속 갔다.)
                hit = sorted(self.committed_free_lanes & self.blocked_lane_ids())
                self.get_logger().warning(
                    f'약속 파기: 계획 차선 {hit} 이(가) 정지 객체로 막혔다 '
                    f'(다음 한 수 {goal_lane}) → 재계획')
                self.committed = None
                self.last_key = None      # 같은 계획이라도 다시 적용되게 한다
            else:
                if self.demand_now != held:
                    self.apply_demand(held)      # 덮어써졌으면 되돌린다
                return
        ego_seg = next((i for i, s in enumerate(self.route.segments)
                        if any(p.id == lid for p in s.primitives)), None)
        if ego_seg is None:
            return
        corr = self.corridor(ego_seg)
        if len(corr) < 2:
            # 회랑이 남지 않았다(마지막 다차선 구간에 들어섰다). 여기서 조용히 포기하면
            # 우회로 바꿔 둔 요구가 그대로 남아 원래 경로(좌회전 포켓)로 못 돌아온다
            # — 실측 2026-09-08: seg5 진입 후 재계획이 멈춰 포켓이 한 번도 요구되지 않았다.
            # 남은 구간에는 계획할 것이 없으므로 원래 경로를 복원한다.
            self.restore_route()
            return
        _, s0, _ = self.omap.project(lid, x, y)
        path, lc_len = self.plan(corr, lid, s0, max(v, 2.0))
        plan_v = None
        if path is None:
            # 현재 속도로 안 되면 더 느린 속도에서 찾아본다. 되면 그 속도를 상한으로 건다.
            for vc in self.v_cands:
                if vc >= v:
                    continue
                p2, lc2 = self.plan(corr, lid, s0, vc)
                if p2 is not None:
                    path, lc_len, plan_v = p2, lc2, vc
                    break
        if path is not None and plan_v is not None:
            self.set_vlim(plan_v * self.v_margin, plan_v)
        elif path is not None:
            self.clear_vlim()
        if path is None:
            # 우회 시퀀스를 못 찾았다. 바꿔 둔 요구를 그대로 두면 원래 경로(예: 좌회전 포켓)로
            # 영영 못 돌아온다 — 실측 2026-09-08: seg5 에서 계획이 실패한 뒤 포켓이 한 번도
            # 요구되지 않아 직진 차선에서 정지선까지 갔다.
            # 계획할 것이 없으면 경로대로 가는 것이 맞다. stock 모듈에 맡긴다.
            self.restore_route()
            key = ('none', lid)
            if key != self.last_key:
                self.last_key = key
                self.get_logger().warning(
                    f'[계획 실패] 자차 {lid} 에서 목표까지 통행 가능한 시퀀스 없음 '
                    f'(속도 {v:.1f} m/s, 차선변경 소요 {lc_len:.0f} m) → 원래 경로로 복원')
            return
        changes = []
        for a, b in zip(path, path[1:]):
            if a[0] != b[0] and b[0] not in self.omap.successors(a[0]):
                changes.append((a[0], b[0], a[1] * CELL))
        key = tuple(c[:2] for c in changes) + (lid,)
        if key == self.last_key:
            return
        self.last_key = key
        if self.orig_preferred is not None:
            demand, step = self.build_demand(path, changes)
            ok = self.apply_demand(demand)
            if ok and step is not None:
                self.committed = (step[1], list(demand))   # step=(from,to)
                # 계획 경로가 지나는 모든 차선 중, 지금 비어 있다고 본 것만 기록한다.
                # 이미 막힌 줄 알고도 A* 가 고른 차선은 감안된 것이므로 파기 근거가 아니다.
                self.committed_free_lanes = (
                    frozenset(a[0] for a in path) - self.blocked_lane_ids())
            self.publish_state(
                ego_lane=lid, v=f'{v:.1f}', 남은변경=len(changes),
                다음한칸=(f'{step[0]}→{step[1]}' if step else '없음'),
                재라우팅=('적용' if ok else
                       ('보류(차선변경 실행 중)' if self.lc_running else '보류(재라우팅 불가)')))

        if changes:
            desc = ' → '.join(f'{f}→{t}' for f, t, _ in changes)
            self.get_logger().info(
                f'[계획] 자차 {lid} v={v:.1f}  차선변경 {len(changes)}회: {desc} '
                f'(회당 {lc_len:.0f} m)')
        else:
            self.get_logger().info(f'[계획] 자차 {lid} 직진 유지 (차선변경 불필요)')


def main():
    rclpy.init()
    n = LanePlanner()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

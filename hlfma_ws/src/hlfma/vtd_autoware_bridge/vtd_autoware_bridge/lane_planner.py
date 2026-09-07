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
from tier4_rtc_msgs.msg import CooperateStatusArray, CooperateCommand, Command
from tier4_rtc_msgs.srv import CooperateCommands
from autoware_internal_planning_msgs.msg import VelocityLimit

from .osm_map import OsmMap

CELL = 2.0              # [m] 종방향 격자
LC_PENALTY = 8.0        # 차선변경 1회의 추가 비용 [m 환산]
LAT_SHIFT = 3.2         # [m] 차선 간격 (실측)
OBJ_MARGIN = 6.0        # [m] 객체 앞뒤 여유 (차체 + 정지 여유)


class LanePlanner(Node):
    def __init__(self):
        super().__init__('lane_planner')
        self.declare_parameter('map_osm', '')
        self.declare_parameter('lat_accel', 2.0)      # 차선변경 횡가속 [m/s^2]
        self.declare_parameter('plan_horizon', 250.0) # [m]
        self.declare_parameter('publish', False)      # 1단계: 계획만
        osm = self.get_parameter('map_osm').value
        self.omap = OsmMap(osm, self.get_logger()) if osm else None
        self.lat_accel = float(self.get_parameter('lat_accel').value)
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
        # RTC: 계획의 첫 수가 우측이면 external_request_lane_change_right 를 승인한다.
        # 실측(2026-09-08): 우측 후보는 안전 판정을 통과하지만 cmd=0 으로 승인자가 없어 실행되지 않았다.
        self.declare_parameter('approve_rtc', True)
        self.approve_rtc = bool(self.get_parameter('approve_rtc').value)
        self.rtc_status = {}
        for side in ('left', 'right'):
            self.create_subscription(
                CooperateStatusArray, f'/planning/cooperate_status/external_request_lane_change_{side}',
                lambda m, s=side: self.rtc_status.__setitem__(s, m), 1)
        # RTC 승인 서비스는 모듈마다 따로 만들어진다:
        #   rtc_interface.cpp:142  cooperate_commands_namespace_ + "/" + name
        # 접미사 없는 '/planning/cooperate_commands' 로 잡으면 service_is_ready() 가 영원히 false 라
        # 승인이 한 번도 전송되지 않는다(2026-09-08 실측: 우측 후보 valid=1 인데 WAITING_APPROVAL 고착).
        self.rtc_cli = {
            side: self.create_client(
                CooperateCommands,
                f'/planning/cooperate_commands/external_request_lane_change_{side}')
            for side in ('left', 'right')
        }
        self.approved_uuid = set()

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
        self.get_logger().info('lane_planner 시작 (계획만 + RTC 승인)')

    def on_route(self, m): self.route = m; self.last_key = None
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
    def blocked(self, corr):
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

    # ---------------------------------------------------------------- A*
    def plan(self, corr, start_lane, start_s, v):
        reach = self.must_reach(corr)
        occ = self.blocked(corr)
        seg_of = {}
        for idx, (i, lanes, ln) in enumerate(corr):
            for j, l in enumerate(lanes):
                seg_of[l] = (idx, j, ln)
        lc_len = max(9.0, v * 2.0 * math.sqrt(LAT_SHIFT / self.lat_accel))  # 하한 실측 8.88m
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
            if idx == len(corr) - 1:
                goal = node; break
            ncell = int(ln // CELL)
            # 직진
            if c + 1 <= ncell:
                nn = (lid, c + 1)
                if free(lid, c + 1, t_here) and g + CELL < best.get(nn, 1e18):
                    best[nn] = g + CELL
                    heapq.heappush(openq, (g + CELL, g + CELL, nn, node))
            elif idx + 1 < len(corr):
                for s2 in self.omap.successors(lid):
                    if s2 in seg_of:
                        nn = (s2, 0)
                        if free(s2, 0, t_here) and g + CELL < best.get(nn, 1e18):
                            best[nn] = g + CELL
                            heapq.heappush(openq, (g + CELL, g + CELL, nn, node))
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

    def side_of(self, src, tgt):
        """src 기준 tgt 가 왼쪽인지 오른쪽인지. 세그먼트를 넘는 변경도 판정되도록 기하로 본다."""
        try:
            a = self.omap.lanelets[src]
            b = self.omap.lanelets[tgt]
        except Exception:
            return None
        ca, cb = a.center, b.center
        if len(ca) < 2 or not cb:
            return None
        i = len(ca) // 2
        hx, hy = ca[i + 1][0] - ca[i][0], ca[i + 1][1] - ca[i][1]
        n = math.hypot(hx, hy)
        if n < 1e-6:
            return None
        # 진행방향 기준 왼쪽 단위벡터
        lx, ly = -hy / n, hx / n
        j = len(cb) // 2
        d = (cb[j][0] - ca[i][0]) * lx + (cb[j][1] - ca[i][1]) * ly
        return 'left' if d > 0 else 'right'

    def send_rtc(self, side):
        """해당 방향 external_request 후보를 승인한다. 이미 보낸 uuid 는 건너뛴다."""
        msg = self.rtc_status.get(side)
        cli = self.rtc_cli.get(side)
        if msg is None or not msg.statuses or cli is None or not cli.service_is_ready():
            # 조용히 반환하면 승인이 안 되는 것을 관측할 수 없다 — 이유를 남긴다.
            why = ('상태 토픽 없음' if msg is None else
                   '후보 없음' if not msg.statuses else '서비스 미준비')
            self.get_logger().warning(f'RTC 승인 불가({side}): {why}', throttle_duration_sec=5.0)
            return
        reqs = []
        for st in msg.statuses:
            u = bytes(st.uuid.uuid)
            if u in self.approved_uuid:
                continue
            if not st.safe:
                self.get_logger().warning(
                    f'RTC 후보가 unsafe 로 표시됨({side}) — 승인 보류', throttle_duration_sec=5.0)
                continue
            reqs.append(st)
        if not reqs:
            return
        req = CooperateCommands.Request()
        req.stamp = self.get_clock().now().to_msg()
        for st in reqs:
            cc = CooperateCommand()
            cc.uuid = st.uuid
            cc.module = st.module
            cc.command.type = Command.ACTIVATE
            req.commands.append(cc)
            self.approved_uuid.add(bytes(st.uuid.uuid))
        self.get_logger().info(f'RTC 승인 전송: {side} {len(req.commands)}건')
        cli.call_async(req)

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
        key = self.event_key()
        if key is not None and key == self.ev_key:
            return          # 이벤트 없음 → 재계획하지 않는다
        self.ev_key = key
        x, y, yaw, v = self.ego
        lid = self.omap.nearest_lanelet(x, y, yaw)
        if lid is None:
            return
        ego_seg = next((i for i, s in enumerate(self.route.segments)
                        if any(p.id == lid for p in s.primitives)), None)
        if ego_seg is None:
            return
        corr = self.corridor(ego_seg)
        if len(corr) < 2:
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
            key = ('none', lid)
            if key != self.last_key:
                self.last_key = key
                self.get_logger().warning(
                    f'[계획 실패] 자차 {lid} 에서 목표까지 통행 가능한 시퀀스 없음 '
                    f'(속도 {v:.1f} m/s, 차선변경 소요 {lc_len:.0f} m)')
            return
        changes = []
        for a, b in zip(path, path[1:]):
            if a[0] != b[0] and b[0] not in self.omap.successors(a[0]):
                changes.append((a[0], b[0], a[1] * CELL))
        key = tuple(c[:2] for c in changes) + (lid,)
        if key == self.last_key:
            return
        self.last_key = key
        if changes and self.approve_rtc:
            # 첫 수의 방향을 판단해 해당 external_request 를 승인한다.
            f0, t0, _ = changes[0]
            side = self.side_of(f0, t0)   # 세그먼트를 넘는 변경도 있으므로 기하로 판정한다
            if side:
                self.send_rtc(side)

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

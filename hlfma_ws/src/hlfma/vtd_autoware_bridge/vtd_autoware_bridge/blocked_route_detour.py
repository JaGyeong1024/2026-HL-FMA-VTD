"""차선 단위 회피 판단 노드 (ROS 래퍼) — 판단은 detour_logic.DetourLogic, 이 파일은 변환만 한다.

역할 (todo0906 묶음 2)
  구독: 자차 odom, GT objects(PredictedObjects), behavior path_with_lane_id, mission route(latched),
        external_request 좌/우 cooperate_status, traffic_light planning_factors, 브리지 traffic_signals, /vtd/respawn
  판단: 0.2 s tick 마다 Inputs 를 만들어 DetourLogic.step → Decision
  출력: RTC CooperateCommands(ACTIVATE) — external_request_lane_change_{left,right} 만 (일반 차선변경은 RTC 를 켜지 않는다)
        VelocityLimit(max_velocity_candidates, sender 별) / VelocityLimitClearCommand
조향·RSS 안전 판정·취소는 Autoware lane_change 모듈이 한다. 이 노드가 죽으면 우회만 없어지고 정지·추종·필수 차선변경은 그대로다.

골격은 NG blocked_route_detour(경로 투영·RTC·속도제한·factor 게이트)를 재사용했고 판단 규칙은 교체했다 (adv_bundle2 D3·D5·D6·D7).
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Empty, String
from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import (
    PredictedObjects, ObjectClassification, TrafficLightGroupArray, TrafficLightElement)
from autoware_planning_msgs.msg import LaneletRoute
from autoware_internal_planning_msgs.msg import (
    PathWithLaneId, VelocityLimit, VelocityLimitClearCommand, PlanningFactor, PlanningFactorArray)
from tier4_rtc_msgs.msg import CooperateStatusArray, CooperateCommand, Command, State, Module
from tier4_rtc_msgs.srv import CooperateCommands

from .osm_map import OsmMap
from .tl_router import TrafficLightRouter
from .detour_logic import (
    DetourLogic, Params, EgoState, ObjectInfo, LaneInfo, SignalInfo, CandidateInfo, Inputs, LEFT, RIGHT)

_CLASS_NAME = {
    ObjectClassification.CAR: 'CAR', ObjectClassification.TRUCK: 'TRUCK', ObjectClassification.BUS: 'BUS',
    ObjectClassification.TRAILER: 'TRAILER', ObjectClassification.MOTORCYCLE: 'MOTORCYCLE',
    ObjectClassification.BICYCLE: 'BICYCLE', ObjectClassification.PEDESTRIAN: 'PEDESTRIAN',
}
_STATE_NAME = {State.WAITING_FOR_EXECUTION: 'WAITING', State.RUNNING: 'RUNNING', State.ABORTING: 'ABORTING',
               State.SUCCEEDED: 'SUCCEEDED', State.FAILED: 'FAILED'}
_EXT_MODULE = {LEFT: Module.EXT_REQUEST_LANE_CHANGE_LEFT, RIGHT: Module.EXT_REQUEST_LANE_CHANGE_RIGHT}
_MODULE_TOPIC = {LEFT: 'external_request_lane_change_left', RIGHT: 'external_request_lane_change_right'}


def qyaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PathFrame:
    """behavior path 위 투영: (부호 있는 횡거리[+좌], 호길이, 좌측 법선) 을 준다."""

    def __init__(self, points):
        self.segs = []
        arc = 0.0
        for a, b in zip(points, points[1:]):
            dx, dy = b.x - a.x, b.y - a.y
            n = math.hypot(dx, dy)
            if n > 1e-6:
                self.segs.append((a.x, a.y, dx, dy, n, arc))
                arc += n
        self.length = arc

    def ok(self):
        return len(self.segs) > 0

    def project(self, x, y):
        best = None
        for ax, ay, dx, dy, n, arc0 in self.segs:
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (n * n)))
            px, py = ax + t * dx, ay + t * dy
            d = math.hypot(x - px, y - py)
            if best is None or d < best[0]:
                cross = (dx * (y - ay) - dy * (x - ax)) / n      # +면 진행방향 왼쪽
                best = (d, math.copysign(d, cross), arc0 + t * n, (-dy / n, dx / n))
        return best  # (거리, 부호 횡거리, 호길이, 좌측 법선)


class BlockedRouteDetour(Node):
    def __init__(self):
        super().__init__('blocked_route_detour')
        dp = self.declare_parameter
        dp('map_osm', '')
        # 9/7 시뮬 실주행 반영: VTD GT objects 는 ~80 m 까지만 온다(2 회 주행 bag 실측).
        # 감지 거리를 늘려도 소용없으므로 확인 시간을 줄이고 감속 개시를 앞당긴다.
        # 9/7 정책 교체: 우회 가능 거리를 남기고 서서(standoff) 관찰한 뒤 판단한다.
        # standoff 근거(실측, 정지 상태 후보 finish): detour1 40.4 m / detour2 29.8 m → 큰 쪽 + 여유 6 m.
        dp('standoff_m', 48.0); dp('standoff_decel_mps2', 1.5); dp('approach_speed_mps', 4.0)
        dp('standoff_margin_m', 18.0)     # 오버슈트 보정(실측 11~16.6 m). 유효 목표 = standoff + margin
        dp('signal_debounce_s', 0.5)      # 신호 색 전이 히스테리시스(양방향 동일)
        dp('no_response_s', 3.0)          # 서서 관찰: 이만큼 무응답이면 우회 결정
        dp('blocker_move_speed_mps', 0.5)  # 이 이상이면 blocker 출발 → 추종 복귀
        dp('lc_finish_margin_m', 6.0)
        dp('blocker_stop_speed_mps', 0.3); dp('blocked_time_s', 0.5); dp('detection_distance_m', 80.0)
        dp('slowdown_time_s', 0.2)
        dp('hold_timeout_s', 20.0)
        dp('signal_hold_s', 2.0)
        dp('path_lateral_margin_m', 2.2); dp('regulatory_zone_m', 26.6); dp('stopline_min_m', 28.0)
        dp('abort_backoff_s', 10.0); dp('max_abort_count', 2)
        dp('pedestrian_lookahead_m', 50.0); dp('pedestrian_lateral_margin_m', 2.5); dp('pedestrian_slow_kph', 30.0)
        dp('pedestrian_release_s', 1.0); dp('respawn_hold_s', 1.5); dp('input_timeout_s', 1.0); dp('clear_time_s', 1.0)
        dp('tick_s', 0.2)
        g = lambda k: self.get_parameter(k).value
        self.params = Params(
            standoff_m=float(g('standoff_m')), standoff_decel_mps2=float(g('standoff_decel_mps2')),
            standoff_margin_m=float(g('standoff_margin_m')), signal_debounce_s=float(g('signal_debounce_s')),
            approach_speed_mps=float(g('approach_speed_mps')), no_response_s=float(g('no_response_s')),
            blocker_move_speed_mps=float(g('blocker_move_speed_mps')),
            lc_finish_margin_m=float(g('lc_finish_margin_m')),
            blocker_stop_speed_mps=float(g('blocker_stop_speed_mps')),
            blocker_min_stopped_s=float(g('blocked_time_s')),
            slowdown_min_stopped_s=float(g('slowdown_time_s')),
            signal_hold_s=float(g('signal_hold_s')),
            detection_distance_m=float(g('detection_distance_m')),
            path_lateral_margin_m=float(g('path_lateral_margin_m')),
            regulatory_clearance_m=float(g('regulatory_zone_m')), stopline_min_distance_m=float(g('stopline_min_m')),
            abort_backoff_s=float(g('abort_backoff_s')), max_abort_count=int(g('max_abort_count')),
            ped_lookahead_m=float(g('pedestrian_lookahead_m')),
            ped_lateral_margin_m=float(g('pedestrian_lateral_margin_m')),
            ped_speed_limit_mps=float(g('pedestrian_slow_kph')) / 3.6,
            ped_release_delay_s=float(g('pedestrian_release_s')),
            respawn_hold_s=float(g('respawn_hold_s')), input_timeout_s=float(g('input_timeout_s')),
            clear_time_s=float(g('clear_time_s')))
        self.logic = DetourLogic(self.params)
        # 파라미터 이름이 어긋나면 노드가 기동 중 죽어 우회 판단이 통째로 사라진다(9/7 detour4 에서 발생).
        # 런치가 넘기는 인자와 Params 필드가 어긋나지 않는지 기동 시 한 번 확인한다.
        import dataclasses as _dc
        _known = {f.name for f in _dc.fields(Params)}
        _passed = set(_dc.asdict(self.params))
        if _passed - _known:
            self.get_logger().error(f'Params 에 없는 인자: {sorted(_passed - _known)}')
        self.timeout = self.params.input_timeout_s

        # 맵 (이웃 차선·실선·정지선·회전 lanelet). 로드 실패 시 구조 조건이 항상 불성립 → STOP 만 (안전 쪽)
        self.map = None
        self.tlr = None
        try:
            self.map = OsmMap(str(g('map_osm')), self.get_logger())
            self.tlr = TrafficLightRouter(self.map, self.get_logger())
        except Exception as e:  # noqa
            self.get_logger().error(f'맵 로드 실패 → 우회 판단 비활성(항상 STOP): {e}')

        # 입력 상태
        self.odom = self.objects = self.path = self.route = None
        self.rx = {}                   # 이름 → 수신 시각(monotonic)
        self.status = {LEFT: None, RIGHT: None}
        self.tl_factors = None
        self.tl_signal_groups = 0
        self.tl_color = None
        self.stopped_since = {}        # 객체 id → 정지 시작 시각
        self.respawn_pending = False
        self.route_pref = []           # preferred lanelet 열
        self.route_segs = []           # [[preferred, 이웃...]]
        self.pending = None

        best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, '/localization/kinematic_state', lambda m: self._rx('odom', m), 10)
        self.create_subscription(PredictedObjects, '/perception/object_recognition/objects', lambda m: self._rx('objects', m), 10)
        self.create_subscription(PathWithLaneId, '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
                                 lambda m: self._rx('path', m), 10)
        self.create_subscription(LaneletRoute, '/planning/mission_planning/route', self.on_route, latched)
        for side, base in _MODULE_TOPIC.items():
            self.create_subscription(CooperateStatusArray, f'/planning/cooperate_status/{base}',
                                     lambda m, s=side: self.on_status(s, m), 10)
        self.create_subscription(PlanningFactorArray, '/planning/planning_factors/traffic_light', self.on_tl_factor, 10)
        self.create_subscription(TrafficLightGroupArray, '/perception/traffic_light_recognition/traffic_signals', self.on_tl_signal, 10)
        self.create_subscription(Empty, '/vtd/respawn', self.on_respawn, 10)
        self.rtc = {side: self.create_client(CooperateCommands, f'/planning/cooperate_commands/{base}')
                    for side, base in _MODULE_TOPIC.items()}
        self.status_pub = self.create_publisher(String, '/detour/status', 10)   # 매 tick JSON, bag 사후분석용
        self.red_last_t = -math.inf     # 적신호 factor 를 마지막으로 본 시각 (래퍼 단 latch)
        self.limit_pub = self.create_publisher(VelocityLimit, '/planning/scenario_planning/max_velocity_candidates', latched)
        self.clear_pub = self.create_publisher(VelocityLimitClearCommand, '/planning/scenario_planning/clear_velocity_limit', latched)
        self.last_state = None
        self.create_timer(float(g('tick_s')), self.tick)
        self.get_logger().info(
            f'blocked_route_detour: 실정지 목표 {self.params.standoff_m}m / 프로파일 겨냥 {self.params.standoff_m + self.params.standoff_margin_m}m, '
            f'무응답 {self.params.no_response_s}s 면 우회, 접근 상한 {self.params.approach_speed_mps}m/s, '
            f'감지 {self.params.detection_distance_m}m(VTD GT 실측 상한 ~80m), '
            f'게이트 감속 {self.params.slowdown_min_stopped_s}s / 승인 {self.params.blocker_min_stopped_s}s, '
            f'신호 latch {self.params.signal_hold_s}s')

    # ------------------------------------------------------------ 수신
    def _rx(self, name, msg):
        setattr(self, name, msg)
        self.rx[name] = time.monotonic()

    def fresh(self, name, now):
        return now - self.rx.get(name, -math.inf) <= self.timeout

    def input_ages(self, now):
        """각 입력의 마지막 수신 경과시간[s]. 미수신은 None. (9/7: 노드가 42 s 동안 FOLLOW 였던 원인 추적용)"""
        out = {}
        for name in ('odom', 'objects', 'path', 'tl_factor', 'tl_signal', 'status_left', 'status_right'):
            ts = self.rx.get(name)
            out[name] = None if ts is None else round(now - ts, 2)
        out['route'] = self.route is not None
        return out

    def on_route(self, m):
        self.route = m
        self.route_pref = [int(s.preferred_primitive.id) for s in m.segments]
        self.route_segs = [[int(s.preferred_primitive.id)] + [int(p.id) for p in s.primitives if int(p.id) != int(s.preferred_primitive.id)]
                           for s in m.segments]
        if self.tlr is not None:
            try:
                self.tlr.set_route(self.route_pref)
            except Exception as e:  # noqa
                self.get_logger().warning(f'tl_router set_route 실패: {e}')

    def on_status(self, side, m):
        self.status[side] = list(m.statuses)
        self.rx['status_' + side] = time.monotonic()

    def on_tl_factor(self, m):
        self.tl_factors = m
        self.rx['tl_factor'] = time.monotonic()

    def on_tl_signal(self, m):
        """경로상 다음 정지선의 신호 색을 뽑는다 (브리지는 그 정지선의 그룹만 발행한다).

        GREEN 이 하나라도 있으면 비적색으로 본다 — 좌회전 화살표(GREEN LEFT_ARROW)를 포함하기 위해서다.
        (9/7 detour9: factor 부재를 직전값 유지로 해석해 적신호가 영구 고착됐다. 색이 1차 근거다.)"""
        self.tl_signal_groups = len(m.traffic_light_groups)
        color = None
        for g in m.traffic_light_groups:
            for e in g.elements:
                if e.color == TrafficLightElement.GREEN:
                    color = 'GREEN'
                    break
                if e.color == TrafficLightElement.RED:
                    color = 'RED'
                elif e.color == TrafficLightElement.AMBER and color != 'RED':
                    color = 'AMBER'
            if color == 'GREEN':
                break
        self.tl_color = color
        self.rx['tl_signal'] = time.monotonic()

    def on_respawn(self, _):
        self.respawn_pending = True
        self.stopped_since.clear()

    # ------------------------------------------------------------ Inputs 구성
    def _ego_lane(self, x, y, yaw):
        """(현재 lanelet id, 경로 세그먼트 idx, lane_role, 반폭). 맵/경로 없으면 None."""
        if self.map is None:
            return None
        lid = self.map.nearest_lanelet(x, y, heading=yaw, avoid_dead_end=False)
        if lid is None:
            return None
        role, seg_idx = 'preferred', None
        for k, seg in enumerate(self.route_segs):
            if lid in seg:
                seg_idx = k
                pref = seg[0]
                if lid != pref:
                    ln, rn = self.map.neighbors(pref)     # preferred 의 좌/우 이웃
                    role = 'left_of_preferred' if lid == ln else 'right_of_preferred'
                break
        half = max(1.0, self.map.width_at(lid, x, y) / 2.0)
        return lid, seg_idx, role, half

    def _lane_info(self, lid, seg_idx, x, y, s_route_ego):
        """이웃 차선·실선·다음 규제요소(회전 lanelet·신호등)·정지선 거리."""
        ln, rn = self.map.neighbors(lid)
        info = LaneInfo(neighbor_left=ln is not None, neighbor_right=rn is not None,
                        left_boundary_solid=self.map.boundary_solid(lid, 'left'),
                        right_boundary_solid=self.map.boundary_solid(lid, 'right'))
        # 정지선: tl_router 가 경로 누적거리로 계산 (경로상 다음 정지선)
        if self.tlr is not None and self.tlr.has_route() and s_route_ego is not None:
            entry = self.tlr.next_stop(s_route_ego)
            if entry is not None:
                info.distance_to_stopline_m = max(0.0, entry.s_route - s_route_ego)
        # 회전 lanelet: preferred 열을 따라 다음 turn_direction(left/right) lanelet 시작까지
        if seg_idx is not None and lid in self.map.lanelets:
            _, s_in, _ = self.map.project(lid, x, y)
            dist = -s_in
            for k in range(seg_idx, len(self.route_pref)):
                pl = self.route_pref[k]
                ll = self.map.lanelets.get(pl)
                if ll is None:
                    break
                if k > seg_idx and ll.tags.get('turn_direction') in ('left', 'right'):
                    info.distance_to_regulatory_m = max(0.0, dist)
                    break
                if k == seg_idx and ll.tags.get('turn_direction') in ('left', 'right'):
                    info.distance_to_regulatory_m = 0.0
                    break
                dist += ll.length
        # 규제요소 = 회전 lanelet 과 신호등 정지선 중 가까운 것 (lane_change is_near_regulatory_element 와 동일 취지)
        cands = [d for d in (info.distance_to_regulatory_m, info.distance_to_stopline_m) if d is not None]
        info.distance_to_regulatory_m = min(cands) if cands else None
        return info

    def _objects(self, frame, ego_arc, now_t):
        out = []
        for o in self.objects.objects:
            oid = int.from_bytes(bytes(o.object_id.uuid[:4]), 'little')
            cls = _CLASS_NAME.get(o.classification[0].label if o.classification else -1, 'UNKNOWN')
            pose = o.kinematics.initial_pose_with_covariance.pose
            speed = float(o.kinematics.initial_twist_with_covariance.twist.linear.x)
            pr = frame.project(pose.position.x, pose.position.y)
            if pr is None:
                continue
            _, lat, arc, (nx, ny) = pr
            h = qyaw(pose.orientation)
            vel_n = speed * (math.cos(h) * nx + math.sin(h) * ny)     # 좌측 법선 방향 속도
            toward = -vel_n if lat > 0 else vel_n                      # 경로 쪽으로 다가오면 +
            if abs(speed) <= self.params.blocker_stop_speed_mps:
                self.stopped_since.setdefault(oid, now_t)
            else:
                self.stopped_since.pop(oid, None)
            out.append(ObjectInfo(id=oid, cls=cls, v=abs(speed), longitudinal_m=arc - ego_arc, lateral_m=lat,
                                  stopped_since=self.stopped_since.get(oid), lateral_speed_toward_path_mps=toward))
        return out

    def _candidate(self, side, now):
        sts = self.status.get(side)
        if not sts:
            return CandidateInfo(present=False, stale=not self.fresh('status_' + side, now))
        st = next((s for s in sts if s.module.type == _EXT_MODULE[side]), sts[0])
        return CandidateInfo(present=True, safe=bool(st.safe), state=_STATE_NAME.get(st.state.type),
                             start_distance_m=float(st.start_distance), finish_distance_m=float(st.finish_distance),
                             stale=not self.fresh('status_' + side, now))

    def _red_factor(self, now):
        """보조 근거: traffic_light 모듈 STOP factor 의 control point 거리. 없으면 None.

        **부재를 '직전값 유지' 로 해석하지 않는다.** 초록이 되면 factor 가 사라지므로 그렇게 하면
        영원히 적신호가 된다(9/7 detour9). 히스테리시스는 detour_logic 이 색 기준으로 양방향 처리한다.
        """
        if self.tl_factors is None or not self.fresh('tl_factor', now):
            return None
        ds = [cp.distance for f in self.tl_factors.factors
              if f.behavior == PlanningFactor.STOP for cp in f.control_points]
        return min(ds) if ds else None

    # ------------------------------------------------------------ tick
    def tick(self):
        now = time.monotonic()
        t = now
        respawn, self.respawn_pending = self.respawn_pending, False
        stale = not all(self.fresh(n, now) for n in ('odom', 'objects', 'path'))
        ego_v = float(self.odom.twist.twist.linear.x) if self.odom else 0.0
        objects, lane = [], LaneInfo(False, False)
        role, half = 'preferred', 1.75
        if not stale and self.path is not None and len(self.path.points) >= 2:
            frame = PathFrame([pp.point.pose.position for pp in self.path.points])
            ex, ey = self.odom.pose.pose.position.x, self.odom.pose.pose.position.y
            yaw = qyaw(self.odom.pose.pose.orientation)
            pr = frame.project(ex, ey) if frame.ok() else None
            if pr is not None:
                ego_arc = pr[2]
                objects = self._objects(frame, ego_arc, t)
                el = self._ego_lane(ex, ey, yaw)
                if el is not None:
                    lid, seg_idx, role, half = el
                    s_route = None
                    if self.tlr is not None and self.tlr.has_route():
                        loc = self.tlr.locate(ex, ey)
                        s_route = loc[1] if loc is not None else None
                    lane = self._lane_info(lid, seg_idx, ex, ey, s_route)
        sig_fresh = self.fresh('tl_signal', now)
        sig_age = None if 'tl_signal' not in self.rx else round(now - self.rx['tl_signal'], 2)
        signal = SignalInfo(red_stop_factor_distance_m=self._red_factor(now),
                            available=sig_fresh and self.tl_signal_groups > 0,
                            color=(self.tl_color if sig_fresh else None), color_age_s=sig_age)
        inputs = Inputs(t=t, ego=EgoState(t=t, v=ego_v, lane_role=role, lane_half_width_m=half), objects=objects,
                        lane=lane, signal=signal, candidates={LEFT: self._candidate(LEFT, now), RIGHT: self._candidate(RIGHT, now)},
                        respawn=respawn, stale=stale)
        d = self.logic.step(inputs)
        self._apply(d, now)

    # ------------------------------------------------------------ Decision → 채널
    def _apply(self, d, now):
        stamp = self.get_clock().now().to_msg()
        for sender, v in d.limits:
            m = VelocityLimit(); m.stamp = stamp; m.sender = sender; m.max_velocity = float(v)
            self.limit_pub.publish(m)
        for sender in d.clears:
            m = VelocityLimitClearCommand(); m.stamp = stamp; m.sender = sender; m.command = True
            self.clear_pub.publish(m)
        if d.approve is not None:
            self._approve(d.approve, stamp)
        payload = {'t': round(now, 2), 'state': d.state, 'reason': d.reason, 'approve': d.approve,
                   'limits': [[k, round(v, 2)] for k, v in d.limits], 'clears': list(d.clears),
                   'ages': self.input_ages(now)}
        payload.update(d.detail)
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        if now - getattr(self, 'last_beat', -1e9) >= 10.0:
            self.last_beat = now
            self.get_logger().info(f'DETOUR heartbeat {d.state} {d.reason} | '
                                   + json.dumps(d.detail, ensure_ascii=False)
                                   + ' ages=' + json.dumps(self.input_ages(now)))
        if d.state != self.last_state or d.approve:
            self.get_logger().info(f'DETOUR {d.state} {d.reason} | ' + json.dumps(d.detail, ensure_ascii=False)
                                   + f' limits={d.limits} clears={d.clears} approve={d.approve}'
                                   + ' ages=' + json.dumps(self.input_ages(now)))
            self.last_state = d.state

    def _approve(self, side, stamp):
        sts = self.status.get(side) or []
        st = next((s for s in sts if s.module.type == _EXT_MODULE[side]), sts[0] if sts else None)
        cli = self.rtc.get(side)
        if st is None or cli is None or not cli.service_is_ready():
            self.get_logger().warning(f'DETOUR {side} 승인 불가: status/service 없음')
            return
        req = CooperateCommands.Request()
        req.stamp = stamp
        req.commands = [CooperateCommand(uuid=st.uuid, module=st.module, command=Command(type=Command.ACTIVATE))]
        self.pending = cli.call_async(req)
        self.pending.add_done_callback(self._on_rtc_response)

    def _on_rtc_response(self, fut):
        try:
            r = fut.result()
            if not r.responses or not all(x.success for x in r.responses):
                self.get_logger().warning('DETOUR RTC 승인 거부됨')
        except Exception as e:  # noqa
            self.get_logger().error(f'DETOUR RTC 호출 실패: {e}')


def main():
    rclpy.init()
    n = BlockedRouteDetour()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()

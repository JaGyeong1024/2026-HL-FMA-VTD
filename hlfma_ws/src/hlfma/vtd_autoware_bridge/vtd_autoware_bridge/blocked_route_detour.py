"""Route-aware, staged RTC approval of Autoware lane-change candidates.

Autoware owns candidate generation, RSS and control. This node adds signal/map/
footprint vetoes, an approach velocity limit and sequential detour/return states.
"""
import json
import math
import time
from xml.etree.ElementTree import ParseError

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from autoware_planning_msgs.msg import Path, LaneletRoute
from autoware_perception_msgs.msg import PredictedObjects, ObjectClassification, TrafficLightGroupArray, TrafficLightElement as TL
from autoware_internal_planning_msgs.msg import PathWithLaneId, VelocityLimit, VelocityLimitClearCommand, PlanningFactor, PlanningFactorArray
from tier4_rtc_msgs.msg import CooperateStatusArray, CooperateCommand, Command, State
from tier4_rtc_msgs.srv import CooperateCommands
from std_msgs.msg import String

from .detour_geometry import ArcPath, clearance, clearance_with_collision, object_id, yaw as qyaw
from .detour_map import DetourMap
from .osm_map import OsmMap


DEFAULTS = {
    'object_stop_speed_mps': 0.3, 'blocked_time_s': 0.1,
    'detection_distance_m': 120.0, 'input_timeout_s': 1.0,
    'hold_distance_m': 45.0, 'approach_speed_mps': 4.0,
    'approach_deceleration_mps2': 1.5, 'clear_time_s': 5.0,
    'lookahead_m': 150.0, 'retry_interval_s': 0.2,
    'signal_queue_check_distance_m': 80.0, 'signal_timeout_s': 0.5,
    'factor_timeout_s': 1.0, 'reaction_time_s': 1.0,
    'completion_margin_m': 3.0, 'stop_margin_m': 2.0,
    'completion_deceleration_mps2': 1.0, 'completion_jerk_mps3': 1.0,
    'completion_initial_acceleration_mps2': 1.0,
    'return_reserve_m': 30.0, 'footprint_margin_m': 0.2,
    'current_lane_half_width_m': 1.5,
    'geometry_step_m': 0.5, 'prediction_horizon_s': 4.0,
    'vehicle_front_m': 3.808, 'vehicle_rear_m': 1.04, 'vehicle_width_m': 1.886,
    'emergency_trigger_distance_m': 15.0, 'emergency_creep_speed_mps': 1.0,
    'emergency_reaction_time_s': 0.3, 'emergency_deceleration_mps2': 3.0,
    'emergency_margin_m': 0.5,
    # HL FMA 9/9: 접근 제한이 0 까지 내려가면 궤적 속도가 전부 0 이 되어,
    # 오토웨어가 stuck 차선변경을 결정해도 실행할 수 없다(실측: t=46.17 에 0.0).
    # 블로커 앞 정지는 obstacle_stop 이 5m 마진으로 담당한다. 되돌리려면 0.0
    'min_approach_speed_mps': 1.0,
    'switch_hysteresis_m': 5.0, 'execution_ack_timeout_s': 5.0,
}


class BlockedRouteDetour(Node):
    def __init__(self):
        super().__init__('blocked_route_detour')
        for name, value in DEFAULTS.items():
            value = float(self.declare_parameter(name, value).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
            setattr(self, name, value)
        self.map_policy = None
        map_path = self.declare_parameter('map_osm', '').value
        try:
            self.map_policy = DetourMap(OsmMap(map_path))
        except (OSError, ValueError, KeyError, ParseError) as error:
            self.get_logger().error(f'MAP_UNAVAILABLE: {error}; RTC approval disabled')
        self.odom = self.objects = self.behavior_path = self.signals = None
        self.received, self.source_stamps, self.regulatory_factors = {}, {}, {}
        self.candidates = {k: None for k in ('left', 'right', 'route_left', 'route_right')}
        self.status, self.diagnostics = {}, {}
        self.limit_active = False
        self.last_blocked = self.last_request = self.last_diagnostic = -math.inf
        self.last_diagnostic_message = None
        self.blocked_since = None
        self.pending = self.committed = self.executing = self.last_choice = None
        self.emergency_committed = self.emergency_executing = False
        self.detouring = False
        self.execution_uuid = self.committed_uuid = None
        self.terminal_events = {}
        self.state = 'CRUISE'
        self.last_blocker_info = None
        self.last_blocker_uuid = self.last_blocker_object = None
        self.signal_info = {}
        self.route_uuid = None
        self.create_subscription(Odometry, '/localization/kinematic_state', lambda m: self.receive('odom', m), 10)
        self.create_subscription(PredictedObjects, '/perception/object_recognition/objects', lambda m: self.receive('objects', m), 10)
        self.create_subscription(PathWithLaneId, '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id', lambda m: self.receive('behavior_path', m), 10)
        self.create_subscription(TrafficLightGroupArray, '/perception/traffic_light_recognition/traffic_signals', lambda m: self.receive('signals', m), 10)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LaneletRoute, '/planning/mission_planning/route', self.on_route, retained)
        modules = {'left': 'external_request_lane_change_left', 'right': 'external_request_lane_change_right',
                   'route_left': 'lane_change_left', 'route_right': 'lane_change_right'}
        for key, module in modules.items():
            self.create_subscription(Path, '/planning/path_candidate/'+module, lambda m, k=key: self.on_candidate(k, m), 10)
            self.create_subscription(CooperateStatusArray, '/planning/cooperate_status/'+module, lambda m, k=key: self.on_status(k, m), 10)
        self.rtc_clients = {k: self.create_client(CooperateCommands, '/planning/cooperate_commands/'+v) for k, v in modules.items()}
        self.limit_pub = self.create_publisher(VelocityLimit, '/planning/scenario_planning/max_velocity_candidates', retained)
        self.clear_pub = self.create_publisher(VelocityLimitClearCommand, '/planning/scenario_planning/clear_velocity_limit', retained)
        for name in ('traffic_light', 'intersection', 'crosswalk', 'stop_line', 'road_user_stop'):
            self.create_subscription(PlanningFactorArray, '/planning/planning_factors/'+name, lambda m, k=name: self.on_factor(k, m), 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.diagnostic_pub = self.create_publisher(String, '~/diagnostics', 10)
        self.create_timer(0.2, self.tick)

    def record_time(self, key, message):
        self.received[key] = time.monotonic()
        stamp = getattr(getattr(message, 'header', message), 'stamp', None)
        self.source_stamps[key] = (stamp.sec+stamp.nanosec*1e-9) if stamp else None

    def receive(self, name, message):
        setattr(self, name, message)
        self.record_time(name, message)

    def fresh(self, key, now, timeout=None):
        timeout = self.input_timeout_s if timeout is None else timeout
        if now-self.received.get(key, -math.inf) > timeout:
            return False
        stamp = self.source_stamps.get(key)
        if stamp is not None:
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            return -0.1 <= age <= timeout
        return True

    def on_candidate(self, key, message):
        self.candidates[key] = message
        self.record_time('candidate_'+key, message)

    def on_status(self, key, message):
        self.status[key] = list(message.statuses)
        self.record_time('status_'+key, message)
        if self.fresh('status_'+key, time.monotonic()):
            for st in message.statuses:
                identity = (key, bytes(st.uuid.uuid))
                if st.state.type in (State.SUCCEEDED, State.FAILED):
                    self.terminal_events[identity] = (st.state.type, time.monotonic())
        self.terminal_events = {k: v for k, v in self.terminal_events.items()
                                if time.monotonic()-v[1] < 30.0}

    def on_factor(self, key, message):
        self.regulatory_factors[key] = message
        self.record_time('factor_'+key, message)

    def on_route(self, message):
        ids = [s.preferred_primitive.id for s in message.segments]
        if self.map_policy:
            self.map_policy.set_route(ids)
        signature = (bytes(message.uuid.uuid), tuple(ids))
        if signature != self.route_uuid:
            self.route_uuid = signature
            self.last_choice = None
            # No old candidate may be approved against a replacement route.
            self.candidates = {key: None for key in self.candidates}
            self.status = {}
            self.detouring = False
            self.emergency_committed = self.emergency_executing = False
            self.last_blocker_uuid = self.last_blocker_object = None

    @staticmethod
    def label(obj):
        return max(obj.classification, key=lambda c: c.probability).label if obj.classification else ObjectClassification.UNKNOWN

    @classmethod
    def is_pedestrian(cls, obj):
        return cls.label(obj) == ObjectClassification.PEDESTRIAN

    @staticmethod
    def speed(obj):
        v = obj.kinematics.initial_twist_with_covariance.twist.linear
        return math.hypot(v.x, v.y)

    def ego_speed(self):
        v = self.odom.twist.twist.linear
        return math.hypot(v.x, v.y)

    def path_geometry(self, path):
        return ArcPath([p.point if hasattr(p, 'point') else p for p in path.points])

    def corridor(self, path, check_objects=True, objects=None, predicted=False):
        if path is None or self.odom is None or self.objects is None:
            return -1.0
        try:
            geometry = self.path_geometry(path)
            selected = self.objects.objects if objects is None else objects
            return clearance(geometry, self.odom.pose.pose.position, selected if check_objects else [],
                             self.vehicle_front_m, self.vehicle_rear_m, self.vehicle_width_m,
                             self.footprint_margin_m, self.lookahead_m, self.geometry_step_m,
                             self.prediction_horizon_s if predicted else 0.0)
        except (ValueError, OverflowError):
            return -1.0

    def corridor_details(self, path, objects=None, predicted=False):
        if path is None or self.odom is None or self.objects is None:
            return -1.0, None
        try:
            geometry = self.path_geometry(path)
            selected = self.objects.objects if objects is None else objects
            return clearance_with_collision(
                geometry, self.odom.pose.pose.position, selected, self.vehicle_front_m,
                self.vehicle_rear_m, self.vehicle_width_m, self.footprint_margin_m,
                self.lookahead_m, self.geometry_step_m,
                self.prediction_horizon_s if predicted else 0.0)
        except (ValueError, OverflowError):
            return -1.0, None

    def blocking(self, use_forward_fallback=True):
        if self.behavior_path is None or self.objects is None or self.odom is None:
            return None
        ego = self.odom.pose.pose
        heading = qyaw(ego.orientation)
        c, s = math.cos(heading), math.sin(heading)
        stopped = []
        for obj in self.objects.objects:
            if self.is_pedestrian(obj) or self.speed(obj) > self.object_stop_speed_mps:
                continue
            position = obj.kinematics.initial_pose_with_covariance.pose.position
            dx, dy = position.x-ego.position.x, position.y-ego.position.y
            longitudinal = c*dx+s*dy
            lateral = -s*dx+c*dy
            # Blocking is a current-lane decision. Adjacent-lane objects remain
            # available to candidate polygon/RSS checks, but must not stop ego's
            # unobstructed lane.
            if longitudinal >= 0.0 and abs(lateral) <= self.current_lane_half_width_m:
                stopped.append(obj)
        self.last_blocker_info = None
        self.last_blocker_uuid = self.last_blocker_object = None
        if not stopped:
            return None
        clear, blocker_uuid = self.corridor_details(self.behavior_path, objects=stopped)
        available = self.corridor(self.behavior_path, check_objects=False)
        if 0 <= clear < min(available, self.detection_distance_m):
            self.last_blocker_info = {'distance_m': clear, 'stationary_objects': len(stopped),
                                      'source': 'PATH_SWEEP',
                                      'object_id': blocker_uuid.hex() if blocker_uuid else None}
            self.last_blocker_uuid = blocker_uuid
            self.last_blocker_object = next(
                (obj for obj in stopped if object_id(obj) == blocker_uuid), None)
            return clear
        # A stop point can shorten the published behavior path before the object.
        # In that case clear == available and releasing the velocity limit makes
        # the path grow again, causing stop/creep cycles toward the blocker.
        # Detect same-lane stopped objects directly in the ego frame as fallback.
        if not use_forward_fallback:
            return None
        direct = []
        for obj in stopped:
            position = obj.kinematics.initial_pose_with_covariance.pose.position
            dx, dy = position.x-ego.position.x, position.y-ego.position.y
            longitudinal = c*dx+s*dy
            lateral = -s*dx+c*dy
            radius = 0.5*math.hypot(obj.shape.dimensions.x, obj.shape.dimensions.y)
            if (0.0 <= longitudinal <= self.detection_distance_m+self.vehicle_front_m+radius
                    and abs(lateral) <= self.current_lane_half_width_m):
                direct.append((max(0.0, longitudinal-self.vehicle_front_m-radius), obj))
        if direct:
            clear, blocker_object = min(direct, key=lambda item: item[0])
            blocker_uuid = object_id(blocker_object)
            self.last_blocker_info = {'distance_m': clear, 'stationary_objects': len(stopped),
                                      'source': 'EGO_FORWARD_FALLBACK',
                                      'object_id': blocker_uuid.hex() if blocker_uuid else None}
            self.last_blocker_uuid = blocker_uuid
            self.last_blocker_object = blocker_object
            return clear
        return None

    @staticmethod
    def signal_color(groups, turn):
        arrows = {'left': (TL.LEFT_ARROW, TL.UP_LEFT_ARROW), 'right': (TL.RIGHT_ARROW, TL.UP_RIGHT_ARROW),
                  'straight': (TL.UP_ARROW, TL.UP_LEFT_ARROW, TL.UP_RIGHT_ARROW)}.get(turn, ())
        decisions = []
        for group in groups:
            elements = [e for e in group.elements if e.status == TL.SOLID_ON and e.confidence > 0]
            applicable = [e for e in elements if e.shape == TL.CIRCLE or e.shape in arrows]
            if any(e.shape in arrows and e.color == TL.GREEN for e in applicable):
                decisions.append('GREEN')
            elif any(e.color == TL.RED for e in applicable):
                decisions.append('RED')
            elif any(e.color == TL.AMBER for e in applicable):
                decisions.append('AMBER')
            elif any(e.color == TL.GREEN for e in applicable):
                decisions.append('GREEN')
            else:
                decisions.append('UNKNOWN')
        return next((c for c in ('RED', 'AMBER', 'UNKNOWN') if c in decisions), 'GREEN') if decisions else 'UNKNOWN'

    def signal_gate(self, now, horizon=None):
        self.signal_info = {}
        if self.map_policy is None or not self.map_policy.route:
            return 'MAP_UNAVAILABLE'
        context = self.map_policy.next_signal(self.odom.pose.pose.position)
        if context is None:
            return None
        if not -1.0 <= context[0] <= max(self.signal_queue_check_distance_m, horizon or 0.):
            return None
        return self.check_signal(context, now)

    def check_signal(self, context, now):
        distance, ids, turn, lanelet = context
        self.signal_info = {'distance_m': distance, 'groups': ids, 'turn': turn, 'lanelet': lanelet}
        if self.signals is None or not self.fresh('signals', now, self.signal_timeout_s):
            self.signal_info['color'] = 'STALE'
            return 'STALE_INPUT'
        groups = [g for g in self.signals.traffic_light_groups if g.traffic_light_group_id in ids]
        color = self.signal_color(groups, turn) if len(groups) == len(set(ids)) else 'UNKNOWN'
        self.signal_info['color'] = color
        return 'SIGNAL_WAIT' if color == 'RED' else ('REGULATION' if color != 'GREEN' else None)

    def planner_stop_before(self, distance):
        now = time.monotonic()
        for key, message in self.regulatory_factors.items():
            if not self.fresh('factor_'+key, now, self.factor_timeout_s):
                continue
            for factor in message.factors:
                if factor.behavior != PlanningFactor.STOP:
                    continue
                try:
                    path = self.path_geometry(self.behavior_path)
                    ego = self.odom.pose.pose.position
                    _, origin = path.project(ego.x, ego.y)
                    for point in factor.control_points:
                        lateral, arc = path.project(point.pose.position.x, point.pose.position.y)
                        if lateral <= self.vehicle_width_m/2+self.footprint_margin_m and -1.0 <= arc-origin <= distance:
                            return True
                except ValueError:
                    return True
        return False

    def set_state(self, state):
        if self.state != state:
            self.get_logger().info(f'DETOUR {self.state} -> {state}')
            self.state = state
        self.state_pub.publish(String(data=state))

    def set_approach_limit(self, blocker):
        # Clearance is to first footprint collision, not to the object's center.
        available = max(0., blocker-self.hold_distance_m)
        a, t = self.approach_deceleration_mps2, self.reaction_time_s
        speed = min(self.approach_speed_mps, max(0., math.sqrt((a*t)**2+2*a*available)-a*t))
        # blocker<=0 은 '지금 세워라' 라는 명시적 호출이므로 그대로 둔다.
        if blocker > 0.:
            speed = max(speed, self.min_approach_speed_mps)
        self.limit_pub.publish(VelocityLimit(stamp=self.get_clock().now().to_msg(), sender='blocked_route_detour', max_velocity=speed))
        self.limit_active = True

    def clear_approach_limit(self):
        if self.limit_active:
            self.clear_pub.publish(VelocityLimitClearCommand(stamp=self.get_clock().now().to_msg(), sender='blocked_route_detour', command=True))
            self.limit_active = False

    def set_emergency_limit(self):
        self.limit_pub.publish(VelocityLimit(
            stamp=self.get_clock().now().to_msg(), sender='blocked_route_detour',
            max_velocity=self.emergency_creep_speed_mps))
        self.limit_active = True

    def emergency_stopping_distance(self, speed):
        return speed*self.emergency_reaction_time_s + speed*speed/(2*self.emergency_deceleration_mps2)

    def stopping_distance(self, speed):
        # Conservative initial acceleration equals the configured planning maximum.
        # Integrate reaction, jerk ramp to nominal deceleration, then braking.
        a = self.completion_initial_acceleration_mps2
        d = self.completion_deceleration_mps2
        j = self.completion_jerk_mps3
        t = self.reaction_time_s
        distance = speed*t+0.5*a*t*t
        velocity = speed+a*t
        ramp = min((a+d)/j, (a+math.sqrt(a*a+2*j*velocity))/j)
        distance += velocity*ramp+0.5*a*ramp*ramp-j*ramp**3/6
        velocity += a*ramp-0.5*j*ramp*ramp
        return distance+max(0., velocity)**2/(2*d)

    def safe_choices(self, keys, blocker=None):
        now = time.monotonic()
        choices = []
        for key in keys:
            d = self.diagnostics[key] = {'reason': 'NO_CANDIDATE'}
            candidate = self.candidates[key]
            if candidate is None or len(candidate.points) < 2:
                continue
            if not all(self.fresh(prefix+key, now) for prefix in ('candidate_', 'status_')):
                d['reason'] = 'STALE_INPUT'
                continue
            if (candidate.header.frame_id != 'map' or not all(
                    math.isfinite(p.longitudinal_velocity_mps) and p.longitudinal_velocity_mps >= 0
                    for p in candidate.points)):
                d['reason'] = 'INVALID_INPUT'
                continue
            clear, collision_uuid = self.corridor_details(candidate, predicted=True)
            available = self.corridor(candidate, check_objects=False)
            d.update(clearance_m=clear, available_m=available,
                     collision_object_id=collision_uuid.hex() if collision_uuid else None)
            for st in self.status.get(key, []):
                if not st.safe:
                    d['reason'] = 'UNSAFE_RSS'
                    continue
                if st.auto_mode or st.state.type != State.WAITING_FOR_EXECUTION:
                    d['reason'] = 'NOT_WAITING'
                    continue
                if not (math.isfinite(st.start_distance) and math.isfinite(st.finish_distance)
                        and 0 <= st.start_distance < st.finish_distance):
                    d['reason'] = 'INVALID_DISTANCE'
                    continue
                # Candidate velocities can be zero while waiting for approval. Never
                # use that zero to erase completion stopping space.
                geometry = self.path_geometry(candidate)
                ego = self.odom.pose.pose.position
                _, origin = geometry.project(ego.x, ego.y)
                phase_speeds = []
                arc = 0.0
                previous = candidate.points[0].pose.position
                for point in candidate.points:
                    position = point.pose.position
                    arc += math.hypot(position.x-previous.x, position.y-previous.y)
                    if -1.0 <= arc-origin <= st.finish_distance+self.completion_margin_m:
                        phase_speeds.append(abs(point.longitudinal_velocity_mps))
                    previous = position
                # Candidate points retain the road speed while this node limits a
                # blocked approach. Cap that nominal speed so it cannot inflate the
                # post-change stopping reserve and reject every usable candidate.
                phase_speed = max(phase_speeds, default=0.0)
                if blocker is not None or self.limit_active:
                    phase_speed = min(phase_speed, self.approach_speed_mps)
                v = max(self.ego_speed(), self.approach_speed_mps, phase_speed)
                stopping = self.stopping_distance(v)
                required = st.finish_distance+self.completion_margin_m+self.stop_margin_m+stopping
                if not math.isfinite(required):
                    d['reason'] = 'INVALID_DISTANCE'
                    continue
                d.update(start_m=st.start_distance, finish_m=st.finish_distance,
                         required_m=required, stopping_m=stopping)
                mode = 'NORMAL'
                if clear < required:
                    emergency = (blocker is not None and self.last_blocker_object is not None and
                                 blocker <= self.emergency_trigger_distance_m)
                    other_objects = [obj for obj in self.objects.objects
                                     if obj is not self.last_blocker_object]
                    emergency_clear, emergency_collision_uuid = self.corridor_details(
                        candidate, objects=other_objects, predicted=True)
                    emergency_speed = max(self.ego_speed(), self.emergency_creep_speed_mps)
                    emergency_stopping = self.emergency_stopping_distance(emergency_speed)
                    emergency_required = st.finish_distance+self.emergency_margin_m+emergency_stopping
                    d.update(emergency_clearance_m=emergency_clear,
                             emergency_required_m=emergency_required,
                             emergency_stopping_m=emergency_stopping,
                             emergency_collision_object_id=(
                                 emergency_collision_uuid.hex() if emergency_collision_uuid else None))
                    if not emergency or emergency_clear < emergency_required:
                        d['reason'] = 'COLLISION' if clear < available else 'INSUFFICIENT_HORIZON'
                        continue
                    mode = 'EMERGENCY'
                    required = emergency_required
                # Check regulatory stops for route candidates too, including beyond
                # the current blocker when the lane change would enter that interval.
                signal_reason = self.signal_gate(now, required)
                if signal_reason:
                    d['reason'] = signal_reason
                    continue
                if self.planner_stop_before(required):
                    d['reason'] = 'REGULATION'
                    continue
                if self.map_policy is None:
                    d['reason'] = 'MAP_UNAVAILABLE'
                    continue
                reserve = max(self.return_reserve_m, stopping+self.completion_margin_m)
                reason = self.map_policy.validate(self.path_geometry(candidate), self.odom.pose.pose.position,
                                                 st.finish_distance, required, self.vehicle_front_m,
                                                 self.vehicle_rear_m, self.vehicle_width_m, reserve)
                if reason:
                    d['reason'] = reason
                    continue
                for context in self.map_policy.candidate_signals(
                        self.path_geometry(candidate), self.odom.pose.pose.position, required):
                    reason = self.check_signal(context, now)
                    if reason:
                        break
                if reason:
                    d['reason'] = reason
                    continue
                d['reason'] = 'READY_EMERGENCY' if mode == 'EMERGENCY' else 'READY'
                d['mode'] = mode
                choices.append((emergency_clear if mode == 'EMERGENCY' else clear, key, st, mode))
        return choices

    def report_status(self, reason=None):
        data = {'state': self.state, 'reason': reason, 'blocker': self.last_blocker_info,
                'signal': self.signal_info, 'selected': self.committed, 'candidates': self.diagnostics}
        message = json.dumps(data, allow_nan=False)
        self.diagnostic_pub.publish(String(data=message))
        now = time.monotonic()
        # HL FMA 9/9: 5초 주기면 left/right 후보의 거절 사유가 로그에 거의 안 남는다
        # (실측: route_* 만 보였다). 내용이 바뀔 때마다 남기되 0.5s 간격은 유지.
        if message != self.last_diagnostic_message and now-self.last_diagnostic >= 0.5:
            self.get_logger().info('DETOUR '+message)
            self.last_diagnostic = now
            self.last_diagnostic_message = message

    def tick(self):
        now = time.monotonic()
        self.diagnostics = {}
        if not all(self.fresh(n, now) for n in ('odom', 'objects', 'behavior_path')):
            self.blocked_since = None
            if self.limit_active or self.detouring or self.committed:
                self.set_approach_limit(0.)
            self.set_state('WAIT_SAFE_GAP')
            self.report_status('STALE_INPUT')
            return
        pose, velocity = self.odom.pose.pose, self.odom.twist.twist.linear
        values = (pose.position.x, pose.position.y, qyaw(pose.orientation), velocity.x, velocity.y)
        if (not all(math.isfinite(v) for v in values) or
                any(getattr(self, key).header.frame_id != 'map' for key in ('odom','objects','behavior_path'))):
            self.set_approach_limit(0.)
            self.set_state('WAIT_SAFE_GAP')
            self.report_status('INVALID_INPUT')
            return
        running = [(key, st) for key, statuses in self.status.items() if self.fresh('status_'+key, now)
                   for st in statuses if st.state.type in (State.RUNNING, State.ABORTING)]
        expected = None
        if self.executing is not None:
            expected = (self.executing, self.execution_uuid)
        elif self.committed is not None:
            expected = (self.committed, self.committed_uuid)
        owned_running = next(
            ((key, st) for key, st in running
             if expected == (key, bytes(st.uuid.uuid))), None)
        if owned_running:
            self.executing = owned_running[0]
            self.execution_uuid = bytes(owned_running[1].uuid.uuid)
            self.detouring = self.detouring or not self.executing.startswith('route_')
            self.emergency_executing = self.emergency_committed
            self.emergency_committed = False
            self.committed = None
            # RUNNING is only an RTC state. Keep the fail-safe limit until the
            # actually selected behavior path no longer intersects the blocker.
            blocker = self.blocking(use_forward_fallback=False)
            if self.emergency_executing:
                self.set_emergency_limit()
            elif blocker is None:
                self.clear_approach_limit()
            else:
                self.last_blocked = now
                self.set_approach_limit(blocker)
            self.set_state('EXECUTE_RETURN' if self.executing.startswith('route_') else 'EXECUTE_DETOUR')
            self.report_status()
            return
        if running:
            # Never release the limiter or approve the opposite direction for
            # an unrelated/autonomous RTC execution that this node did not request.
            blocker = self.blocking()
            if blocker is not None:
                self.last_blocked = now
                self.blocked_since = now if self.blocked_since is None else self.blocked_since
                self.set_approach_limit(blocker)
            self.set_state('WAIT_SAFE_GAP')
            self.report_status('UNTRACKED_RTC_RUNNING')
            return
        if self.executing:
            # Stale/missing RTC is not completion. Require an explicit terminal state.
            identity = (self.executing, self.execution_uuid)
            event = self.terminal_events.get(identity)
            if event is None:
                self.set_approach_limit(0.)
                self.set_state('WAIT_SAFE_GAP')
                self.report_status('EXECUTION_UNCONFIRMED')
                return
            self.last_choice = self.executing
            if self.executing.startswith('route_') and event[0] == State.SUCCEEDED and self.map_policy:
                self.detouring = not self.map_policy.on_preferred(
                    self.odom.pose.pose.position, qyaw(self.odom.pose.pose.orientation))
            if self.emergency_executing:
                self.clear_approach_limit()
            self.emergency_executing = False
            self.executing = self.execution_uuid = None
        blocker = self.blocking()
        if blocker is not None:
            self.last_blocked = now
            self.blocked_since = now if self.blocked_since is None else self.blocked_since
            if self.emergency_executing:
                self.set_emergency_limit()
            else:
                # A committed RTC request is still following the current path.
                # Keep the normal stop limit until the owned UUID is actually RUNNING.
                self.set_approach_limit(blocker)
        else:
            self.blocked_since = None
            if now-self.last_blocked >= self.clear_time_s:
                self.clear_approach_limit()
        gate = self.signal_gate(now)
        if gate:
            self.set_state('WAIT_SIGNAL' if gate == 'SIGNAL_WAIT' else 'WAIT_SAFE_GAP')
            self.report_status(gate)
            return
        if self.committed:
            # A successful service response is not proof of execution. Do not send
            # an opposite command while acknowledgement is outstanding.
            if now-self.last_request < self.execution_ack_timeout_s:
                self.report_status('AWAIT_EXECUTION')
                return
            # Timeout alone never authorizes an opposite command. Only a fresh
            # explicitly deactivated waiting status or terminal event closes it.
            event = self.terminal_events.get((self.committed, self.committed_uuid))
            statuses = self.status.get(self.committed, [])
            cancelled = self.fresh('status_'+self.committed, now) and any(
                bytes(st.uuid.uuid) == self.committed_uuid and
                st.state.type == State.WAITING_FOR_EXECUTION and
                st.command_status.type == Command.DEACTIVATE for st in statuses)
            if event is None and not cancelled:
                self.set_approach_limit(0.)
                self.set_state('WAIT_SAFE_GAP')
                self.report_status('APPROVAL_TIMEOUT')
                return
            if event and not self.committed.startswith('route_'):
                self.detouring = True
            self.committed = self.committed_uuid = None
            self.emergency_committed = False
            if self.pending and not self.pending.done():
                self.pending.cancel()
            self.pending = None
        if now-self.last_request < self.retry_interval_s:
            return
        self.set_state('PLAN_RETURN' if self.detouring else 'CRUISE')
        choices = self.safe_choices(('route_left', 'route_right'), blocker)
        if not choices and blocker is not None and now-self.blocked_since >= self.blocked_time_s:
            self.set_state('PLAN_DETOUR')
            choices = self.safe_choices(('left', 'right'), blocker)
        if not choices:
            if self.detouring:
                self.set_approach_limit(0.)
            self.set_state('WAIT_SAFE_GAP' if blocker is not None or self.detouring else 'CRUISE')
            self.report_status('NO_CANDIDATE' if not self.diagnostics else None)
            return
        best = max(choices, key=lambda c: c[0])
        # Prefer an already selected direction within a configurable clearance band.
        best = next((c for c in choices if c[1] == self.last_choice and c[0]+self.switch_hysteresis_m >= best[0]), best)
        clear, key, st, mode = best
        client = self.rtc_clients[key]
        if not client.service_is_ready():
            self.report_status('SERVICE_UNAVAILABLE')
            return
        request = CooperateCommands.Request()
        request.stamp = self.get_clock().now().to_msg()
        request.commands = [CooperateCommand(uuid=st.uuid, module=st.module, command=Command(type=Command.ACTIVATE))]
        self.committed = self.last_choice = key
        self.emergency_committed = mode == 'EMERGENCY'
        self.committed_uuid = bytes(st.uuid.uuid)
        self.last_request = now
        self.set_state('PLAN_RETURN' if key.startswith('route_') else 'PLAN_DETOUR')
        self.pending = client.call_async(request)
        self.pending.add_done_callback(self.on_response)
        self.report_status('APPROVAL_REQUESTED')

    def on_response(self, future):
        if future is not self.pending:
            return
        try:
            response = future.result()
            if response is None or not response.responses or not all(r.success for r in response.responses):
                self.committed = None
                self.emergency_committed = False
                self.get_logger().warning('DETOUR APPROVAL_REJECTED')
        except Exception as error:
            self.committed = None
            self.emergency_committed = False
            self.get_logger().error(f'DETOUR APPROVAL_FAILED: {error}')


def main():
    rclpy.init()
    node = BlockedRouteDetour()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

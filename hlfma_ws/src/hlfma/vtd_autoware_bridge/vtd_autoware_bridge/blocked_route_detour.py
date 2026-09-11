"""정지 차량군 우회용 external-request lane-change RTC 승인 게이트.

Autoware가 좌/우 후보와 안전성 검사를 만들고, 이 노드는 ego가 실제로 정지 차량에
막히기 전 감속하며 일반/우회 차선변경 승인을 중재한다. 조향은 Autoware가 담당한다.
"""
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from autoware_planning_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from autoware_perception_msgs.msg import PredictedObjects
from autoware_internal_planning_msgs.msg import PathWithLaneId, VelocityLimit, VelocityLimitClearCommand, PlanningFactor, PlanningFactorArray
from rclpy.qos import QoSProfile, DurabilityPolicy
from tier4_rtc_msgs.msg import CooperateStatusArray, CooperateCommand, Command, State
from tier4_rtc_msgs.srv import CooperateCommands


def qyaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BlockedRouteDetour(Node):
    def __init__(self):
        super().__init__('blocked_route_detour')
        p = self.declare_parameter
        p('object_stop_speed_mps', 0.3)
        p('blocked_time_s', 0.4); p('detection_distance_m', 80.0); p('input_timeout_s', 1.0)
        # HL FMA 9/11: 45.0 -> 26.0 (JG 값). 좌회전 직후 blocker=40.3m 에서
        #   available = max(0, 40.3-45.0) = 0 이라 속도제한 0 이 나가고, 차가 멈추면
        #   blocker 거리도 안 변해 영구 교착이었다(4회 재현, 정지점 433.5/-24.9).
        #   되돌리려면 45.0
        p('hold_distance_m', 26.0); p('approach_speed_mps', 4.0); p('approach_deceleration_mps2', 1.5)
        # HL FMA 9/11: 접근 속도 하한(JG 값). 위 식이 0 을 뱉는 구간에서도 기어가게 해
        #   교착을 막는다. 실제 정지는 obstacle_stop/AEB 가 담당하고 이 값은
        #   '원하는 속도' 상한일 뿐이다(set_approach_limit 주석 참조). 되돌리려면 0.0
        p('min_approach_speed_mps', 1.0)
        p('clear_time_s', 1.0)
        p('lookahead_m', 100.0); p('path_lateral_margin_m', 2.2); p('retry_interval_s', 0.4)   # HL FMA 9/10: 2.0 이면 13m/s 에서 승인 요청 사이에 26m 를 지나간다. 그동안 모듈은 WaitingForApproval 이라 매 주기 경로를 새로 그려(interface.cpp:110) 경로가 뚝뚝 끊긴다. 0.4 로 줄여 후보가 유효해지는 즉시 승인이 나가게 한다. 되돌리려면 2.0
        g = lambda n: self.get_parameter(n).value
        self.obj_stop_v = float(g('object_stop_speed_mps'))
        self.lookahead, self.margin, self.retry = float(g('lookahead_m')), float(g('path_lateral_margin_m')), float(g('retry_interval_s'))
        self.blocked_time = float(g('blocked_time_s'))
        self.detection_distance = float(g('detection_distance_m'))
        self.input_timeout = float(g('input_timeout_s'))
        self.hold_distance = float(g('hold_distance_m'))
        self.min_approach_speed = float(g('min_approach_speed_mps'))
        self.approach_speed = float(g('approach_speed_mps'))
        self.approach_deceleration = float(g('approach_deceleration_mps2'))
        self.clear_time = float(g('clear_time_s'))
        if min(self.hold_distance, self.approach_speed, self.approach_deceleration) <= 0:
            raise ValueError('Detour approach parameters must be positive')
        self.limit_active = False
        self.last_blocked = -math.inf
        self.committed = None
        self.received = {}
        self.regulatory_factors = {}
        self.pending = None
        self.last_diagnostic = -math.inf
        self.odom = self.objects = self.behavior_path = None; self.candidates = {key: None for key in ('left', 'right', 'route_left', 'route_right')}; self.status = {}
        self.blocked_since = None; self.last_request = 0.0
        self.create_subscription(Odometry, '/localization/kinematic_state', self.on_odom, 10)
        self.create_subscription(PredictedObjects, '/perception/object_recognition/objects', self.on_objects, 10)
        self.create_subscription(PathWithLaneId, '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id', lambda m: self.receive('behavior_path', m), 10)
        self.module_names = {
            'left': 'external_request_lane_change_left',
            'right': 'external_request_lane_change_right',
            'route_left': 'lane_change_left',
            'route_right': 'lane_change_right',
        }
        for key, base in self.module_names.items():
            self.create_subscription(Path, f'/planning/path_candidate/{base}',
                                     lambda m, k=key: self.on_candidate(k, m), 10)
            self.create_subscription(CooperateStatusArray, f'/planning/cooperate_status/{base}',
                                     lambda m, k=key: self.on_status(k, m), 10)
        self.rtc_clients = {key: self.create_client(CooperateCommands,
                            f'/planning/cooperate_commands/{base}')
                            for key, base in self.module_names.items()}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.limit_pub = self.create_publisher(
            VelocityLimit, '/planning/scenario_planning/max_velocity_candidates', qos)
        self.clear_pub = self.create_publisher(
            VelocityLimitClearCommand, '/planning/scenario_planning/clear_velocity_limit', qos)
        for name in ('traffic_light', 'intersection', 'crosswalk', 'stop_line'):
            self.create_subscription(PlanningFactorArray, '/planning/planning_factors/' + name,
                                     lambda m, k=name: self.regulatory_factors.__setitem__(k, m), 10)
        self.create_timer(0.2, self.tick)

    def receive(self, name, message):
        setattr(self, name, message)
        self.received[name] = time.monotonic()

    def on_candidate(self, side, message):
        self.candidates[side] = message
        self.received['candidate_' + side] = time.monotonic()

    def fresh(self, name, now):
        return now - self.received.get(name, -math.inf) <= self.input_timeout

    def on_odom(self, m):
        self.receive('odom', m)

    def on_objects(self, m): self.receive('objects', m)

    def on_status(self, side, m):
        self.status[side] = list(m.statuses)
        self.received['status_' + side] = time.monotonic()

    def planner_stop_before(self, blocker_distance):
        """차선변경 승인 대기 정지점과 교통 규제에 따른 정지를 구별한다.

        Behavior path의 0속도만으로는 사유를 알 수 없다. 신호/교차로 정지
        factor를 추가 게이트로 사용하며 실제 정지는 기존 velocity planner가 담당한다.
        """
        for message in self.regulatory_factors.values():
            for factor in message.factors:
                if factor.behavior != PlanningFactor.STOP:
                    continue
                if any(-1.0 <= point.distance <= blocker_distance + 1.0
                       for point in factor.control_points):
                    return True
        return False

    def corridor(self, path, check_objects=True):
        """Ego 투영점 이후의 실제 후보 구간만 검사한다 (후방 path 길이 제외)."""
        if not path or not self.objects or not self.odom or len(path.points) < 2:
            return -1.0
        pts = [x.pose.position for x in path.points]
        segments = []
        length = 0.0
        for a, b in zip(pts, pts[1:]):
            dx, dy = b.x-a.x, b.y-a.y
            size = math.hypot(dx, dy)
            if size > 1e-6:
                segments.append((a, dx, dy, size, length))
                length += size
        if not segments:
            return -1.0

        def project(p):
            projections = []
            for a, dx, dy, size, start in segments:
                t = max(0.0, min(1.0, ((p.x-a.x)*dx+(p.y-a.y)*dy)/(size*size)))
                distance = math.hypot(p.x-a.x-t*dx, p.y-a.y-t*dy)
                projections.append((distance, start+t*size))
            return min(projections)

        _, ego_s = project(self.odom.pose.pose.position)
        best = min(self.lookahead, length-ego_s)
        for obj in (self.objects.objects if check_objects else []):
            velocity = obj.kinematics.initial_twist_with_covariance.twist.linear
            if math.hypot(velocity.x, velocity.y) > self.obj_stop_v:
                continue
            distance, obj_s = project(obj.kinematics.initial_pose_with_covariance.pose.position)
            if distance < self.margin and obj_s >= ego_s:
                best = min(best, obj_s-ego_s)
        return best

    def blocking(self):
        """정지차를 현재 behavior path에 투영하여 곡선 도로도 경로 거리로 감지한다."""
        if not self.odom or not self.objects or not self.behavior_path:
            return None
        points = [x.point.pose.position for x in self.behavior_path.points]
        if len(points) < 2:
            return None

        def project(position):
            matches = []
            arc = 0.0
            for a, b in zip(points, points[1:]):
                dx, dy = b.x - a.x, b.y - a.y
                length = math.hypot(dx, dy)
                if length < 1e-6:
                    continue
                ratio = max(0.0, min(1.0, ((position.x-a.x)*dx + (position.y-a.y)*dy)/(length*length)))
                lateral = math.hypot(position.x-a.x-ratio*dx, position.y-a.y-ratio*dy)
                matches.append((lateral, arc+ratio*length))
                arc += length
            return min(matches) if matches else (math.inf, 0.0)

        _, ego_arc = project(self.odom.pose.pose.position)
        distances = []
        for obj in self.objects.objects:
            velocity = obj.kinematics.initial_twist_with_covariance.twist.linear
            if math.hypot(velocity.x, velocity.y) > self.obj_stop_v:
                continue
            lateral, arc = project(obj.kinematics.initial_pose_with_covariance.pose.position)
            distance = arc - ego_arc
            if lateral < self.margin and 0.0 < distance < self.detection_distance:
                distances.append(distance)
        return min(distances) if distances else None

    def report_status(self, now):
        if now - self.last_diagnostic < 5.0:
            return
        self.last_diagnostic = now
        missing = [name for name in ('odom', 'objects', 'behavior_path')
                   if not self.fresh(name, now)]
        blocker = self.blocking() if not missing else None
        details = []
        for side in self.candidates:
            candidate = self.candidates[side]
            statuses = self.status.get(side, [])
            details.append(
                f'{side}: points={len(candidate.points) if candidate else 0} '
                f'fresh={self.fresh("candidate_" + side, now)} '
                f'rtc_fresh={self.fresh("status_" + side, now)} '
                f'corridor={self.corridor(candidate):.1f} '
                f'rtc={[(st.safe, st.start_distance, st.finish_distance, st.state.type) for st in statuses]}')
        stop = blocker is not None and self.planner_stop_before(blocker)
        self.get_logger().info(
            f'DETOUR status missing={missing} blocker={blocker} planner_stop={stop}; '
            + '; '.join(details))

    def set_approach_limit(self, blocker):
        # The smoother applies its configured acceleration/jerk constraints. This
        # envelope is a desired speed, not a direct brake command or a stop guarantee.
        available = max(0.0, blocker - self.hold_distance)
        speed = min(self.approach_speed, math.sqrt(2.0 * self.approach_deceleration * available))
        # blocker<=0 은 '지금 세워라' 라는 명시적 호출이므로 그대로 둔다.
        if blocker > 0.0:
            speed = max(speed, self.min_approach_speed)
        msg = VelocityLimit()
        msg.stamp = self.get_clock().now().to_msg()
        msg.sender = 'blocked_route_detour'
        msg.max_velocity = speed
        self.limit_pub.publish(msg)
        self.limit_active = True

    def clear_approach_limit(self):
        if not self.limit_active:
            return
        msg = VelocityLimitClearCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.sender = 'blocked_route_detour'
        msg.command = True
        self.clear_pub.publish(msg)
        self.limit_active = False

    def safe_choices(self, keys, blocker=None):
        now = time.monotonic()
        choices = []
        for key in keys:
            if not self.fresh('candidate_' + key, now) or not self.fresh('status_' + key, now):
                continue
            candidate = self.candidates[key]
            clear = self.corridor(candidate)
            # Do not approve entering a queue just because it starts beyond the
            # shift end. Check the entire available candidate corridor.
            if candidate and key.startswith('route_'):
                available = self.corridor(candidate, check_objects=False)
                if clear < available - 1e-3:
                    continue
            for st in self.status.get(key, []):
                required = st.finish_distance + 5.0
                if blocker is not None:
                    required = max(required, blocker + 5.0)
                if (st.safe and not st.auto_mode
                        and st.state.type == State.WAITING_FOR_EXECUTION
                        and math.isfinite(st.start_distance) and math.isfinite(st.finish_distance)
                        # HL FMA 9/10: 시작점이 자차보다 조금 뒤여도 승인 대상에 남긴다.
                        #   준비시간 오름차순 적용 후 후보가 start=0.32 처럼 0 에 붙어 나오는데,
                        #   >= 0.0 이면 자차가 조금만 더 가도 음수가 되어 그 주기를 통째로 놓치고,
                        #   놓친 만큼 재계획이 이어져 경로가 끊긴다. 되돌리려면 0.0
                        and st.start_distance >= -2.0 and st.finish_distance > 0.0
                        and clear >= required):
                    choices.append((clear, key, st))
        return choices

    def tick(self):
        now = time.monotonic()
        self.report_status(now)
        if (not all(self.fresh(name, now) for name in ('odom', 'objects', 'behavior_path'))
                or not self.behavior_path or len(self.behavior_path.points) < 2):
            self.blocked_since = None
            # Keep an existing restriction if observations disappear while blocked.
            # Never release it using stale/absent perception.
            if self.limit_active:
                self.set_approach_limit(0.0)
            return
        running = [(key, st) for key, statuses in self.status.items()
                   if self.fresh('status_' + key, now) for st in statuses
                   if st.state.type in (State.RUNNING, State.ABORTING)
                   # HL FMA 9/10: 이미 끝난 기동을 RUNNING 으로 붙잡고 있으면 이 가드가 영구히
                   #   걸려 다음 승인 요청을 아예 못 보낸다. 실측: 우측 우회가 start=-46.24
                   #   finish=0.33 로 46m 뒤에서 끝났는데 state=1 이라, lane_change_left 가
                   #   safe=True/start=0.32/finish=27.21 인 정상 후보를 들고도 cmd=0 으로 대기했고
                   #   DETOUR request 가 우측 1건만 나갔다. 종료분은 가드에서 뺀다.
                   #   되돌리려면 아래 finish_distance 조건을 지운다.
                   and not (math.isfinite(st.finish_distance) and st.finish_distance <= 0.5)]
        if running:
            # Never send an opposing command mid-maneuver. Only release the
            # waiting speed limit once the planner reports execution, not on send.
            self.clear_approach_limit()
            self.committed = None
            return

        b = self.blocking()
        if b is not None:
            self.last_blocked = now
            self.blocked_since = now if self.blocked_since is None else self.blocked_since
            self.set_approach_limit(b)
        else:
            self.blocked_since = None
            if now - self.last_blocked < self.clear_time:
                return
            self.clear_approach_limit()

        if self.pending is not None and not self.pending.done():
            return
        # HL FMA 9/10 검토: 여기서 committed 일 때 retry 를 건너뛰어 uuid 교체를 따라잡게
        #   해봤으나 RTC 항목 수가 10 -> 9 로 거의 안 줄었다(플래너 쪽 취소를 막고 나면
        #   churn 자체가 낮아 retry 가 병목이 아니다). 발행률만 올라가므로 되돌렸다.
        if self.committed is not None and now - self.last_request < self.retry:
            return
        if now - self.last_request < self.retry:
            return

        # First retain the original route if its candidate is physically clear.
        # This also provides the return to the left-turn route after passing the queue.
        choices = self.safe_choices(('route_left', 'route_right'), b)
        if not choices and b is not None:
            if now - self.blocked_since < self.blocked_time or self.planner_stop_before(b):
                return
            choices = self.safe_choices(('left', 'right'), b)
        if not choices:
            return
        clear, key, st = max(choices, key=lambda x: x[0])
        cli = self.rtc_clients[key]
        if not cli.service_is_ready():
            return
        req = CooperateCommands.Request()
        req.stamp = self.get_clock().now().to_msg()
        cmd = CooperateCommand(uuid=st.uuid, module=st.module, command=Command(type=Command.ACTIVATE))
        req.commands = [cmd]
        self.pending = cli.call_async(req)
        self.pending.add_done_callback(self.on_response)
        self.last_request = now
        self.committed = key
        self.get_logger().info(f'DETOUR request {key}: blocker={b} corridor={clear:.1f}m')

    def on_response(self, future):
        try:
            response = future.result()
            if not response.responses or not all(r.success for r in response.responses):
                self.get_logger().warning('DETOUR RTC command rejected')
        except Exception as error:
            self.get_logger().error(f'DETOUR RTC command failed: {error}')


def main():
    rclpy.init(); n = BlockedRouteDetour()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()

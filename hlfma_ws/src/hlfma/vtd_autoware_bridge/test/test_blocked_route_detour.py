"""ROS 메시지 단위 테스트와 별도 도메인의 송수신 통합 테스트."""
import time
import math
import unittest
from unittest.mock import Mock
from nav_msgs.msg import Odometry
from autoware_planning_msgs.msg import Path as CandidatePath
from autoware_perception_msgs.msg import PredictedObjects, PredictedObject
from autoware_internal_planning_msgs.msg import PathWithLaneId, PathPointWithLaneId, PlanningFactor, PlanningFactorArray, ControlPoint
from tier4_rtc_msgs.msg import CooperateStatus, Command, State
from vtd_autoware_bridge.blocked_route_detour import BlockedRouteDetour


def path(y=0.0, start=-40, end=120):
    result = PathWithLaneId()
    for x in range(start, end + 1, 5):
        point = PathPointWithLaneId()
        point.point.pose.position.x = float(x)
        point.point.pose.position.y = y if x >= 20 else y * max(0, x) / 20
        point.point.longitudinal_velocity_mps = 10.0
        result.points.append(point)
    return result


def candidate(y=0.0, start=-40, end=120):
    return CandidatePath(points=[p.point for p in path(y, start, end).points])


def vehicle(x, y=0.0):
    obj = PredictedObject()
    obj.kinematics.initial_pose_with_covariance.pose.position.x = float(x)
    obj.kinematics.initial_pose_with_covariance.pose.position.y = float(y)
    return obj


class DetourTests(unittest.TestCase):
    def setUp(self):
        self.node = n = object.__new__(BlockedRouteDetour)
        n.odom = Odometry()
        n.odom.pose.pose.orientation.w = 1.0
        n.odom.twist.twist.linear.x = 12.0
        n.objects = PredictedObjects(objects=[vehicle(60), vehicle(75)])
        n.behavior_path = path()
        n.candidates = {'left': candidate(), 'right': candidate(-3.5), 'route_left': None, 'route_right': None}
        n.status = {side: [CooperateStatus(safe=True, requested=False,
                    start_distance=5.0, finish_distance=45.0)] for side in n.candidates}
        n.lookahead = n.detection_distance = 80.0
        n.margin = 2.2
        n.obj_stop_v = 0.3
        n.input_timeout = 1.0
        n.blocked_time = 0.4
        n.retry = 2.0
        n.last_request = 0.0
        n.blocked_since = time.monotonic() - 0.5
        n.pending = None
        n.committed = None
        n.hold_distance = 45.0
        n.approach_speed = 4.0
        n.approach_deceleration = 1.5
        n.clear_time = 1.0
        n.last_blocked = -float('inf')
        n.limit_active = False
        n.limit_pub = Mock()
        n.clear_pub = Mock()
        n.last_diagnostic = time.monotonic()
        n.regulatory_factors = {}
        n.received = {key: time.monotonic() for key in
                      ('odom', 'objects', 'behavior_path', 'candidate_left',
                       'candidate_right', 'status_left', 'status_right',
                       'candidate_route_left', 'candidate_route_right', 'status_route_left', 'status_route_right')}
        n.rtc_clients = {side: Mock() for side in n.candidates}
        n.get_clock = Mock()
        from builtin_interfaces.msg import Time
        n.get_clock.return_value.now.return_value.to_msg.return_value = Time()
        n.get_logger = Mock()

    def assert_no_request(self):
        self.node.tick()
        for client in self.node.rtc_clients.values():
            client.call_async.assert_not_called()

    def test_moving_ego_approves_clear_right_with_requested_false(self):
        n = self.node
        n.tick()
        n.rtc_clients['left'].call_async.assert_not_called()
        request = n.rtc_clients['right'].call_async.call_args.args[0]
        self.assertEqual(request.commands[0].command.type, Command.ACTIVATE)

    def test_corridor_distance_is_relative_to_ego(self):
        self.assertAlmostEqual(self.node.corridor(candidate()), 60.0)

    def test_short_path_does_not_claim_unobserved_clearance(self):
        self.assertAlmostEqual(self.node.corridor(candidate(end=10)), 10.0)

    def test_stop_line_prevents_detour(self):
        self.node.regulatory_factors['traffic_light'] = PlanningFactorArray(factors=[
            PlanningFactor(behavior=PlanningFactor.STOP, control_points=[ControlPoint(distance=10.0)])])
        self.assert_no_request()

    def test_unsafe_or_invalid_candidate_is_not_approved(self):
        for value in (False, True):
            self.node.status['right'][0].safe = value
            if value:
                self.node.status['right'][0].start_distance = -float('inf')
            self.assert_no_request()

    def test_stale_candidate_is_not_approved(self):
        self.node.received['candidate_right'] -= 2.0
        self.assert_no_request()

    def test_active_maneuver_is_not_interrupted(self):
        self.node.status['left'][0].state.type = State.RUNNING
        self.assert_no_request()

    def test_no_blocker_no_detour(self):
        self.node.objects.objects = []
        self.assert_no_request()

    def test_stale_objects_reset_detection(self):
        self.node.received['objects'] -= 2.0
        self.assert_no_request()
        self.assertIsNone(self.node.blocked_since)

    def test_lateral_moving_object_is_not_stationary(self):
        for obj in self.node.objects.objects:
            obj.kinematics.initial_twist_with_covariance.twist.linear.y = 2.0
        self.assert_no_request()

    def test_no_candidate_brakes_on_early_detection(self):
        n = self.node
        n.candidates = {key: None for key in n.candidates}
        self.assert_no_request()
        self.assertEqual(n.limit_pub.publish.call_args.args[0].max_velocity, 4.0)

    def test_no_candidate_stops_at_reserved_distance(self):
        n = self.node
        n.objects.objects = [vehicle(40)]
        n.candidates = {key: None for key in n.candidates}
        self.assert_no_request()
        self.assertEqual(n.limit_pub.publish.call_args.args[0].max_velocity, 0.0)

    def test_pending_approval_does_not_release_limit(self):
        n = self.node
        n.tick()
        self.assertTrue(n.limit_active)
        n.clear_pub.publish.assert_not_called()

    def test_limit_released_when_execution_confirmed(self):
        n = self.node
        n.limit_active = True
        n.status['right'][0].state.type = State.RUNNING
        self.assert_no_request()
        self.assertEqual(n.clear_pub.publish.call_args.args[0].sender, 'blocked_route_detour')
        self.assertFalse(n.limit_active)

    def test_stale_perception_holds_existing_limit(self):
        n = self.node
        n.limit_active = True
        n.received['objects'] -= 2.0
        self.assert_no_request()
        self.assertEqual(n.limit_pub.publish.call_args.args[0].max_velocity, 0.0)
        n.clear_pub.publish.assert_not_called()

    def test_clear_route_candidate_is_approved_without_blocker(self):
        n = self.node
        n.objects.objects = []
        n.candidates['route_left'] = candidate(3.5)
        n.tick()
        n.rtc_clients['route_left'].call_async.assert_called_once()
        n.rtc_clients['right'].call_async.assert_not_called()

    def test_queue_beyond_shift_end_blocks_route_approval(self):
        n = self.node
        n.candidates['route_left'] = candidate()
        n.tick()
        n.rtc_clients['route_left'].call_async.assert_not_called()
        n.rtc_clients['right'].call_async.assert_called_once()

    def test_return_only_after_left_corridor_clears(self):
        n = self.node
        n.candidates['route_left'] = candidate(3.5)
        n.objects.objects = [vehicle(60, 3.5)]
        self.assert_no_request()
        n.objects.objects = []
        n.tick()
        n.rtc_clients['route_left'].call_async.assert_called_once()

    def test_limit_cleared_only_after_stable_clear_observations(self):
        n = self.node
        n.limit_active = True
        n.objects.objects = []
        n.last_blocked = time.monotonic()
        self.assert_no_request()
        n.clear_pub.publish.assert_not_called()
        n.last_blocked -= 2.0
        n.tick()
        n.clear_pub.publish.assert_called_once()


    def test_lane_change_waiting_stop_is_not_misread_as_red_light(self):
        n = self.node
        n.behavior_path.points[10].point.longitudinal_velocity_mps = 0.0
        n.tick()
        n.rtc_clients['right'].call_async.assert_called_once()

    def test_curved_path_detects_blocker_outside_ego_heading(self):
        n = self.node
        for point in n.behavior_path.points:
            x = point.point.pose.position.x
            point.point.pose.position.y = max(0.0, x - 20.0)
        n.objects.objects = [vehicle(40, 20)]
        self.assertAlmostEqual(n.blocking(), 20.0 + math.sqrt(800.0))



class CandidateTransportTests(unittest.TestCase):
    def test_actual_path_publisher_reaches_detour_subscription(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        rclpy.init()
        detour = BlockedRouteDetour()
        publisher_node = Node('detour_candidate_transport_test')
        executor = SingleThreadedExecutor()
        executor.add_node(detour)
        executor.add_node(publisher_node)
        topic = '/planning/path_candidate/external_request_lane_change_right'
        publisher = publisher_node.create_publisher(CandidatePath, topic, 10)
        message = candidate(-3.5)
        try:
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and detour.candidates['right'] is None:
                publisher.publish(message)
                executor.spin_once(timeout_sec=0.05)
            self.assertIsNotNone(detour.candidates['right'])
            self.assertEqual(len(detour.candidates['right'].points), len(message.points))
            self.assertTrue(detour.fresh('candidate_right', time.monotonic()))
            detour.odom = Odometry()
            detour.objects = PredictedObjects(objects=[vehicle(60)])
            self.assertGreater(detour.corridor(detour.candidates['right']), 60.0)
        finally:
            executor.shutdown()
            publisher_node.destroy_node()
            detour.destroy_node()
            rclpy.try_shutdown()

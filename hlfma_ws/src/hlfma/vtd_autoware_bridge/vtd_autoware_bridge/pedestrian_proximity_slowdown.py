"""Preventive slowdown for pedestrians near the ego trajectory.

This node only requests a moderate speed limit. Actual predicted collisions and stops remain the
responsibility of the motion velocity planner run_out module.
"""
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from autoware_internal_planning_msgs.msg import PathWithLaneId, VelocityLimit, VelocityLimitClearCommand
from autoware_perception_msgs.msg import ObjectClassification, PredictedObjects
from nav_msgs.msg import Odometry


class PedestrianProximitySlowdown(Node):
    def __init__(self):
        super().__init__('pedestrian_proximity_slowdown')
        p = self.declare_parameter
        p('lateral_distance_m', 10.0)
        p('lookahead_distance_m', 50.0)
        p('normal_slowdown_speed_mps', 9.7222)
        p('school_zone_slowdown_speed_mps', 6.94)
        p('school_zone_limit_kph', 30.0)
        p('map_osm', '')
        p('clear_time_s', 1.0)
        p('input_timeout_s', 1.0)
        g = lambda name: self.get_parameter(name).value
        self.lateral = float(g('lateral_distance_m'))
        self.lookahead = float(g('lookahead_distance_m'))
        self.normal_speed = float(g('normal_slowdown_speed_mps'))
        self.school_zone_speed = float(g('school_zone_slowdown_speed_mps'))
        self.school_zone_limit_kph = float(g('school_zone_limit_kph'))
        self.school_zone_lane_ids = self.load_school_zone_lane_ids(str(g('map_osm')))
        self.clear_time = float(g('clear_time_s'))
        self.timeout = float(g('input_timeout_s'))
        self.path = self.objects = self.odom = None
        self.received = {}
        self.limit_active = False
        self.last_nearby = -math.inf
        self.create_subscription(PathWithLaneId, '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id', lambda m: self.receive('path', m), 10)
        self.create_subscription(PredictedObjects, '/perception/object_recognition/objects', lambda m: self.receive('objects', m), 10)
        self.create_subscription(Odometry, '/localization/kinematic_state', lambda m: self.receive('odom', m), 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.limit_pub = self.create_publisher(VelocityLimit, '/planning/scenario_planning/max_velocity_candidates', qos)
        self.clear_pub = self.create_publisher(VelocityLimitClearCommand, '/planning/scenario_planning/clear_velocity_limit', qos)
        self.create_timer(0.2, self.tick)

    def load_school_zone_lane_ids(self, map_osm):
        lane_ids = set()
        if not map_osm:
            self.get_logger().warning('map_osm is empty; school-zone pedestrian slowdown is unavailable')
            return lane_ids
        try:
            for _, elem in ET.iterparse(map_osm, events=('end',)):
                if elem.tag != 'relation':
                    if elem.tag in ('node', 'way'):
                        elem.clear()
                    continue
                tags = {tag.get('k'): tag.get('v') for tag in elem.findall('tag')}
                if tags.get('type') == 'lanelet' and 'speed_limit' in tags:
                    speed_kph = float(tags['speed_limit'].split()[0])
                    if speed_kph <= self.school_zone_limit_kph:
                        lane_ids.add(int(elem.get('id')))
                elem.clear()
        except (OSError, ValueError, ET.ParseError) as error:
            self.get_logger().error(f'failed to load school-zone lanelets from {map_osm}: {error}')
        self.get_logger().info(f'loaded {len(lane_ids)} school-zone lanelets')
        return lane_ids

    def receive(self, name, msg):
        setattr(self, name, msg)
        self.received[name] = time.monotonic()

    @staticmethod
    def is_pedestrian(obj):
        return any(c.label == ObjectClassification.PEDESTRIAN and c.probability > 0.0 for c in obj.classification)

    def nearby_pedestrian(self):
        if not self.path or not self.objects or not self.odom or len(self.path.points) < 2:
            return False
        pts = [q.point.pose.position for q in self.path.points]
        segments, total = [], 0.0
        for a, b in zip(pts, pts[1:]):
            dx, dy = b.x - a.x, b.y - a.y
            length = math.hypot(dx, dy)
            if length > 1e-6:
                segments.append((a, dx, dy, length, total))
                total += length
        if not segments:
            return False

        def project(pos):
            matches = []
            for a, dx, dy, length, start in segments:
                ratio = max(0.0, min(1.0, ((pos.x-a.x)*dx + (pos.y-a.y)*dy)/(length*length)))
                lateral = math.hypot(pos.x-a.x-ratio*dx, pos.y-a.y-ratio*dy)
                matches.append((lateral, start + ratio*length))
            return min(matches)

        _, ego_s = project(self.odom.pose.pose.position)
        for obj in self.objects.objects:
            if not self.is_pedestrian(obj):
                continue
            lateral, obj_s = project(obj.kinematics.initial_pose_with_covariance.pose.position)
            ahead = obj_s - ego_s
            if 0.0 <= ahead <= self.lookahead and lateral <= self.lateral:
                return True
        return False

    def in_school_zone(self):
        if not self.path or not self.odom or not self.path.points:
            return False
        ego = self.odom.pose.pose.position
        nearest = min(
            self.path.points,
            key=lambda q: (q.point.pose.position.x - ego.x) ** 2 +
            (q.point.pose.position.y - ego.y) ** 2)
        return any(lane_id in self.school_zone_lane_ids for lane_id in nearest.lane_ids)

    def publish_limit(self):
        msg = VelocityLimit()
        msg.stamp = self.get_clock().now().to_msg()
        msg.sender = 'pedestrian_proximity_slowdown'
        msg.max_velocity = self.school_zone_speed if self.in_school_zone() else self.normal_speed
        self.limit_pub.publish(msg)
        self.limit_active = True

    def clear_limit(self):
        if not self.limit_active:
            return
        msg = VelocityLimitClearCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.sender = 'pedestrian_proximity_slowdown'
        msg.command = True
        self.clear_pub.publish(msg)
        self.limit_active = False

    def tick(self):
        now = time.monotonic()
        if not all(now - self.received.get(k, -math.inf) <= self.timeout for k in ('path', 'objects', 'odom')):
            return
        if self.nearby_pedestrian():
            self.last_nearby = now
            self.publish_limit()
        elif now - self.last_nearby >= self.clear_time:
            self.clear_limit()


def main():
    rclpy.init()
    node = PedestrianProximitySlowdown()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

"""VTD(HL FMA TCP 9910) <-> Autoware 브리지 노드.

수신(DataPacket 1109B @20Hz) → Autoware 토픽:
  ego pose        → /localization/kinematic_state (Odometry), /tf(map→base_link),
                    /localization/acceleration, /vehicle/status/velocity_report·steering_report 등
  objects[30]     → /perception/object_recognition/objects (PredictedObjects, 크기 휴리스틱 분류)
  trafficLight    → /perception/traffic_light_recognition/traffic_signals
                    (OSM의 xodr_signal_id 태그로 lanelet2 regulatory element id 매핑)

송신(CtrlPacket 9B @20Hz) ← Autoware:
  /control/command/control_cmd (Control): steering_tire_angle·acceleration
  /control/command/turn_indicators_cmd  : 방향지시등

조향 부호(9/1 실측 확정): 패킷 +steer = 좌회전(CCW) = Autoware 규약과 동일 → steer_sign=+1.0.
좌표계: VTD 월드좌표 = map 프레임(Autoware local projector, 변환 없음). ego 원점 = 후륜축.
"""
import math
import socket
import struct
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import AccelWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from unique_identifier_msgs.msg import UUID

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import (
    ControlModeReport, GearReport, SteeringReport, TurnIndicatorsCommand,
    TurnIndicatorsReport, HazardLightsReport, VelocityReport,
)
from autoware_perception_msgs.msg import (
    ObjectClassification, PredictedObject, PredictedObjectKinematics,
    PredictedObjects, PredictedPath, Shape,
    TrafficLightElement, TrafficLightGroup, TrafficLightGroupArray,
)

from .protocol import DATA_SIZE, unpack_data, pack_ctrl

MAX_STEER_RAD = 0.48
ACCEL_MIN, ACCEL_MAX = -6.0, 3.0

# VTD tl_state → (color, shape, status)
_TL = TrafficLightElement
TL_STATE_MAP = {
    1: [(_TL.RED, _TL.CIRCLE, _TL.SOLID_ON)],
    2: [(_TL.AMBER, _TL.CIRCLE, _TL.SOLID_ON)],
    3: [(_TL.GREEN, _TL.CIRCLE, _TL.SOLID_ON)],
    4: [(_TL.RED, _TL.CIRCLE, _TL.SOLID_ON), (_TL.GREEN, _TL.LEFT_ARROW, _TL.SOLID_ON)],
    5: [(_TL.GREEN, _TL.CIRCLE, _TL.SOLID_ON), (_TL.GREEN, _TL.LEFT_ARROW, _TL.SOLID_ON)],
    6: [(_TL.RED, _TL.CIRCLE, _TL.FLASHING)],
}


def yaw_to_quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def load_tl_mapping(osm_path: str):
    """OSM에서 xodr_signal_id → regulatory element relation id 매핑 생성."""
    mapping = {}
    for _, elem in ET.iterparse(osm_path):
        if elem.tag != 'relation':
            continue
        tags = {t.get('k'): t.get('v') for t in elem.findall('tag')}
        if tags.get('subtype') == 'traffic_light' and 'xodr_signal_id' in tags:
            mapping[int(tags['xodr_signal_id'])] = int(elem.get('id'))
        elem.clear()
    return mapping


class VtdAutowareBridge(Node):
    def __init__(self):
        super().__init__('vtd_autoware_bridge')
        self.declare_parameter('vtd_host', '192.168.50.11')
        self.declare_parameter('vtd_port', 9910)
        self.declare_parameter('steer_sign', 1.0)   # 실측 확정(9/1): 패킷 +steer=좌(CCW)=Autoware와 동일
        self.declare_parameter('map_osm', '')
        self.declare_parameter('ctrl_rate_hz', 20.0)

        self.host = self.get_parameter('vtd_host').value
        self.port = int(self.get_parameter('vtd_port').value)
        self.steer_sign = float(self.get_parameter('steer_sign').value)

        osm = self.get_parameter('map_osm').value
        self.tl_map = {}
        if osm:
            t0 = time.time()
            try:
                self.tl_map = load_tl_mapping(osm)
                self.get_logger().info(
                    f'신호등 매핑 {len(self.tl_map)}개 로드 ({time.time()-t0:.1f}s): {osm}')
            except Exception as e:
                self.get_logger().error(f'OSM 신호등 매핑 실패: {e}')
        else:
            self.get_logger().warning('map_osm 미지정 → 신호등 토픽 발행 안 함')

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_odom = self.create_publisher(Odometry, '/localization/kinematic_state', qos)
        self.pub_accel = self.create_publisher(
            AccelWithCovarianceStamped, '/localization/acceleration', qos)
        self.pub_vel = self.create_publisher(VelocityReport, '/vehicle/status/velocity_report', qos)
        self.pub_steer = self.create_publisher(SteeringReport, '/vehicle/status/steering_report', qos)
        self.pub_gear = self.create_publisher(GearReport, '/vehicle/status/gear_status', qos)
        self.pub_mode = self.create_publisher(ControlModeReport, '/vehicle/status/control_mode', qos)
        self.pub_turn_rep = self.create_publisher(
            TurnIndicatorsReport, '/vehicle/status/turn_indicators_status', qos)
        self.pub_hazard = self.create_publisher(
            HazardLightsReport, '/vehicle/status/hazard_lights_status', qos)
        self.pub_objects = self.create_publisher(
            PredictedObjects, '/perception/object_recognition/objects', qos)
        self.pub_tl = self.create_publisher(
            TrafficLightGroupArray, '/perception/traffic_light_recognition/traffic_signals', qos)
        self.tf_br = TransformBroadcaster(self)

        self.sub_ctrl = self.create_subscription(
            Control, '/control/command/control_cmd', self.on_control, 1)
        self.sub_turn = self.create_subscription(
            TurnIndicatorsCommand, '/control/command/turn_indicators_cmd', self.on_turn, 1)

        # 최신 명령 (Autoware 부호 기준)
        self.cmd_lock = threading.Lock()
        self.cmd_steer = 0.0
        self.cmd_accel = 0.0
        self.cmd_turn = 0     # VTD: 0=끔/1=좌/2=우
        self.ctrl_count = 0

        # 속도/가속 추정용 이전 상태
        self.prev = None      # (t, x, y, yaw)
        self.vx_f = 0.0
        self.wz_f = 0.0
        self.ax_f = 0.0
        self.prev_vx = 0.0

        self.sock = None
        self.connected = False
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        period = 1.0 / float(self.get_parameter('ctrl_rate_hz').value)
        self.create_timer(period, self.tx_tick)
        self.create_timer(5.0, self.report_tick)
        self.rx_count = 0

    # ---------------- RX: VTD → Autoware ----------------

    def rx_loop(self):
        buf = b''
        while rclpy.ok():
            if self.sock is None:
                try:
                    self.sock = socket.create_connection((self.host, self.port), timeout=3.0)
                    self.sock.settimeout(2.0)
                    self.connected = True
                    buf = b''
                    self.get_logger().info(f'VTD 연결됨 {self.host}:{self.port}')
                except OSError as e:
                    self.connected = False
                    self.get_logger().warning(f'VTD 연결 실패({e}), 2초 후 재시도', throttle_duration_sec=10.0)
                    time.sleep(2.0)
                    continue
            try:
                chunk = self.sock.recv(65536)
                if not chunk:
                    raise ConnectionError('closed')
                buf += chunk
            except (OSError, ConnectionError) as e:
                self.get_logger().warning(f'VTD 수신 오류({e}) → 재연결')
                try:
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None
                self.connected = False
                continue
            n = len(buf) // DATA_SIZE
            if n == 0:
                continue
            pkt = buf[(n - 1) * DATA_SIZE: n * DATA_SIZE]   # 밀리면 최신만
            buf = buf[n * DATA_SIZE:]
            try:
                self.publish_state(unpack_data(pkt))
                self.rx_count += 1
            except Exception as e:
                self.get_logger().error(f'상태 발행 오류: {e}')

    def publish_state(self, st):
        now = self.get_clock().now()
        stamp = now.to_msg()
        t = now.nanoseconds * 1e-9

        # 속도·각속도·가속 추정 (pose 미분 + 저역통과)
        if self.prev is not None:
            dt = t - self.prev[0]
            if 1e-4 < dt < 0.5:
                dx, dy = st.x - self.prev[1], st.y - self.prev[2]
                v = math.hypot(dx, dy) / dt
                # 진행 방향 부호 (후진 감지)
                if dx * math.cos(st.heading) + dy * math.sin(st.heading) < 0:
                    v = -v
                dyaw = (st.heading - self.prev[3] + math.pi) % (2 * math.pi) - math.pi
                a = 0.35
                self.vx_f += a * (v - self.vx_f)
                self.wz_f += a * (dyaw / dt - self.wz_f)
                self.ax_f += a * ((self.vx_f - self.prev_vx) / dt - self.ax_f)
                self.prev_vx = self.vx_f
        self.prev = (t, st.x, st.y, st.heading)
        qx, qy, qz, qw = yaw_to_quat(st.heading)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = st.x
        odom.pose.pose.position.y = st.y
        odom.pose.pose.position.z = st.z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = odom.pose.covariance[7] = odom.pose.covariance[14] = 0.01
        odom.pose.covariance[21] = odom.pose.covariance[28] = odom.pose.covariance[35] = 0.01
        odom.twist.twist.linear.x = self.vx_f
        odom.twist.twist.angular.z = self.wz_f
        self.pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = st.x
        tf.transform.translation.y = st.y
        tf.transform.translation.z = st.z
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_br.sendTransform(tf)

        acc = AccelWithCovarianceStamped()
        acc.header.stamp = stamp
        acc.header.frame_id = 'base_link'
        acc.accel.accel.linear.x = self.ax_f
        self.pub_accel.publish(acc)

        vel = VelocityReport()
        vel.header.stamp = stamp
        vel.header.frame_id = 'base_link'
        vel.longitudinal_velocity = self.vx_f
        vel.heading_rate = self.wz_f
        self.pub_vel.publish(vel)

        steer = SteeringReport()
        steer.stamp = stamp
        with self.cmd_lock:
            steer.steering_tire_angle = self.cmd_steer
        self.pub_steer.publish(steer)

        gear = GearReport()
        gear.stamp = stamp
        gear.report = GearReport.DRIVE
        self.pub_gear.publish(gear)

        mode = ControlModeReport()
        mode.stamp = stamp
        mode.mode = ControlModeReport.AUTONOMOUS
        self.pub_mode.publish(mode)

        turn = TurnIndicatorsReport()
        turn.stamp = stamp
        with self.cmd_lock:
            turn.report = {0: TurnIndicatorsReport.DISABLE,
                           1: TurnIndicatorsReport.ENABLE_LEFT,
                           2: TurnIndicatorsReport.ENABLE_RIGHT}[self.cmd_turn]
        self.pub_turn_rep.publish(turn)

        hz = HazardLightsReport()
        hz.stamp = stamp
        hz.report = HazardLightsReport.DISABLE
        self.pub_hazard.publish(hz)

        self.publish_objects(st, stamp)
        self.publish_traffic_light(st, stamp)

    def publish_objects(self, st, stamp):
        msg = PredictedObjects()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        for (oid, x, y, z, heading, speed, length, width, height) in st.objects:
            obj = PredictedObject()
            obj.object_id = UUID(uuid=list(struct.pack('<IIII', int(oid) & 0xFFFFFFFF, 0, 0, 0)))
            obj.existence_probability = 1.0
            cls = ObjectClassification()
            if length < 1.2 and width < 1.2:
                cls.label = ObjectClassification.PEDESTRIAN
            elif length < 2.8:
                cls.label = ObjectClassification.MOTORCYCLE
            else:
                cls.label = ObjectClassification.CAR
            cls.probability = 1.0
            obj.classification.append(cls)

            k = PredictedObjectKinematics()
            qx, qy, qz, qw = yaw_to_quat(heading)
            p = k.initial_pose_with_covariance.pose
            p.position.x, p.position.y, p.position.z = x, y, z
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
            k.initial_twist_with_covariance.twist.linear.x = speed
            # 등속 직진 예측 경로 (8초, 0.5초 간격)
            path = PredictedPath()
            path.time_step.sec = 0
            path.time_step.nanosec = 500_000_000
            path.confidence = 1.0
            for i in range(17):
                dt = 0.5 * i
                pp = type(p)()  # geometry_msgs/Pose
                pp.position.x = x + speed * dt * math.cos(heading)
                pp.position.y = y + speed * dt * math.sin(heading)
                pp.position.z = z
                pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = qx, qy, qz, qw
                path.path.append(pp)
            k.predicted_paths.append(path)
            obj.kinematics = k

            obj.shape.type = Shape.BOUNDING_BOX
            obj.shape.dimensions.x = max(length, 0.3)
            obj.shape.dimensions.y = max(width, 0.3)
            obj.shape.dimensions.z = height if height > 0.1 else 1.6
            msg.objects.append(obj)
        self.pub_objects.publish(msg)

    def publish_traffic_light(self, st, stamp):
        if not self.tl_map:
            return
        msg = TrafficLightGroupArray()
        msg.stamp = stamp
        if st.tl_id != 0 and st.tl_state in TL_STATE_MAP:
            group_id = self.tl_map.get(st.tl_id)
            if group_id is None:
                self.get_logger().warning(
                    f'trafficLightId {st.tl_id} 매핑 없음', throttle_duration_sec=10.0)
            else:
                g = TrafficLightGroup()
                g.traffic_light_group_id = group_id
                for color, shape, status in TL_STATE_MAP[st.tl_state]:
                    e = TrafficLightElement()
                    e.color, e.shape, e.status, e.confidence = color, shape, status, 1.0
                    g.elements.append(e)
                msg.traffic_light_groups.append(g)
        self.pub_tl.publish(msg)

    # ---------------- TX: Autoware → VTD ----------------

    def on_control(self, msg: Control):
        s = msg.lateral.steering_tire_angle
        a = msg.longitudinal.acceleration
        if not (math.isfinite(s) and math.isfinite(a)):
            self.get_logger().warning('control_cmd에 NaN/Inf → 무시', throttle_duration_sec=5.0)
            return
        with self.cmd_lock:
            self.cmd_steer = max(-MAX_STEER_RAD, min(MAX_STEER_RAD, s))
            self.cmd_accel = max(ACCEL_MIN, min(ACCEL_MAX, a))
            self.ctrl_count += 1

    def on_turn(self, msg: TurnIndicatorsCommand):
        with self.cmd_lock:
            self.cmd_turn = {TurnIndicatorsCommand.ENABLE_LEFT: 1,
                             TurnIndicatorsCommand.ENABLE_RIGHT: 2}.get(msg.command, 0)

    def tx_tick(self):
        if not self.connected or self.sock is None:
            return
        with self.cmd_lock:
            pkt = pack_ctrl(self.steer_sign * self.cmd_steer, self.cmd_accel, self.cmd_turn)
        try:
            self.sock.sendall(pkt)
        except OSError:
            pass  # rx_loop이 재연결 처리

    def report_tick(self):
        self.get_logger().info(
            f'rx {self.rx_count} pkts, ctrl_cmd {self.ctrl_count}건, 연결={self.connected}',
            throttle_duration_sec=0.0)


def main():
    rclpy.init()
    node = VtdAutowareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

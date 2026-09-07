"""VTD(HL FMA TCP 9910) <-> Autoware 브리지 노드 (vehicle / localization / perception 어댑터).

Autoware에서 끈 서브시스템(측위·인지·차량 인터페이스)이 원래 발행하던 토픽을 VTD 데이터로 대신 공급하고,
Autoware의 제어 출력을 9B 패킷으로 돌려보낸다. 토픽 이름은 autoware_launch의 remap과 동일해야 한다
(검증: tools/check_topic_contract.sh — 구독자만 있고 발행자 없는 토픽이 0이어야 함).

수신 DataPacket(1109B @20Hz) → 발행
  ego pose      /localization/kinematic_state (Odometry, map→base_link), /tf, /localization/acceleration
                /localization/initialization_state (첫 패킷 수신 후 INITIALIZED)
  차량 상태     /vehicle/status/velocity_status, steering_status, gear_status, control_mode,
                turn_indicators_status, hazard_lights_status
  objects[30]   /perception/object_recognition/objects (PredictedObjects)
  신호등        /perception/traffic_light_recognition/traffic_signals (TrafficLightGroupArray)
                — 경로상 다음 정지선의 모든 규제요소에 "가라/서라"(GREEN/RED CIRCLE)만 실어 보냄
  더미 인지     /perception/obstacle_segmentation/pointcloud (빈 점군), /perception/occupancy_grid_map/map (전부 free)
                — motion/behavior_velocity_planner의 필수 구독(코드에 고정)을 채우기 위한 임시안 (개발계획_0902 §4-1)
  이벤트        /vtd/respawn (std_msgs/Empty) — 위치 점프 감지 시
  원본 패킷     /vtd/raw_rx (1109B), /vtd/raw_tx (9B) — ros2 bag 기록용 (tools/record.sh)

송신 CtrlPacket(9B @20Hz) ← /control/command/control_cmd, /control/command/turn_indicators_cmd
  워치독: 제어 명령이 watchdog_timeout 이상 끊기면 조향 유지 + failsafe_accel 로 감속

좌표계: VTD 월드 = Autoware map (local projector), ego 원점 = 후륜축 = base_link.
조향 부호: 패킷 +steer = 좌회전 = Autoware 규약 (9/1 실측) → steer_sign=+1.
속도·가속: 패킷에 없어 pose 차분 + 저역통과로 추정 (위치 점프 시 리셋).
"""
import math
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Empty, UInt8MultiArray
from geometry_msgs.msg import AccelWithCovarianceStamped, TransformStamped, Pose
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu, PointCloud2, PointField
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
from autoware_planning_msgs.msg import LaneletRoute
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState

from .protocol import DATA_SIZE, unpack_data, pack_ctrl
from .osm_map import OsmMap
from .tl_router import TrafficLightRouter

MAX_STEER_RAD = 0.48
ACCEL_MIN, ACCEL_MAX = -6.0, 3.0
_TL = TrafficLightElement

# VTD tl_state (대회정보.md §6): 0 미할당 / 1 적 / 2 황 / 3 녹 / 4 좌회전 / 5 녹+좌 / 6 점멸
STATE_NAME = {0: 'none', 1: 'red', 2: 'amber', 3: 'green', 4: 'left', 5: 'green+left', 6: 'flash'}


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class VtdAutowareBridge(Node):
    def __init__(self):
        super().__init__('vtd_autoware_bridge')
        dp = self.declare_parameter
        dp('vtd_host', '192.168.50.11')
        dp('vtd_port', 9910)
        dp('steer_sign', 1.0)
        dp('map_osm', '')
        dp('ctrl_rate_hz', 20.0)
        dp('watchdog_timeout', 0.4)      # [s] 제어 명령 두절 판정
        dp('failsafe_accel', -3.0)       # [m/s²] 두절 시 감속
        dp('jump_reset_dist', 2.0)       # [m] 한 프레임 이동이 이보다 크면 리스폰으로 판정
        dp('ego_z_mode', 'raw')          # 'raw': VTD z 그대로 (맵에 elevation 반영됨, 9/2) / 'zero': 맵이 z=0일 때
        dp('steer_report_tau', 0.2)      # [s] 조향 보고 1차 지연 (실측 조향 없음)
        dp('state4_go', True)            # state 4(적+좌회전 화살표)를 '가라'로 (개발계획_0902 §4-4)
        dp('flash_stop_time', 0.5)       # [s] state 6: 정지 유지 후 통과
        dp('flash_stop_dist', 8.0)       # [m] state 6: 정지선까지 이 거리 안에서 정지해야 인정
        dp('publish_dummy_perception', True)
        dp('predict_horizon', 4.0)       # [s] objects 예측 경로 길이
        dp('report_period', 5.0)

        g = lambda k: self.get_parameter(k).value
        self.host, self.port = g('vtd_host'), int(g('vtd_port'))
        self.steer_sign = float(g('steer_sign'))
        self.watchdog_timeout = float(g('watchdog_timeout'))
        self.failsafe_accel = float(g('failsafe_accel'))
        self.jump_reset_dist = float(g('jump_reset_dist'))
        self.ego_z_zero = g('ego_z_mode') == 'zero'
        self.steer_tau = float(g('steer_report_tau'))
        self.state4_go = bool(g('state4_go'))
        self.flash_stop_time = float(g('flash_stop_time'))
        self.flash_stop_dist = float(g('flash_stop_dist'))
        self.predict_horizon = float(g('predict_horizon'))

        # 맵 + 신호등 라우터
        self.tl_router = None
        osm = g('map_osm')
        if osm:
            t0 = time.time()
            try:
                self.tl_router = TrafficLightRouter(OsmMap(osm, self.get_logger()), self.get_logger())
                self.get_logger().info(f'맵 로드 {time.time() - t0:.1f}s: {osm}')
            except Exception as e:
                self.get_logger().error(f'맵 로드 실패 → 신호등 발행 안 함: {e}')
        else:
            self.get_logger().warning('map_osm 미지정 → 신호등 토픽 발행 안 함')

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        mk = self.create_publisher
        self.pub_odom = mk(Odometry, '/localization/kinematic_state', qos)
        self.pub_accel = mk(AccelWithCovarianceStamped, '/localization/acceleration', qos)
        self.pub_init = mk(LocalizationInitializationState, '/localization/initialization_state', latched)
        self.pub_vel = mk(VelocityReport, '/vehicle/status/velocity_status', qos)
        self.pub_imu = mk(Imu, '/sensing/imu/imu_data', qos)  # VTD는 IMU 미제공 → ego 운동에서 합성(AEB 등 표준입력)
        self.pub_steer = mk(SteeringReport, '/vehicle/status/steering_status', qos)
        self.pub_gear = mk(GearReport, '/vehicle/status/gear_status', qos)
        self.pub_mode = mk(ControlModeReport, '/vehicle/status/control_mode', qos)
        self.pub_turn_rep = mk(TurnIndicatorsReport, '/vehicle/status/turn_indicators_status', qos)
        self.pub_hazard = mk(HazardLightsReport, '/vehicle/status/hazard_lights_status', qos)
        self.pub_objects = mk(PredictedObjects, '/perception/object_recognition/objects', qos)
        self.pub_tl = mk(TrafficLightGroupArray, '/perception/traffic_light_recognition/traffic_signals', qos)
        self.pub_respawn = mk(Empty, '/vtd/respawn', qos)
        # 원본 패킷 기록용 (ros2 bag 에 담기도록): 수신 DataPacket 1109B / 송신 CtrlPacket 9B
        self.pub_raw_rx = mk(UInt8MultiArray, '/vtd/raw_rx', 10)
        self.pub_raw_tx = mk(UInt8MultiArray, '/vtd/raw_tx', 10)
        self.pub_pc = self.pub_ogm = None
        if bool(g('publish_dummy_perception')):
            self.pub_pc = mk(PointCloud2, '/perception/obstacle_segmentation/pointcloud', qos)
            self.pub_ogm = mk(OccupancyGrid, '/perception/occupancy_grid_map/map', qos)
            self.create_timer(0.1, self.dummy_perception_tick)
        self.tf_br = TransformBroadcaster(self)

        self.create_subscription(Control, '/control/command/control_cmd', self.on_control, 1)
        self.create_subscription(TurnIndicatorsCommand, '/control/command/turn_indicators_cmd', self.on_turn, 1)
        self.create_subscription(LaneletRoute, '/planning/mission_planning/route', self.on_route, latched)

        # 제어 명령 (Autoware 부호)
        self.cmd_lock = threading.Lock()
        self.cmd_steer = 0.0
        self.cmd_accel = 0.0
        self.cmd_turn = 0
        self.ctrl_count = 0
        self.last_cmd_time = None
        self.watchdog_active = False

        # 상태 추정
        self.prev = None
        self.vx_f = self.wz_f = self.ax_f = self.prev_vx = 0.0
        self.steer_rep = 0.0
        self.last_pose = None          # (x, y, z, yaw) 더미 인지·로그용
        self.obj_hist = {}             # id -> (t, x, y, heading)
        self.initialized = False

        # 신호등 상태
        self.tl_last = {'state': 0, 'entry': None, 'dist': None, 'color': None, 'groups': 0}
        self.flash = {'lanelet': None, 'stopped_since': None, 'go': False}

        # 통신
        self.sock = None
        self.connected = False
        self.rx_count = 0
        self._rx_count_prev = 0
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.create_timer(1.0 / float(g('ctrl_rate_hz')), self.tx_tick)
        self.create_timer(1.0, self.init_state_tick)
        self.create_timer(float(g('report_period')), self.report_tick)
        self.get_logger().info(f'브리지 시작: VTD {self.host}:{self.port}, state4_go={self.state4_go}, '
                               f'ego_z={"0" if self.ego_z_zero else "raw"}')

    # ------------------------------------------------------------ RX
    def rx_loop(self):
        buf = b''
        while rclpy.ok():
            if self.sock is None:
                try:
                    s = socket.create_connection((self.host, self.port), timeout=3.0)
                    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    s.settimeout(2.0)
                    self.sock = s
                    self.connected = True
                    buf = b''
                    self.get_logger().info(f'VTD 연결됨 {self.host}:{self.port}')
                except OSError as e:
                    self.connected = False
                    self.get_logger().warning(f'VTD 연결 실패({e}), 재시도', throttle_duration_sec=5.0)
                    time.sleep(0.5)
                    continue
            try:
                chunk = self.sock.recv(65536)
                if not chunk:
                    raise ConnectionError('closed')
                buf += chunk
            except Exception as e:
                self.get_logger().error(f'VTD 수신 오류({e}) → 재연결. 끊긴 동안 VTD는 steer=0/accel=0 처리')
                try:
                    self.sock.close()
                except Exception:
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
                self.pub_raw_rx.publish(UInt8MultiArray(data=list(pkt)))
                self.publish_state(unpack_data(pkt))
                self.rx_count += 1
            except Exception as e:
                self.get_logger().error(f'상태 발행 오류: {e!r}', throttle_duration_sec=2.0)

    def publish_state(self, st):
        now = self.get_clock().now()
        stamp = now.to_msg()
        t = now.nanoseconds * 1e-9
        z = 0.0 if self.ego_z_zero else st.z

        # 속도·각속도·가속 추정 (pose 차분 + 저역통과). 위치 점프 = 리스폰 → 리셋
        if self.prev is not None:
            dt = t - self.prev[0]
            dx, dy = st.x - self.prev[1], st.y - self.prev[2]
            jump = math.hypot(dx, dy)
            if jump > self.jump_reset_dist:
                self.get_logger().warning(f'위치 점프 {jump:.1f}m → 리스폰 판정, 추정기 리셋')
                self.vx_f = self.wz_f = self.ax_f = self.prev_vx = 0.0
                self.obj_hist.clear()
                self.flash = {'lanelet': None, 'stopped_since': None, 'go': False}
                self.pub_respawn.publish(Empty())
            elif 1e-4 < dt < 0.5:
                v = jump / dt
                if dx * math.cos(st.heading) + dy * math.sin(st.heading) < 0:
                    v = -v
                a = 0.35
                self.vx_f += a * (v - self.vx_f)
                self.wz_f += a * (wrap(st.heading - self.prev[3]) / dt - self.wz_f)
                self.ax_f += a * ((self.vx_f - self.prev_vx) / dt - self.ax_f)
                self.prev_vx = self.vx_f
        # 조향 보고 필터용 dt. 아래에서 self.prev 를 t 로 덮으므로 여기서 미리 뽑아 둔다.
        # (9/7 정적 감사: t - self.prev[0] 을 갱신 후에 계산해 항상 하한 1e-3 이 되고,
        #  실효 시정수가 0.2s 가 아니라 약 10s 가 되어 조향 보고가 명령을 못 따라갔다.)
        self.steer_dt = 0.05 if self.prev is None else min(0.2, max(1e-3, t - self.prev[0]))
        self.prev = (t, st.x, st.y, st.heading)
        self.last_pose = (st.x, st.y, z, st.heading)
        qx, qy, qz, qw = yaw_to_quat(st.heading)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        p = odom.pose.pose
        p.position.x, p.position.y, p.position.z = st.x, st.y, z
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        for i in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[i] = 0.01
        odom.twist.twist.linear.x = self.vx_f
        odom.twist.twist.angular.z = self.wz_f
        self.pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = st.x, st.y, z
        tf.transform.rotation = p.orientation
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

        # 합성 IMU: VTD가 IMU를 안 주므로 ego 운동 추정치(yaw rate wz_f, 종가속 ax_f)로 채운다.
        # AEB 등 IMU 필수 컴포넌트의 표준입력. 값은 실차 IMU와 같은 body(base_link) 프레임.
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'base_link'
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = qx, qy, qz, qw
        imu.angular_velocity.z = self.wz_f
        imu.linear_acceleration.x = self.ax_f
        imu.linear_acceleration.y = self.vx_f * self.wz_f   # 원심(횡) 가속
        for i in (0, 4, 8):
            imu.orientation_covariance[i] = 0.01
            imu.angular_velocity_covariance[i] = 0.01
            imu.linear_acceleration_covariance[i] = 0.04
        self.pub_imu.publish(imu)

        # 조향 보고: 실측값이 없어 명령값에 1차 지연을 씌운 근사 (개발계획_0902 §4 '임시 아님' 항목)
        with self.cmd_lock:
            target = self.cmd_steer
        dt_s = getattr(self, 'steer_dt', 0.05)
        self.steer_rep += (target - self.steer_rep) * min(1.0, dt_s / max(self.steer_tau, 1e-3))
        steer = SteeringReport()
        steer.stamp = stamp
        steer.steering_tire_angle = self.steer_rep
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
            turn.report = {0: TurnIndicatorsReport.DISABLE, 1: TurnIndicatorsReport.ENABLE_LEFT,
                           2: TurnIndicatorsReport.ENABLE_RIGHT}[self.cmd_turn]
        self.pub_turn_rep.publish(turn)

        hz = HazardLightsReport()
        hz.stamp = stamp
        hz.report = HazardLightsReport.DISABLE
        self.pub_hazard.publish(hz)

        if not self.initialized:
            self.initialized = True
            self.init_state_tick()
            self.get_logger().info('첫 패킷 수신 → localization INITIALIZED')

        self.publish_objects(st, stamp, t)
        self.publish_traffic_light(st, stamp, t)

    # ------------------------------------------------------------ objects
    def publish_objects(self, st, stamp, t):
        msg = PredictedObjects()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        seen = set()
        for (oid, x, y, z, heading, speed, length, width, height) in st.objects:
            oid = int(oid)
            seen.add(oid)
            if self.ego_z_zero:
                z = 0.0
            # id는 시나리오 내 유지(9/2 답변) → 이전 프레임으로 yaw rate 추정
            yaw_rate = 0.0
            h = self.obj_hist.get(oid)
            if h is not None and 1e-3 < t - h[0] < 1.0:
                yaw_rate = wrap(heading - h[3]) / (t - h[0])
                yaw_rate = max(-1.0, min(1.0, yaw_rate))
            self.obj_hist[oid] = (t, x, y, heading)

            obj = PredictedObject()
            obj.object_id = UUID(uuid=list(struct.pack('<IIII', oid & 0xFFFFFFFF, 0, 0, 0)))
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
            pp = k.initial_pose_with_covariance.pose
            pp.position.x, pp.position.y, pp.position.z = x, y, z
            pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = qx, qy, qz, qw
            k.initial_twist_with_covariance.twist.linear.x = speed
            k.initial_twist_with_covariance.twist.angular.z = yaw_rate
            path = PredictedPath()
            path.time_step.sec = 0
            path.time_step.nanosec = 500_000_000
            path.confidence = 1.0
            px, py, ph = x, y, heading
            n = int(self.predict_horizon / 0.5) + 1
            for i in range(n):
                q = Pose()
                q.position.x, q.position.y, q.position.z = px, py, z
                a, b, c, d = yaw_to_quat(ph)
                q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w = a, b, c, d
                path.path.append(q)
                px += speed * 0.5 * math.cos(ph)
                py += speed * 0.5 * math.sin(ph)
                ph += yaw_rate * 0.5
            k.predicted_paths.append(path)
            obj.kinematics = k
            obj.shape.type = Shape.BOUNDING_BOX
            obj.shape.dimensions.x = max(length, 0.3)
            obj.shape.dimensions.y = max(width, 0.3)
            obj.shape.dimensions.z = height if height > 0.1 else 1.6
            msg.objects.append(obj)
        for oid in [k for k in self.obj_hist if k not in seen]:
            if t - self.obj_hist[oid][0] > 2.0:
                del self.obj_hist[oid]
        self.pub_objects.publish(msg)

    # ------------------------------------------------------------ traffic light
    def on_route(self, msg: LaneletRoute):
        if self.tl_router is None:
            return
        ids = [seg.preferred_primitive.id for seg in msg.segments]
        threading.Thread(target=self._apply_route, args=(ids,), daemon=True).start()

    def _apply_route(self, ids):
        try:
            self.tl_router.set_route(ids)
            self.flash = {'lanelet': None, 'stopped_since': None, 'go': False}
        except Exception as e:
            self.get_logger().error(f'경로 신호등 구축 실패: {e!r}')

    def decide_color(self, state, entry, dist_to_stop, t):
        """VTD state → (색, 부가 element 목록). None이면 발행 안 함. 브리지가 가라/서라를 결정한다."""
        if state in (1,):
            return _TL.RED, []
        if state == 2:
            return _TL.AMBER, []
        if state in (3, 5):
            return _TL.GREEN, [(_TL.GREEN, _TL.LEFT_ARROW)] if state == 5 else []
        if state == 4:
            # GT state는 이미 Ego 진행방향 판정 결과(9/1 Q&A). Autoware는 lanelet turn_direction 없이는
            # 화살표를 정지로 해석하므로, 가라로 판단되면 GREEN CIRCLE을 함께 보낸다 (개발계획_0902 §4-4)
            return (_TL.GREEN if self.state4_go else _TL.RED), [(_TL.GREEN, _TL.LEFT_ARROW)]
        if state == 6:
            # 점멸: 규정 9 "정지선 2m 이내 0.5초 이상 일시정지 후 통과". Autoware는 FLASHING을 무시하므로
            # 브리지가 상태기계로 구현: RED 유지 → 정지선 근처에서 정지 flash_stop_time 지속 → GREEN (정지선 통과까지)
            f = self.flash
            if f['lanelet'] != entry.lanelet_id:
                f.update(lanelet=entry.lanelet_id, stopped_since=None, go=False)
            if f['go']:
                return _TL.GREEN, []
            stopped = abs(self.vx_f) < 0.1 and dist_to_stop is not None and dist_to_stop < self.flash_stop_dist
            if stopped:
                if f['stopped_since'] is None:
                    f['stopped_since'] = t
                elif t - f['stopped_since'] >= self.flash_stop_time:
                    f['go'] = True
                    self.get_logger().info(f'점멸 신호: {t - f["stopped_since"]:.1f}s 정지 확인 → 통과 허용 (lanelet {entry.lanelet_id})')
                    return _TL.GREEN, []
            else:
                f['stopped_since'] = None
            return _TL.RED, []
        return None, []

    def publish_traffic_light(self, st, stamp, t):
        if self.tl_router is None or not self.tl_router.has_route():
            return
        msg = TrafficLightGroupArray()
        msg.stamp = stamp
        loc = self.tl_router.locate(st.x, st.y)
        entry = None
        dist = None
        color = None
        if loc is not None:
            _, s_ego = loc
            entry = self.tl_router.next_stop(s_ego)
            if entry is not None:
                dist = entry.s_route - s_ego
                if st.tl_state != 6 and self.flash['lanelet'] is not None:
                    self.flash = {'lanelet': None, 'stopped_since': None, 'go': False}
                color, extra = self.decide_color(st.tl_state, entry, dist, t)
                if color is not None:
                    for gid in entry.groups:      # 같은 정지선의 규제요소 전부에 동일 상태
                        g = TrafficLightGroup()
                        g.traffic_light_group_id = gid
                        e = TrafficLightElement()
                        e.color, e.shape, e.status, e.confidence = color, _TL.CIRCLE, _TL.SOLID_ON, 1.0
                        g.elements.append(e)
                        for c, shp in extra:
                            e2 = TrafficLightElement()
                            e2.color, e2.shape, e2.status, e2.confidence = c, shp, _TL.SOLID_ON, 1.0
                            g.elements.append(e2)
                        msg.traffic_light_groups.append(g)
        self.pub_tl.publish(msg)
        prev = self.tl_last
        cur = {'state': st.tl_state, 'entry': entry.lanelet_id if entry else None, 'dist': dist,
               'color': color, 'groups': len(entry.groups) if entry else 0}
        if cur['state'] != prev['state'] or cur['entry'] != prev['entry'] or cur['color'] != prev['color']:
            self.get_logger().info(
                f"신호등: VTD state={cur['state']}({STATE_NAME.get(cur['state'], '?')}) → "
                f"lanelet {cur['entry']} 정지선 {dist if dist is None else round(dist, 1)}m, "
                f"규제요소 {cur['groups']}개에 {self._color_name(color)} 발행")
        self.tl_last = cur

    @staticmethod
    def _color_name(c):
        return {None: '없음', _TL.RED: 'RED', _TL.AMBER: 'AMBER', _TL.GREEN: 'GREEN'}.get(c, str(c))

    # ------------------------------------------------------------ dummy perception / init state
    def dummy_perception_tick(self):
        if self.last_pose is None:
            return
        stamp = self.get_clock().now().to_msg()
        pc = PointCloud2()
        pc.header.stamp = stamp
        pc.header.frame_id = 'base_link'
        pc.height, pc.width = 1, 0
        pc.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                     for i, n in enumerate(('x', 'y', 'z'))]
        pc.is_bigendian = False
        pc.point_step, pc.row_step = 12, 0
        pc.data = b''
        pc.is_dense = True
        self.pub_pc.publish(pc)

        x, y, _, _ = self.last_pose
        og = OccupancyGrid()
        og.header.stamp = stamp
        og.header.frame_id = 'map'
        og.info.map_load_time = stamp
        og.info.resolution = 0.5
        og.info.width = og.info.height = 200          # 100m × 100m, ego 중심, 전부 free(0)
        og.info.origin.position.x = x - 50.0
        og.info.origin.position.y = y - 50.0
        og.info.origin.orientation.w = 1.0
        og.data = [0] * (200 * 200)
        self.pub_ogm.publish(og)

    def init_state_tick(self):
        m = LocalizationInitializationState()
        m.stamp = self.get_clock().now().to_msg()
        m.state = (LocalizationInitializationState.INITIALIZED if self.initialized
                   else LocalizationInitializationState.UNINITIALIZED)
        self.pub_init.publish(m)

    # ------------------------------------------------------------ TX
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
            self.last_cmd_time = time.monotonic()

    def on_turn(self, msg: TurnIndicatorsCommand):
        with self.cmd_lock:
            self.cmd_turn = {TurnIndicatorsCommand.ENABLE_LEFT: 1,
                             TurnIndicatorsCommand.ENABLE_RIGHT: 2}.get(msg.command, 0)

    def tx_tick(self):
        sock = self.sock
        if sock is None or not self.connected:
            return
        now = time.monotonic()
        with self.cmd_lock:
            steer, accel, turn = self.cmd_steer, self.cmd_accel, self.cmd_turn
            last = self.last_cmd_time
        if last is None:
            steer, accel = 0.0, 0.0                     # 아직 명령 없음: 정지 유지
            stale = False
        else:
            stale = (now - last) > self.watchdog_timeout
            if stale:
                accel = self.failsafe_accel             # 워치독: 조향 유지, 감속
        if stale != self.watchdog_active:
            self.watchdog_active = stale
            if stale:
                self.get_logger().error(f'제어 명령 두절 {now - last:.2f}s → 페일세이프 감속 {self.failsafe_accel} m/s²')
            else:
                self.get_logger().info('제어 명령 재개')
        pkt = pack_ctrl(self.steer_sign * steer, accel, turn)
        self.pub_raw_tx.publish(UInt8MultiArray(data=list(pkt)))
        try:
            sock.sendall(pkt)
        except Exception as e:
            self.get_logger().warning(f'제어 송신 실패({e!r}) — rx_loop이 재연결', throttle_duration_sec=2.0)

    # ------------------------------------------------------------ 감시
    def report_tick(self):
        d = self.rx_count - self._rx_count_prev
        self._rx_count_prev = self.rx_count
        tl = ''
        if self.tl_router is not None:
            if not self.tl_router.has_route():
                tl = ', 신호등: 경로 없음'
            else:
                L = self.tl_last
                tl = (f", 신호등: state={L['state']} 다음정지선=lanelet {L['entry']} "
                      f"{'-' if L['dist'] is None else round(L['dist'], 1)}m 규제요소 {L['groups']}개 "
                      f"발행={self._color_name(L['color'])}")
        self.get_logger().info(
            f'rx {d}pkt/{self.get_parameter("report_period").value:.0f}s (총 {self.rx_count}), '
            f'ctrl_cmd 총 {self.ctrl_count}, 연결={self.connected}, 워치독={"작동" if self.watchdog_active else "정상"}, '
            f'v={self.vx_f:.2f}m/s{tl}')
        if self.connected and d == 0:
            self.get_logger().error('연결 상태인데 수신 0건 — VTD 정지 또는 rx 스레드 이상')
        if not self.rx_thread.is_alive():
            self.get_logger().fatal('rx 스레드 사망 — 노드 재시작 필요')


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

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
  원본 패킷     /vtd/raw_rx (1109B), /vtd/raw_tx (9B) — 필요 시 ros2 bag 으로 따로 기록

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
from dataclasses import fields

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
from autoware_internal_planning_msgs.msg import VelocityLimit, VelocityLimitClearCommand
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState

from .protocol import DATA_SIZE, unpack_data, pack_ctrl
from .osm_map import OsmMap
from .tl_router import TrafficLightRouter
from . import size_classifier

MAX_STEER_RAD = 0.48
ACCEL_MIN, ACCEL_MAX = -6.0, 3.0
_TL = TrafficLightElement

# VTD tl_state (대회정보.md §6): 0 미할당 / 1 적 / 2 황 / 3 녹 / 4 좌회전 / 5 녹+좌 / 6 점멸
STATE_NAME = {0: 'none', 1: 'red', 2: 'amber', 3: 'green', 4: 'left', 5: 'green+left', 6: 'flash'}

# size_classifier 분류 → Autoware 라벨
OBJECT_LABEL = {
    size_classifier.UNKNOWN: ObjectClassification.UNKNOWN,
    size_classifier.CAR: ObjectClassification.CAR,
    size_classifier.MOTORCYCLE: ObjectClassification.MOTORCYCLE,
    size_classifier.BICYCLE: ObjectClassification.BICYCLE,
    size_classifier.PEDESTRIAN: ObjectClassification.PEDESTRIAN,
}


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
        dp('jump_reset_dist', 10.0)       # [m] 한 프레임 이동이 이보다 크면 리스폰으로 판정
        dp('ego_z_mode', 'raw')          # 'raw': VTD z 그대로 (맵에 elevation 반영됨, 9/2) / 'zero': 맵이 z=0일 때
        dp('steer_report_tau', 0.2)      # [s] 조향 보고 1차 지연 (실측 조향 없음)
        dp('state4_go', True)            # state 4(적+좌회전 화살표)를 '가라'로 (개발계획_0902 §4-4)
        dp('flash_stop_time', 0.5)       # [s] state 6: 정지 유지 후 통과
        dp('flash_stop_dist', 8.0)       # [m] state 6: 정지선까지 이 거리 안에서 정지해야 인정
        # HL FMA 9/12: 신호 접근 속도 제한(황색 딜레마 제거). 녹색·미할당 신호로 다가갈 때, 언제 황색이
        #   켜져도 '설 수 있다' 또는 '황색 안에 정지선 통과' 중 하나가 되는 속도까지만 낮춘다.
        #   VTD 는 신호 주기를 안 주지만 황색은 전 신호 3.0 s 로 고정이다(로그 4,333회 중 95%).
        #   아래 네 값은 Autoware 신호등 모듈의 통과 판정과 같아야 한다(다르면 둘이 어긋난다):
        #     yellow_s  = traffic_light.param yellow_lamp_period
        #     stop_decel/stop_jerk/delay_s = behavior_velocity_planner_common max_accel/max_jerk/system_delay
        dp('tl_guard_enable', True)
        dp('tl_guard_yellow_s', 2.75)      # [s] 황색 3.0 s - 인지 지연·정지점~정지선 여유
        dp('tl_guard_stop_decel', 2.4)     # [m/s^2] VTD 가 정상 응답하는 감속 (JG 253496d7 실측)
        dp('tl_guard_stop_jerk', 5.0)      # [m/s^3]
        dp('tl_guard_delay_s', 0.5)        # [s]
        dp('tl_guard_stop_margin_m', 1.0)  # [m] = traffic_light.param stop_margin (범퍼가 정지선 1 m 전)
        dp('tl_guard_front_m', 3.808)      # [m] 후륜축(base_link) -> 앞범퍼
        dp('tl_guard_slow_decel', 1.5)     # [m/s^2] 제한 속도까지 줄이는 감속 (시작 거리 계산용)
        dp('tl_guard_margin_m', 3.0)       # [m] 시작 거리 여유
        # 9/12: VTD 가 신호를 정지선 12 m 앞에서야 알려주는 곳(짧은 도로의 신호 3곳, 예: lanelet 116111 → VTD 신호 167)이
        #   있다. 상태 미할당(0)인 정지선이 가까우면 12 m 안에 설 수 있는 속도로 접근한다(반응 0.5 s + 실제 감속 2.3 →
        #   5.5 m/s 정지거리 9.3 m). 0912_050738·054114 둘 다 34 km/h 로 적색 정지선 통과.
        dp('tl_unknown_cap_mps', 5.5)      # [m/s] 미할당 신호 정지선 접근 상한 (20 km/h)
        dp('tl_unknown_dist_m', 45.0)      # [m] 이 거리 안에 미할당 정지선이 있으면 상한 적용
        # 9/12: 교차로 서행. 시나리오의 대향·교차 차량은 자차가 정지선을 넘는 순간 출발해(정지선에 서 있으면
        #   영영 오지 않는다) 80 m 를 4 초에 달려온다(0 -> 70 km/h). 20 km/h 로 지나가면 정확히 그 시점에
        #   대향 차로 한가운데에서 만난다(0912_054114 t=85s, 0912_062716 t=73s 사각형 겹침 실측).
        #   정지선을 넘은 뒤부터만 12 km/h 로 제한한다(정지선 전 감속은 신호 정지 거동에 영향을 주므로 안 건다).
        #   12 km/h 면 대향차 도달(+4 s) 시점에 대향 차로 3.5 m 앞이고, 그전에 서면 그들이 지나간 뒤 간다.
        #   정지·통과 판단 자체는 Autoware 교차로/동적장애물 정지 모듈. 끄려면 isec_cap_mps 를 크게.
        dp('isec_cap_mps', 3.33)           # [m/s] 정지선 통과 후 상한 (12 km/h)
        dp('isec_trigger_m', 0.5)          # [m] 정지점까지 남은 거리가 이 값 이하면 '정지선 통과'로 본다
        dp('isec_after_m', 22.0)           # [m] 정지선을 지난 뒤 이 거리까지 유지
        # 9/12: 차로 방향과 어긋나게 달리는 차(끼어들기)는 차선 투영 대신 직선 예측. 투영하면 옆 차로에 머무는 것으로
        #   예측돼 정지 모듈이 못 본다(0912_054114 t=844s 옆 차로에서 들어와 서는 차와 24 km/h 접촉).
        dp('cutin_yaw_deg', 4.0)           # [deg] 차로 방향과 이 이상 어긋나면 직선 예측
        dp('publish_dummy_perception', True)
        dp('predict_horizon', 8.0)       # [s] objects 예측 경로 길이 (차선변경 2회 기동이 6~8s)
        dp('report_period', 5.0)
        # 객체 크기 분류 경계값 object_class.* — 기본값과 근거는 size_classifier.SizeThresholds
        for f in fields(size_classifier.SizeThresholds):
            dp('object_class.' + f.name, f.default)

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
        self.size_th = size_classifier.SizeThresholds(**{
            f.name: float(g('object_class.' + f.name)) for f in fields(size_classifier.SizeThresholds)})
        self.get_logger().info(f'객체 크기 분류 경계값: {self.size_th}')
        self.tl_guard_enable = bool(g('tl_guard_enable'))
        self.tl_guard_yellow = float(g('tl_guard_yellow_s'))
        self.tl_guard_stop_decel = float(g('tl_guard_stop_decel'))
        self.tl_guard_stop_jerk = float(g('tl_guard_stop_jerk'))
        self.tl_guard_delay = float(g('tl_guard_delay_s'))
        self.tl_guard_stop_margin = float(g('tl_guard_stop_margin_m'))
        self.tl_guard_front = float(g('tl_guard_front_m'))
        self.tl_guard_slow_decel = float(g('tl_guard_slow_decel'))
        self.tl_guard_margin = float(g('tl_guard_margin_m'))
        self.tl_unknown_cap = float(g('tl_unknown_cap_mps'))
        self.tl_unknown_dist = float(g('tl_unknown_dist_m'))
        self.cutin_yaw = math.radians(float(g('cutin_yaw_deg')))
        self.isec_cap = float(g('isec_cap_mps'))
        self.isec_trigger = float(g('isec_trigger_m'))
        self.isec_after = float(g('isec_after_m'))
        self.isec_hold = 0.0          # 정지선을 지난 뒤 남은 서행 거리 [m]
        self.isec_last_t = None       # 서행 거리 적분용
        self.tl_guard_vstar = self._tl_guard_safe_speed()
        self.tl_guard = {'active': False, 'lanelet': None, 'last_pub': -1e9}
        self.get_logger().info(
            f'신호 접근 제한: 딜레마 없는 속도 {self.tl_guard_vstar:.2f} m/s '
            f'({self.tl_guard_vstar * 3.6:.1f} km/h), enable={self.tl_guard_enable}')

        # 맵 + 신호등 라우터
        self.tl_router = None
        self.omap = None          # 객체 예측의 차선 투영에 재사용 (중복 로드 방지)
        osm = g('map_osm')
        if osm:
            t0 = time.time()
            try:
                self.omap = OsmMap(osm, self.get_logger())
                self.tl_router = TrafficLightRouter(self.omap, self.get_logger())
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
        # 신호 접근 제한: 다른 노드(blocked_route_detour, pedestrian_proximity_slowdown)와 같은 경로·QoS
        self.pub_vlim = mk(VelocityLimit, '/planning/scenario_planning/max_velocity_candidates', latched)
        self.pub_vlim_clear = mk(VelocityLimitClearCommand, '/planning/scenario_planning/clear_velocity_limit', latched)
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
        self.overspeed_guard_active = False

        # 상태 추정
        self.prev = None
        self.vx_f = self.wz_f = self.ax_f = self.prev_vx = 0.0
        self.max_plausible_speed = 30.0   # [m/s] 이보다 크면 추정 이상으로 보고 버린다
        # [m/s^2] 한 스텝의 속도 변화 한계. dt 가 패킷 '도착 시각' 기반이라 도착이 몰리면
        # (누락 뒤 2배 이동 ÷ 짧은 dt) 로 과대 속도가 나온다 — 실측(9/8): pose 가 약 4개마다
        # 하나씩 규칙적으로 누락되고 그때 55~60 km/h 스파이크가 0.2~0.6초 지속됐다.
        # 차량 한계는 common.param.yaml limit 의 max_acc 2.0 / min_acc -4.0 이므로 여유를 둬 6.0.
        self.max_accel_step = 6.0
        # 리스폰 리셋 직후 한 샘플은 위 제한을 건너뛴다. 리셋으로 추정이 0 이 됐는데 차는
        # 실제로 움직이는 경우, 제한이 걸리면 재획득이 0.3 m/s 씩만 되어 느려진다.
        self.vel_reacquire = True
        self.steer_rep = 0.0
        self.last_pose = None          # (x, y, z, yaw) 더미 인지·로그용
        self.obj_hist = {}             # id -> (t, x, y, heading, speed)
        self.obj_accel = {}            # id -> 평활된 가속도 [m/s^2] (VTD 미제공 → 차분 추정)
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
        self.create_timer(0.2, self.init_state_tick)
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
                self.vel_reacquire = True
                self.obj_hist.clear()
                self.obj_accel.clear()
                self.flash = {'lanelet': None, 'stopped_since': None, 'go': False}
                self.pub_respawn.publish(Empty())
            elif 0.005 < dt < 0.5:
                # dt 하한 1e-4 였을 때: 패킷 두 개가 거의 동시에 도착하면(네트워크 묶임)
                # 2cm 위치차가 200m/s 로 튀었다. 실측 194.65 m/s 스파이크가 이것.
                # VTD 20Hz 기준 정상 dt 는 0.05s 이므로 0.005s 미만은 갱신을 건너뛴다.
                v = jump / dt
                if dx * math.cos(st.heading) + dy * math.sin(st.heading) < 0:
                    v = -v
                if abs(v) > self.max_plausible_speed:
                    self.get_logger().warning(
                        f'속도 추정 이상 {v:.1f} m/s (dt={dt*1000:.1f}ms) → 무시')
                    v = self.vx_f
                # 물리적으로 가능한 가속으로 제한. 위 max_plausible_speed 는 극단값만 걸러
                # 60 km/h 대 스파이크는 그대로 통과했다(30 m/s = 108 km/h).
                step = self.max_accel_step * dt
                v_clamped = v if self.vel_reacquire \
                    else min(max(v, self.vx_f - step), self.vx_f + step)
                self.vel_reacquire = False
                if abs(v_clamped - v) > 0.5:
                    self.get_logger().warning(
                        f'속도 변화 제한 {v:.1f}→{v_clamped:.1f} m/s '
                        f'(dt={dt*1000:.1f}ms, 한계 {step:.2f} m/s)', throttle_duration_sec=2.0)
                v = v_clamped
                a = 0.35
                self.vx_f += a * (v - self.vx_f)
                self.wz_f += a * (wrap(st.heading - self.prev[3]) / dt - self.wz_f)
                self.ax_f += a * ((self.vx_f - self.prev_vx) / dt - self.ax_f)
                self.prev_vx = self.vx_f
        # dt 가 하한에 못 미치면 기준점을 옮기지 않는다. 그래야 그 구간 이동거리가
        # 다음 패킷으로 누적되어 보존된다(옮기면 그만큼 버려져 속도가 낮게 나온다).
        advance_prev = (self.prev is None) or not (0.0 < (t - self.prev[0]) <= 0.005)

        # 조향 보고 필터용 dt. 아래에서 self.prev 를 t 로 덮으므로 여기서 미리 뽑아 둔다.
        # (9/7 정적 감사: t - self.prev[0] 을 갱신 후에 계산해 항상 하한 1e-3 이 되고,
        #  실효 시정수가 0.2s 가 아니라 약 10s 가 되어 조향 보고가 명령을 못 따라갔다.)
        self.steer_dt = 0.05 if self.prev is None else min(0.2, max(1e-3, t - self.prev[0]))
        if advance_prev:
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
            # id는 시나리오 내 유지(9/2 답변) → 이전 프레임으로 yaw rate·가속도 추정.
            # VTD 패킷에는 가속도가 없다(객체당 x,y,z,heading,speed,l,w,h). 급정지 NPC 를
            # 등속으로 예측하면 실제보다 앞에 있다고 보므로 반드시 차분해서 반영한다.
            yaw_rate = 0.0
            accel = 0.0
            h = self.obj_hist.get(oid)
            if h is not None and 1e-3 < t - h[0] < 1.0:
                dt_h = t - h[0]
                yaw_rate = wrap(heading - h[3]) / dt_h
                yaw_rate = max(-1.0, min(1.0, yaw_rate))
                a_raw = (speed - h[4]) / dt_h
                a_raw = max(-9.0, min(5.0, a_raw))
                a_prev = self.obj_accel.get(oid, 0.0)
                accel = 0.5 * a_prev + 0.5 * a_raw      # 가벼운 평활 (GT 라 노이즈는 없지만 단발 튐 방지)
            self.obj_accel[oid] = accel
            self.obj_hist[oid] = (t, x, y, heading, speed)

            obj = PredictedObject()
            obj.object_id = UUID(uuid=list(struct.pack('<IIII', oid & 0xFFFFFFFF, 0, 0, 0)))
            obj.existence_probability = 1.0
            # 분류는 발행하는 shape 크기로 한다. 높이가 비어(≤0.1) 1.6 으로 채운 객체를
            # 낮은 정지물(UNKNOWN)로 떨어뜨리지 않기 위해서다.
            dim_x, dim_y = max(length, 0.3), max(width, 0.3)
            dim_z = height if height > 0.1 else 1.6
            cls = ObjectClassification()
            cls.label = OBJECT_LABEL[size_classifier.classify(dim_x, dim_y, dim_z, self.size_th)]
            cls.probability = 1.0
            obj.classification.append(cls)
            # 보행자·자전거(교통약자): 차선 투영 없이 직선 예측 (아래 주석)
            is_ped = cls.label in (ObjectClassification.PEDESTRIAN, ObjectClassification.BICYCLE)

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
            n = int(self.predict_horizon / 0.5) + 1

            # 차선 투영: 곡선에서 요레이트 외삽은 차선을 벗어난다. 자차 근처만 투영해 비용을 막는다.
            # 보행자·자전거는 투영하지 않고 진행 방향 그대로 직선 예측한다(9/12: 자전거 추가 — 도로변에서
            #   직각으로 튀어나오는 자전거를 차로 방향으로 투영하면 run_out 이 못 본다, 03:24 bag t=31s). 투영하면 차로를 가로지르는 보행자도
            #   옆 간격을 유지한 채 차로 방향으로 걷는 것으로 예측돼 run_out 이 자차 차로 진입을 미리 못 본다
            #   (9/12 NG 코드 재현: 횡단 각도 45~90° 모두 8초 동안 옆 간격 그대로). 요레이트도 쓰지 않는다 —
            #   8초 지평에서는 모퉁이를 도는 잠깐의 회전율이 제자리를 도는 예측이 된다.
            #   되돌리려면 아래 두 곳의 is_ped 조건을 뺀다.
            pred_yaw_rate = 0.0 if is_ped else yaw_rate
            lid = s_c = lat = None
            if self.omap is not None and not is_ped and math.hypot(x - st.x, y - st.y) < 120.0:
                try:
                    lid = self.omap.nearest_lanelet(x, y, heading)
                    if lid is not None:
                        _, s_c, _ = self.omap.project(lid, x, y)
                        cx, cy, ch, _ = self.omap.point_along(lid, s_c, 0.0)
                        lat = -(x - cx) * math.sin(ch) + (y - cy) * math.cos(ch)
                        dh = (heading - ch + math.pi) % (2.0 * math.pi) - math.pi
                        if abs(dh) > self.cutin_yaw and speed > 1.0:
                            lid = s_c = lat = None   # 끼어들기/차로 이탈 중 → 직선 예측 (dp 'cutin_yaw_deg' 주석)
                except Exception:
                    lid = s_c = lat = None

            px, py, ph, pv, travelled = x, y, heading, speed, 0.0
            for i in range(n):
                q = Pose()
                q.position.x, q.position.y, q.position.z = px, py, z
                a, b, c, d = yaw_to_quat(ph)
                q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w = a, b, c, d
                path.path.append(q)
                # 0.5초 등가속 전진. 감속으로 0 에 닿으면 거기서 멈춘다(후진 금지).
                if accel < 0.0 and pv + accel * 0.5 < 0.0:
                    t_stop = -pv / accel
                    ds = pv * t_stop + 0.5 * accel * t_stop * t_stop
                    pv = 0.0
                else:
                    ds = pv * 0.5 + 0.5 * accel * 0.25
                    pv = max(0.0, pv + accel * 0.5)
                travelled += ds
                if lid is not None:
                    nx, ny, nh, _ = self.omap.point_along(lid, s_c, travelled)
                    px = nx - lat * math.sin(nh)
                    py = ny + lat * math.cos(nh)
                    ph = nh
                else:
                    px += ds * math.cos(ph)
                    py += ds * math.sin(ph)
                    ph += pred_yaw_rate * 0.5
            k.predicted_paths.append(path)
            obj.kinematics = k
            obj.shape.type = Shape.BOUNDING_BOX
            obj.shape.dimensions.x = dim_x
            obj.shape.dimensions.y = dim_y
            obj.shape.dimensions.z = dim_z
            msg.objects.append(obj)
        for oid in [k for k in self.obj_hist if k not in seen]:
            if t - self.obj_hist[oid][0] > 2.0:
                del self.obj_hist[oid]
                self.obj_accel.pop(oid, None)
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
        self.tl_approach_guard(st.tl_state, entry, dist, t)

    # ------------------------------------------------------------ 신호 접근 속도 제한 (황색 딜레마)
    def _tl_judge_dist(self, v):
        """Autoware 신호등 통과 판정의 정지 필요거리(정지점 기준).
        behavior_velocity_planner_common calcJudgeLineDistWithJerkLimit 과 같은 식, 현재 가속 0 가정."""
        if v <= 0.0:
            return 0.0
        acc, jerk = -self.tl_guard_stop_decel, -self.tl_guard_stop_jerk
        x1 = v * self.tl_guard_delay
        v2 = v + acc * acc / (2.0 * jerk)
        if v2 <= 0.0:
            t2 = -(acc + math.sqrt(-2.0 * jerk * v)) / jerk
            return max(0.0, x1 + v * t2 + jerk * t2 ** 3 / 6.0)
        t2 = acc / jerk
        x2 = v * t2 + jerk * t2 ** 3 / 6.0
        x3 = -v2 * v2 / (2.0 * acc)
        return max(0.0, x1 + x2 + x3)

    def _tl_guard_safe_speed(self):
        """정지 필요거리 <= 황색 도달거리 가 되는 최고 속도 (이 속도 이하면 딜레마 구간이 없다)."""
        lo, hi = 0.1, 30.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if self._tl_judge_dist(mid) <= mid * self.tl_guard_yellow:
                lo = mid
            else:
                hi = mid
        return lo

    def tl_approach_guard(self, state, entry, dist, t):
        """녹색·미할당 신호로 다가갈 때 딜레마 구간에 들어가기 전에 속도를 v* 로 제한한다.
        v* 에서는 정지 필요거리 = 황색 도달거리라, 황색이 언제 켜져도 서거나 지나갈 수 있다.
        시작 거리 = v*·T + (v² - v*²)/(2·slow_decel): 이 거리에서 줄이기 시작하면 감속 도중에도
        '설 수 있다'가 유지된다. 한 번 걸면 정지점이 v* 의 황색 도달거리 안에 들어올 때까지 유지.
        적·황·점멸이면 Autoware 신호등 모듈/점멸 상태기계가 처리하므로 해제한다."""
        if not self.tl_guard_enable:
            return
        g = self.tl_guard
        vs, T = self.tl_guard_vstar, self.tl_guard_yellow
        may_turn_yellow = state in (0, 3, 5) or (state == 4 and self.state4_go)
        # 9/12: 적(1)·황(2)도 같은 제한. 47 km/h 에서 적색 정지선까지 -2 로 100 m 감속하면 제동 잔류(시정수 1.2 s)로
        #   정지선 몇 m 전에 서고 기어간다(0912_044131). 35 로 접근하면 감속 구간이 짧아 깔끔히 선다(사용자 관찰).
        #   제한은 최고속도일 뿐 정지·통과 판단은 그대로 Autoware 신호등 모듈. 되돌리려면 아래 줄 삭제
        may_turn_yellow = may_turn_yellow or state in (1, 2)
        s = None if dist is None else dist - self.tl_guard_front - self.tl_guard_stop_margin
        v = self.vx_f
        want, reason, limit = False, '', vs
        if entry is None or s is None:
            reason = '다음 정지선 없음'
        elif state == 0 and -1.0 < s <= self.tl_unknown_dist:
            # 9/12: 미할당 신호 정지선이 가까움 → 12 m 안에 설 수 있는 속도로 (dp 주석 참조)
            want, limit = True, self.tl_unknown_cap
        elif not may_turn_yellow:
            reason = f'state {state}'
        elif s <= vs * T:
            reason = f'정지점 {s:.1f}m 가 제한속도의 황색 도달거리 안'
        elif g['active'] and g['lanelet'] == entry.lanelet_id:
            want = True
        elif v > vs + 0.2 and s > v * T:
            start = vs * T + (v * v - vs * vs) / (2.0 * self.tl_guard_slow_decel) + self.tl_guard_margin
            want = s <= start
        # ── 교차로 서행: 정지선을 넘은 뒤 isec_after 만큼 (위 dp('isec_cap_mps') 주석 참조) ────
        dt = 0.0 if self.isec_last_t is None else max(0.0, min(0.5, t - self.isec_last_t))
        self.isec_last_t = t
        if s is not None and s <= self.isec_trigger:
            self.isec_hold = self.isec_after          # 정지선 통과 순간부터 재충전
        elif self.isec_hold > 0.0:
            self.isec_hold = max(0.0, self.isec_hold - max(0.0, self.vx_f) * dt)
        if self.isec_hold > 0.0:
            if not want:
                want, reason = True, ''
            limit = min(limit, self.isec_cap)

        if want:
            if not g['active'] or g['lanelet'] != entry.lanelet_id:
                self.get_logger().info(
                    f'HLFMA 신호 접근 제한: lanelet {entry.lanelet_id} 정지점 {s:.1f}m state {state}, '
                    f'{v * 3.6:.0f} -> {limit * 3.6:.0f} km/h (48km/h 기준 딜레마 '
                    f'{13.33 * T:.0f}~{self._tl_judge_dist(13.33):.0f}m)')
            g['active'], g['lanelet'] = True, entry.lanelet_id
            if t - g['last_pub'] >= 0.2:
                msg = VelocityLimit()
                msg.stamp = self.get_clock().now().to_msg()
                msg.max_velocity = float(limit)
                msg.sender = 'tl_dilemma_guard'
                self.pub_vlim.publish(msg)
                g['last_pub'] = t
        elif g['active']:
            clr = VelocityLimitClearCommand()
            clr.stamp = self.get_clock().now().to_msg()
            clr.sender = 'tl_dilemma_guard'
            clr.command = True
            self.pub_vlim_clear.publish(clr)
            self.get_logger().info(f"HLFMA 신호 접근 제한 해제: lanelet {g['lanelet']} ({reason})")
            g['active'], g['lanelet'] = False, None

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

        # Final command guard for the 50 km/h scoring limit.  Preserve any stronger
        # braking request (including AEB) by only lowering the acceleration command.
        speed_kph = max(0.0, self.vx_f) * 3.6
        if speed_kph >= 49.0:
            accel = min(accel, -2.5)
        elif speed_kph >= 48.0:
            # 48 km/h: -1.0 m/s², linearly reaching -2.5 m/s² at 49 km/h.
            guard_accel = -1.0 - 1.5 * (speed_kph - 48.0)
            accel = min(accel, guard_accel)
        elif speed_kph >= 47.5:
            accel = min(accel, 0.0)

        guard_active = speed_kph >= 47.5
        if guard_active != self.overspeed_guard_active:
            self.overspeed_guard_active = guard_active
            if guard_active:
                self.get_logger().warning(
                    f'overspeed guard 활성화: {speed_kph:.2f} km/h, accel={accel:.2f} m/s²')
            else:
                self.get_logger().info(f'overspeed guard 해제: {speed_kph:.2f} km/h')
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

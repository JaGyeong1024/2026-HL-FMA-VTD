"""차선 단위 회피 판단 로직 (순수 모듈, ROS 의존 없음).

상태기계 FOLLOW / ASSESS / HOLD / LANE_CHANGE / STOP.
ROS 노드(blocked_route_detour.py)는 토픽·서비스를 이 모듈의 Inputs/Decision 으로 변환하는 얇은 래퍼여야 한다.

★ 인터페이스 스텁 — 구현 전. 테스트(test/test_detour_logic.py)가 계약을 정의한다.
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

FOLLOW, ASSESS, HOLD, LANE_CHANGE, STOP = 'FOLLOW', 'ASSESS', 'HOLD', 'LANE_CHANGE', 'STOP'
LEFT, RIGHT = 'left', 'right'
VEHICLE_CLASSES = ('CAR', 'TRUCK', 'BUS', 'TRAILER')


@dataclass
class Params:
    hold_distance_m: float = 45.0            # blocker 앞 유지 거리 (차선변경 길이 확보)
    approach_speed_mps: float = 4.0          # HOLD 속도
    approach_deceleration_mps2: float = 1.5  # hold 접근 감속
    blocker_stop_speed_mps: float = 0.3      # 이하면 정지 객체
    blocker_min_stopped_s: float = 3.0       # 이 시간 이상 정지해야 blocker
    detection_distance_m: float = 80.0
    path_lateral_margin_m: float = 2.2       # 경로 횡거리 이내면 자차 차선 위
    regulatory_clearance_m: float = 26.6     # max_prepare_duration × max_vel (LC 개시 금지 구역)
    stopline_min_distance_m: float = 28.0    # 정지차~정지선: 우회+복귀 가능 최소
    abort_backoff_s: float = 10.0
    max_abort_count: int = 2
    ped_lookahead_m: float = 50.0
    ped_lateral_margin_m: float = 2.5        # 차선 경계 바깥 여유
    ped_speed_limit_mps: float = 30.0 / 3.6
    ped_release_delay_s: float = 1.0
    respawn_hold_s: float = 1.5
    input_timeout_s: float = 1.0
    sender_hold: str = 'detour_hold'
    sender_ped: str = 'detour_pedestrian'


@dataclass
class EgoState:
    t: float
    v: float
    lane_role: str = 'preferred'    # 'preferred' | 'right_of_preferred' | 'left_of_preferred'
    lane_half_width_m: float = 1.75


@dataclass
class ObjectInfo:
    id: int
    cls: str                         # 'CAR','TRUCK','BUS','TRAILER','MOTORCYCLE','BICYCLE','PEDESTRIAN','UNKNOWN'
    v: float
    longitudinal_m: float            # 자차 기준 경로 진행방향 거리 (+앞)
    lateral_m: float                 # 경로 중심선 기준 횡거리 (+좌)
    stopped_since: Optional[float]   # 정지 시작 시각(t), 움직이면 None
    lateral_speed_toward_path_mps: float = 0.0  # 경로 쪽으로 접근하는 횡속도 (+접근)


@dataclass
class LaneInfo:
    neighbor_left: bool
    neighbor_right: bool
    left_boundary_solid: bool = False
    right_boundary_solid: bool = False
    distance_to_regulatory_m: Optional[float] = None  # 다음 신호등·회전 lanelet 까지 (없으면 None)
    distance_to_stopline_m: Optional[float] = None    # 다음 정지선까지 (없으면 None)


@dataclass
class SignalInfo:
    red_stop_factor_distance_m: Optional[float]  # traffic_light planning_factor STOP 의 control point 거리 (없으면 None)
    available: bool = True                       # 브리지 신호 정보 존재 여부


@dataclass
class CandidateInfo:
    present: bool = False
    safe: bool = False
    state: Optional[str] = None      # 'WAITING' | 'RUNNING' | 'ABORTING' | 'SUCCEEDED' | 'FAILED'
    start_distance_m: float = 0.0
    finish_distance_m: float = 0.0
    stale: bool = True


@dataclass
class Inputs:
    t: float
    ego: EgoState
    objects: List[ObjectInfo]
    lane: LaneInfo
    signal: SignalInfo
    candidates: Dict[str, CandidateInfo] = field(default_factory=dict)  # 'left'/'right'
    respawn: bool = False
    stale: bool = False              # odom/objects/path 중 하나라도 input_timeout 초과


@dataclass
class Decision:
    state: str
    approve: Optional[str] = None                     # 'left' | 'right' | None
    limits: List[Tuple[str, float]] = field(default_factory=list)   # [(sender, v_mps)]
    clears: List[str] = field(default_factory=list)   # 해제할 sender
    reason: str = ''


class DetourLogic:
    """호출자는 tick 마다 step(inputs) 를 부르고 Decision 을 그대로 RTC/속도제한 채널로 옮긴다."""

    def __init__(self, params: Optional[Params] = None):
        self.p = params or Params()

    def step(self, inputs: Inputs) -> Decision:
        raise NotImplementedError('detour_logic.DetourLogic.step — 구현 전 (하네스 red)')

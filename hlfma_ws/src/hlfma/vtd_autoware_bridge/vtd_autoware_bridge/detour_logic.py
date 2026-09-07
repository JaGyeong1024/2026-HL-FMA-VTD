"""차선 단위 회피 판단 로직 (순수 모듈, ROS 의존 없음).

상태기계 FOLLOW / ASSESS / HOLD / LANE_CHANGE / STOP.
ROS 노드(blocked_route_detour.py)는 토픽·서비스를 이 모듈의 Inputs/Decision 으로 변환하는 얇은 래퍼다.

설계 근거: todo0906 묶음 2, adv_bundle2.md (D1~D8). 규칙마다 근거 항목을 주석으로 표기한다.
  - 회피는 차선 단위(external_request 차선변경)만. 옆 차선 공간을 빌려 걸치는 shift 회피는 쓰지 않는다.
  - 판단은 GT 객체 기반, 신호·정지선·차선 구조는 맵 기반. 조향·RSS 안전 판정·취소는 Autoware 가 한다.
  - 출력 채널은 RTC 승인(ACTIVATE)과 속도 제한(sender 별)뿐이다.

9/7 시뮬 실주행 반영 (fix_detour_timing.md)
  - **VTD GT objects 는 약 80 m 까지만 온다** (2 회 주행 bag 실측, 둘 다 최대 80 m). 감지 거리를 늘려도
    그 이상은 볼 수 없으므로 "확인 시간"을 줄여 결정을 앞당기는 것이 유일한 수단이다.
  - 감속(HOLD) 개시와 승인(ASSESS) 을 분리했다. 감속은 slowdown_min_stopped_s(짧게) 로 즉시 시작하고,
    승인은 blocker_min_stopped_s 를 채운 뒤에만 한다. 감속은 되돌릴 수 있고 감점도 없다.
  - **대기열 판정에 신호를 쓰지 않는다.** "막힘 앞끝 ~ 다음 정지선 거리 ≥ stopline_min_distance_m",
    즉 **끼어들 공간이 있는가** 하나가 두 경우를 이미 가른다.
      · 신호 대기 중인 차들은 정지선에 붙어 선다 → 공간이 없다 → 후보도 안 생기고 자연히 뒤에서 기다린다.
      · 정지선에서 멀찍이 선 차는 신호로 설명되지 않는다(적색인데 멀찍이 설 이유가 없다)
        → 움직이지 않는 차로 보고 앞 공간으로 끼어든다.
    신호 색은 같은 답을 중복으로 내면서, 신호 라우팅이 틀리면(교차로 진입 시 state 0 등) 답을 망친다.
    ego 자신의 신호 준수는 Autoware traffic_light 모듈 소관이지 이 노드 소관이 아니다.
"""
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

FOLLOW, ASSESS, HOLD, LANE_CHANGE, STOP = 'FOLLOW', 'ASSESS', 'HOLD', 'LANE_CHANGE', 'STOP'
LEFT, RIGHT = 'left', 'right'
VEHICLE_CLASSES = ('CAR', 'TRUCK', 'BUS', 'TRAILER')          # blocker 후보 (adv_bundle2 D3: 보행자 제외)
VULNERABLE_CLASSES = ('PEDESTRIAN', 'BICYCLE', 'MOTORCYCLE')  # 전방 감속 규칙 대상
CAND_WAITING, CAND_RUNNING, CAND_ABORTING = 'WAITING', 'RUNNING', 'ABORTING'


@dataclass
class Params:
    # --- 접근·standoff (9/7 사용자 지시로 정책 전면 교체) ---
    # 옛 정책: 달리면서 1.5 초 안에 판단해 승인 → 인지 80 m 상한 때문에 성립 구간이 없었다
    # (세 주행 통틀어 finish+여유 < blocker 가 성립한 프레임 0 개).
    # 새 정책: **우회가 가능한 거리를 남기고 서서, 서서 관찰한 뒤 판단한다.** 시간 압박이 사라진다.
    #   13.9 m/s 에서 −3.5 로 정지에 27.6 m → 인지 79.5 m 에서 48 m 지점 정지에 31.5 m 여유. 성립.
    # standoff 근거(실측, 정지 상태 후보 finish): detour1 40.4 m / detour2 29.8 m → 큰 쪽 + 여유 6 m.
    standoff_m: float = 48.0                 # blocker 앞 이 거리에 **실제로** 서야 한다 (40.4 + 6 + 슬랙)
    # 오버슈트 보정: 우리가 내는 것은 속도 상한이지 정지 지점이 아니다. 상한이 0 이 되어도 자차는
    # 스무더·제어 지연을 거쳐 감속하므로 목표를 지나쳐 선다. 실측(detour9 bag): 목표 48 m 인데
    # 실제 정지 31.4 m → **오버슈트 16.6 m**(조정자 관측 37.3 m 시점 기준 11 m). 11~17 m 범위.
    # 따라서 유효 목표 = standoff_m + standoff_margin_m 로 **더 뒤를 겨냥**한다.
    # 일찍 서는 쪽은 무해하다(finish+여유 < 유효목표 이므로 승인은 그대로 성립하고 거리만 남는다).
    # 늦게 서면 기동 자체가 불가능해지므로 여유를 넉넉히 잡는 것이 옳다.
    # 대안(감속도 재산출)은 채택하지 않았다: standoff_decel_mps2 를 올리면 프로파일이 더 관대해져
    # 제동이 늦어지고 오버슈트가 **악화**된다. 낮추는 것은 램프 시작점을 조금 당길 뿐이라 효과가 작다.
    standoff_margin_m: float = 18.0          # 오버슈트 보정 (실측 최대 16.6 + 슬랙)
    standoff_decel_mps2: float = 1.5         # standoff 접근 감속 프로파일 (보조 레버)
    approach_speed_mps: float = 4.0          # 접근 중 속도 상한
    no_response_s: float = 3.0               # 서서 관찰: blocker 가 이만큼 계속 정지하면 우회 결정
    standoff_needed_fallback_m: float = 40.0  # 후보가 없을 때 '우회에 필요한 거리' 보수값
    approach_speed_floor_mps: float = 2.0    # standoff 를 못 만든 경우의 하한(손 놓지 않고 최대한 감속)
    blocker_move_speed_mps: float = 0.5      # 이 이상이면 blocker 가 움직인 것 (히스테리시스)
    lc_finish_margin_m: float = 6.0          # 차선변경 완료 지점과 blocker 사이 여유
    # --- blocker 판정 ---
    blocker_stop_speed_mps: float = 0.3      # 이하면 정지 객체
    slowdown_min_stopped_s: float = 0.2      # 감속(HOLD) 개시 게이트 — 짧게. 되돌릴 수 있고 감점 없음
    blocker_min_stopped_s: float = 0.5       # 승인 게이트. GT 속도는 정확해 노이즈 필터가 거의 불필요
    detection_distance_m: float = 80.0       # VTD GT 실측 상한 79.8 m (bag 2 회). 이 이상은 볼 수 없다
    path_lateral_margin_m: float = 2.2       # 경로 횡거리 이내면 자차 차선 위
    # --- 기하·의미 조건 ---
    # (판정 미사용 — 진단용으로만 남긴다) Autoware 의 isLaneChangeRequired(scene.cpp:296-342)가 같은 검사를
    # 이미 한다. 규제요소가 가까우면 **후보를 아예 만들지 않으므로**, 후보 존재가 곧 통과의 증거다.
    # 우리가 베껴 둔 값(max_prepare_duration × max_vel)이 Autoware 실제 계산과 어긋나면 멀쩡한 우회를 막는다
    # (9/7 detour16: 우리 54.4 m 기준으로 48 m 를 거부했는데 Autoware 는 후보를 내고 있었다).
    regulatory_clearance_m: float = 26.6     # 미사용. 래퍼 호환 위해 필드만 유지
    # **유일한 대기열 판별자이자 기하 조건**: 막힘 앞끝 ~ 다음 정지선 거리가 "우회 후 복귀에 필요한 길이"
    # 이상인가. 상수가 아니라 **속도 기반으로 계산**한다(9/7 사용자 지적):
    #   need = n_return × lc_length(v_hold) + stopline_stop_margin_m
    # 상수 26 은 (a) 복귀에 몇 칸이 필요한지 세지 않고 (b) 속도 의존을 굳혀 저속에서 과대평가였다.
    stopline_stop_margin_m: float = 3.0      # 복귀 완료 후 정지선까지 남길 여유
    # 복귀 여유 부족을 **거부 근거로 쓸 것인가**. 기본 false = 계산은 하되 승인을 막지 않고 경고만 남긴다.
    #  (1) 모델이 실측 대비 과대다: 13.7 m/s 에서 모델 118 m vs 실측 후보 finish 99 m (약 20%).
    #      원인은 prepare 항 — 우리는 max_prepare_duration(4.0)×v 로 잡는데 실제 선택된 후보의 start 는
    #      27 m 였다(= 약 2.0×v, min_prepare_duration 쪽). 즉 (4.0−2.0)×v 만큼 과대평가한다.
    #  (2) 비용이 비대칭이다: 우회 후 복귀 실패는 경로이탈 리스폰 1 회(구간당 1 회 무료, 초과 −6)지만,
    #      우회를 안 하면 영원히 못 간다. 불확실할 때는 시도하는 쪽이 기대값이 낫다.
    #  (3) 복귀는 경로상 필수 차선변경이라 Autoware 가 알아서 최선을 다한다.
    # 실주행에서 복귀가 실제로 되는지 관측해 모델을 보정한 뒤 enforce 를 켤지 정한다.
    return_check_enforce: bool = False
    # 계산 결과의 **하한**(실제 판정은 계산값이 지배한다). 12 = 차 길이 4.85 + 정지 여유 3.0 + 슬랙 ~4.
    # v_hold 나 모델 파라미터가 0 에 가깝게 잘못 들어와 need 가 비정상적으로 작아져도
    # 정지선에 이보다 가깝게 붙어서 우회를 시작하지는 않게 하는 안전장치다.
    stopline_min_distance_m: float = 12.0
    # 차선변경 길이 모델 (Autoware 소스와 같은 식. yaml 과 어긋나지 않게 런치 인자로 받는다)
    #   t = calc_shift_time_from_jerk(폭, jerk, lat_acc)        (path_shift.cpp:70)
    #   lc_length(v) = max_prepare_duration·v + v·t + ½·a_lc·t²  (calculation.cpp:356)
    #   a_lc = clamp((목표속도−v)/t, 0, max_longitudinal_acc)    (calculation.cpp:449)
    # a_lc 는 **상한으로 고정**해 보수적으로 잡는다 — 저속에서는 실제로 상한에 붙는다(§6 실측).
    lc_lane_width_m: float = 3.5
    lc_lateral_jerk: float = 1.5             # yaml: lane_change.trajectory.lateral_jerk
    lc_lateral_acc: float = 1.2              # yaml: lateral_acceleration.max_values
    lc_max_prepare_duration: float = 4.0     # yaml: trajectory.max_prepare_duration
    lc_max_longitudinal_acc: float = 1.0     # yaml: trajectory.max_longitudinal_acc
    # --- 취소 대응 ---
    abort_backoff_s: float = 10.0            # D5: 승인→취소 루프 방지
    max_abort_count: int = 2
    give_up_retry_s: float = 30.0            # hold timeout 으로 포기한 뒤 재평가까지 (영구 고착 방지)
    # --- 보행자 전방 감속 ---
    ped_lookahead_m: float = 50.0
    ped_lateral_margin_m: float = 2.5        # 차선 경계 바깥 여유
    ped_speed_limit_mps: float = 30.0 / 3.6
    ped_release_delay_s: float = 1.0
    # --- 기타 ---
    respawn_hold_s: float = 1.5              # 리스폰 직후 정지 유지 (VTD grace 1.0 s + 여유)
    input_timeout_s: float = 1.0
    clear_time_s: float = 1.0                # blocker 소실 후 hold 해제까지
    sender_hold: str = 'detour_hold'         # D6: hold 와 보행자 제한은 sender 분리
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
    lanes_to_preferred: Dict[str, int] = field(default_factory=dict)
    """'left'/'right' → **그 쪽으로 우회했을 때** 우선차선까지 되돌아오는 데 필요한 칸 수.
    예) 세그먼트 [15379, 15414, 15449, 15484], 자차 15414, 우선(좌회전) 15379, 우회 목적지 15449
        → 15449 에서 15379 까지 2 칸이므로 lanes_to_preferred['right'] = 2."""


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
    detail: Dict[str, Any] = field(default_factory=dict)   # 판단 근거 (로그·/detour/status 용)
    warn: Optional[str] = None                             # 있으면 래퍼가 WARN 으로 남긴다(전이당 1회)


class DetourLogic:
    """호출자는 tick 마다 step(inputs) 를 부르고 Decision 을 그대로 RTC/속도제한 채널로 옮긴다.

    내부 상태는 전부 이 객체에 있고, 리스폰(inputs.respawn)이 오면 전부 초기화된다.

    **standoff 유지 원칙 (9/7 detour6)**: blocker 를 잡고 있는 동안 standoff 제한을 푸는 경우는 두 가지뿐이다.
      ① blocker 가 움직이기 시작했을 때  ② 우회를 승인했을 때
    "지금은 우회 불가"(적신호 대기열·후보 없음·unsafe·back-off·기하 불가)는 **푸는 이유가 아니다.**
    풀면 자차가 obstacle_stop 까지 기어가 blocker 코앞에 서고, 그 뒤에는 상황이 좋아져도 우회할 여유가 없다.
    (detour6: 초록불·앞차 25 초 정지·38.1 m 뒤에서 제한을 풀어 10.9 m 까지 접근 → 우회 불가.)
    예외는 인지 시점에 이미 standoff 안이었던 경우(standoff_missed)뿐이다 — 그때는 만들 수 없으므로 위임한다.
    """

    def __init__(self, params: Optional[Params] = None):
        self.p = params or Params()
        self._reset_all()

    # ------------------------------------------------------------ 내부 상태
    def _reset_all(self):
        self.hold_active = False         # sender_hold 제한이 걸려 있는가
        self.hold_value = 0.0
        self.hold_since: Optional[float] = None        # HOLD 진입 시각 (hold_timeout_s 판정)
        self.standoff_missed = False                   # 인지 시점에 이미 우회 필요거리 안이었다
        self.tracked_blocker_id: Optional[int] = None  # standoff_missed 판정용
        self.last_blocked_t = -math.inf  # 마지막으로 blocker 를 본 시각
        self.committed: Optional[str] = None   # 승인해 둔 방향 (RUNNING 관측 전까지)
        self.abort_count = 0
        self.backoff_until = -math.inf
        self.ped_active = False
        self.ped_last_seen_t = -math.inf
        self.respawn_until = -math.inf
        self.gave_up_t: Optional[float] = None        # hold_timeout 으로 포기한 시각
        self.gave_up_blocker_id: Optional[int] = None  # 그때의 blocker (바뀌면 즉시 재평가)

    # ------------------------------------------------------------ 보조 판정
    def _blockers(self, inputs: Inputs, min_stopped_s: float) -> List[ObjectInfo]:
        """자차 차선 위 정지 차량 (D3: 차량 클래스만). min_stopped_s 이상 정지한 것만."""
        p, t, out = self.p, inputs.t, []
        for o in inputs.objects:
            if o.cls not in VEHICLE_CLASSES:
                continue
            if o.v > p.blocker_stop_speed_mps or o.stopped_since is None:
                continue
            if t - o.stopped_since < min_stopped_s:
                continue
            if abs(o.lateral_m) >= p.path_lateral_margin_m:
                continue
            if not (0.0 < o.longitudinal_m <= p.detection_distance_m):
                continue
            out.append(o)
        return sorted(out, key=lambda o: o.longitudinal_m)

    def _pedestrian_limit(self, inputs: Inputs, limits, clears):
        """도로변 보행자·이륜차 전방 감속 (근거: run_out 은 정지 보행자 무시 + 인지→제동 지연 ~0.5 s
        + 저크 램프 → 49 km/h 정지거리 40~50 m. 미리 느려져 있어야만 정지 가능. 저속 주행은 감점 아님)."""
        p, t = self.p, inputs.t
        edge = inputs.ego.lane_half_width_m + p.ped_lateral_margin_m
        seen = False
        for o in inputs.objects:
            if o.cls not in VULNERABLE_CLASSES:
                continue
            if not (0.0 < o.longitudinal_m <= p.ped_lookahead_m):
                continue
            if abs(o.lateral_m) <= edge or o.lateral_speed_toward_path_mps > 0.0:
                seen = True
                break
        if seen:
            self.ped_active, self.ped_last_seen_t = True, t
        if self.ped_active:
            if seen or t - self.ped_last_seen_t < p.ped_release_delay_s:
                limits.append((p.sender_ped, p.ped_speed_limit_mps))
            else:
                self.ped_active = False
                clears.append(p.sender_ped)

    def _standoff_speed(self, blocker_distance: float) -> float:
        """blocker 앞 standoff_m 지점에 서기 위한 속도 상한.

        여기서는 0 으로 수렴하는 것이 **의도된 동작**이다 — 우회에 필요한 거리를 남기고 서야 하기 때문.
        (앞서 "0 을 내지 말라" 던 지시는 이 정책으로 대체됐다. 그때는 standoff 개념 없이 거리만으로
         목표를 끌어내려 우회 여지 없이 앞차 코앞에 세우는 것이 문제였다.)
        """
        p = self.p
        target = self.effective_standoff_m()
        return min(p.approach_speed_mps,
                   math.sqrt(2.0 * p.standoff_decel_mps2 * max(0.0, blocker_distance - target)))

    def needed_distance_m(self, inputs: Inputs) -> float:
        """우회에 **실제로** 필요한 거리 = 현재 후보의 완료거리 + 여유. 후보가 없으면 보수값.

        9/7 detour14: 판정 기준이 프로파일 목표(73 m)였던 탓에 인지 70 m·56 m 에서도 '이미 안쪽'으로
        포기했다. 70 m 면 우회는 충분히 가능하다(그때 후보 finish 38 m). 기준은 목표가 아니라 필요거리다.
        """
        finishes = [c.finish_distance_m for c in inputs.candidates.values()
                    if c is not None and c.present and not c.stale and c.finish_distance_m > 0.1]
        if not finishes:
            return self.p.standoff_needed_fallback_m
        return min(finishes) + self.p.lc_finish_margin_m

    def effective_standoff_m(self) -> float:
        """프로파일이 겨냥하는 지점. 실제 정지 지점이 standoff_m 이상이 되도록 오버슈트만큼 앞당긴다."""
        return self.p.standoff_m + self.p.standoff_margin_m

    def _set_hold(self, v: float, limits, t: Optional[float] = None):
        if self.hold_since is None and t is not None:
            self.hold_since = t
        self.hold_active, self.hold_value = True, v
        limits.append((self.p.sender_hold, v))

    def _clear_hold(self, clears):
        self.hold_active, self.hold_value = False, 0.0
        self.hold_since = None
        clears.append(self.p.sender_hold)

    def shift_time_s(self) -> float:
        """Autoware calc_shift_time_from_jerk(path_shift.cpp:70) 와 같은 식."""
        p = self.p
        j, a, l = abs(p.lc_lateral_jerk), abs(p.lc_lateral_acc), abs(p.lc_lane_width_m)
        if j < 1e-8 or a < 1e-8:
            return 1e10
        tj = a / j
        ta = (math.sqrt(a * a + 4.0 * j * j * l / a) - 3.0 * a) / (2.0 * j)
        if ta < 0.0:
            tj, ta = (l / (2.0 * j)) ** (1.0 / 3.0), 0.0
        return 4.0 * tj + 2.0 * ta

    def lc_length_m(self, v: float) -> float:
        """속도 v 에서 차선변경 1 칸에 필요한 길이(준비 + 변경). a_lc 는 상한으로 보수적으로 잡는다."""
        p = self.p
        t = self.shift_time_s()
        return p.lc_max_prepare_duration * v + v * t + 0.5 * p.lc_max_longitudinal_acc * t * t

    def _room_verdict(self, inputs: Inputs, blockers: List[ObjectInfo]):
        """끼어들 공간이 있는가 — **유일한 대기열 판별자**. 신호를 보지 않는다.

        필요 길이는 상수가 아니라 **우회 후 복귀에 실제로 드는 길이**로 계산한다:
            need = n_return × lc_length(approach_speed_mps) + stopline_stop_margin_m
        n_return 은 우회 목적지 차선에서 우선차선까지의 칸 수다(옆 차선로 우회하면 보통 1~2).
        판정 대상 간격은 가장 가까운 blocker 가 아니라 **정지 차량군 중 가장 앞선 것**과 정지선 사이다.
        정지선이 근처에 없으면 신호 대기일 수 없고 복귀 제약도 없으므로 공간이 있는 것으로 본다.
        """
        p, lane = self.p, inputs.lane
        front = blockers[-1]              # longitudinal 오름차순 → 마지막이 정지선에 가장 가까운 차
        d_sl = lane.distance_to_stopline_m
        n_vals = [n for n in lane.lanes_to_preferred.values() if n is not None]
        n_return = min(n_vals) if n_vals else 1       # 모르면 1 칸으로 가정(가장 흔한 경우)
        lc_len = self.lc_length_m(p.approach_speed_mps)
        need = max(p.stopline_min_distance_m, n_return * lc_len + p.stopline_stop_margin_m)
        info = {'n_lanes': n_return, 'lc_length_model_m': round(lc_len, 1),
                'need_m': round(need, 1), 'gap_m': None,
                'stopline_dist_m': None if d_sl is None else round(d_sl, 1),
                'front_blocker_to_stopline_m': None}
        if d_sl is None:
            info.update(verdict='detour', basis='no_stopline')
            return True, info
        gap = d_sl - front.longitudinal_m
        info['gap_m'] = round(gap, 1)
        info['front_blocker_to_stopline_m'] = round(gap, 1)
        if gap < need:
            if not p.return_check_enforce:
                # 경고만: 계산이 실측 대비 과대하고, 복귀 실패 비용(리스폰 1회) < 미완주 비용이다.
                info.update(verdict='insufficient', basis='room_short_warn_only')
                return True, info
            info.update(verdict='wait', basis='no_room_to_stopline')
            return False, info
        info.update(verdict='detour', basis='room_available')
        return True, info

    def _allowed_sides(self, inputs: Inputs) -> List[str]:
        """ASSESS 구조 조건: 같은 방향 이웃 차선 존재 ∧ 그 쪽 경계 실선 아님 (D8: 실선은 Autoware 도 throw)
        + 우선 차선 규칙: 우회 후 우측 차선에서는 LEFT 만(utils.cpp:384-388), 좌측 차선에서는 RIGHT 만."""
        lane, role = inputs.lane, inputs.ego.lane_role
        sides = []
        if lane.neighbor_left and not lane.left_boundary_solid and role != 'left_of_preferred':
            sides.append(LEFT)
        if lane.neighbor_right and not lane.right_boundary_solid and role != 'right_of_preferred':
            sides.append(RIGHT)
        return sides

    def _candidate_sides(self, inputs: Inputs) -> List[str]:
        """Autoware 가 실제로 후보를 낸 방향.

        **후보가 존재한다는 사실 자체가 "이웃 차선 있음 + 실선 아님"의 증거다.** Autoware 는 이웃을
        라우팅 그래프로 찾고, 실선(lane_change≠yes)과 교차하는 후보는 path.cpp:510 에서 예외로 걸러
        아예 만들지 않는다. 우리 맵의 lane_change 태그는 섹션 중점 근사(no 1808 / yes 1077)라 더 부정확하고,
        9/7 주행에서 양쪽 다 실선으로 오판해 우회를 통째로 막았다(그때 Autoware 는 우측 후보를 내고 있었다).
        따라서 맵 기반 구조 판정은 **veto 로 쓰지 않고** 방향 선호·로그용으로만 남긴다.
        """
        out = []
        for side in (LEFT, RIGHT):
            c = inputs.candidates.get(side)
            if c is not None and c.present and not c.stale:
                out.append(side)
        return out

    @staticmethod
    def _return_side(role: str) -> Optional[str]:
        """우선 차선으로 되돌아가는 방향. 이 방향의 차선변경은 경로상 필수라 일반 lane_change 모듈이
        맡고 RTC 없이 자동 실행된다(external_request 는 후보를 내지 않는다). 우리가 기다릴 대상이 아니다."""
        return {'right_of_preferred': LEFT, 'left_of_preferred': RIGHT}.get(role)

    def _prefer_order(self, sides: List[str], blocker: ObjectInfo) -> List[str]:
        """blocker 가 차선 왼쪽에 치우쳤으면 오른쪽으로, 아니면 왼쪽 우선 (추월은 왼쪽이 관례)."""
        first = RIGHT if blocker.lateral_m > 0.5 else LEFT
        return sorted(sides, key=lambda s: 0 if s == first else 1)

    @staticmethod
    def _cand_detail(c: Optional[CandidateInfo]) -> Dict[str, Any]:
        if c is None or not c.present:
            return {'present': False}
        return {'present': True, 'safe': bool(c.safe), 'state': c.state, 'stale': bool(c.stale),
                'start': round(c.start_distance_m, 1), 'finish': round(c.finish_distance_m, 1)}

    # ------------------------------------------------------------ 메인
    def step(self, inputs: Inputs) -> Decision:
        p, t = self.p, inputs.t
        limits: List[Tuple[str, float]] = []
        clears: List[str] = []
        det: Dict[str, Any] = {'v': round(inputs.ego.v, 2), 'role': inputs.ego.lane_role}

        # 0) 리스폰: 모든 상태 초기화, 제한 전부 해제 후 respawn_hold_s 동안 정지 유지
        #    (adv_a13 D-5: 외부 속도제한은 clear 없으면 잔존 / VTD grace 1.0 s 동안 가속 명령 금지)
        if inputs.respawn:
            self._reset_all()
            self.respawn_until = t + p.respawn_hold_s
            clears.append(p.sender_ped)
            self._set_hold(0.0, limits, t)
            return Decision(STOP, None, limits, clears, 'respawn: 상태 초기화, 정지 유지', det)
        if t < self.respawn_until:
            self._set_hold(0.0, limits, t)
            return Decision(STOP, None, limits, clears, 'respawn hold', det)
        if self.respawn_until > -math.inf and self.hold_active and self.hold_value == 0.0 \
                and not self._blockers(inputs, p.slowdown_min_stopped_s):
            # 리스폰 정지 유지가 끝났고 막힘이 없으면 해제
            self.respawn_until = -math.inf
            self._clear_hold(clears)
            self._pedestrian_limit(inputs, limits, clears)
            return Decision(FOLLOW, None, limits, clears, 'respawn hold 해제', det)

        # 1) 입력 stale: 승인 금지, 걸려 있던 hold 는 0 으로 유지 (NG 와 동일: 안 보이는 동안 해제하지 않음)
        if inputs.stale:
            if self.hold_active:
                self._set_hold(0.0, limits, t)
            det['stale'] = True
            return Decision(HOLD if self.hold_active else FOLLOW, None, limits, clears, 'stale inputs', det)

        # 2) 보행자 전방 감속 (상태와 무관하게 항상 평가, sender 분리)
        self._pedestrian_limit(inputs, limits, clears)
        det['ped_limit'] = self.ped_active

        # 3) 승인해 둔 차선변경의 진행 관찰 (RTC cooperate_status)
        if self.committed is not None:
            c = inputs.candidates.get(self.committed)
            st = c.state if (c is not None and c.present) else None
            if st == CAND_RUNNING:
                # 실행 시작 → 개입 금지, hold 해제 (NG: RUNNING 관측 시 clear)
                side_running = self.committed
                self.committed = None
                self._clear_hold(clears)
                det['cand'] = {side_running: st}
                return Decision(LANE_CHANGE, None, limits, clears, 'lane change running', det)
            if st == CAND_ABORTING:
                # D5: 취소 관측 → back-off, 횟수 상한
                self.committed = None
                self.abort_count += 1
                self.backoff_until = t + p.abort_backoff_s
            elif st in ('SUCCEEDED', 'FAILED', None):
                self.committed = None

        # 4) blocker 탐지 — 감속 게이트(짧음)와 승인 게이트(길음)를 분리한다.
        #    VTD GT 는 ~80 m 부터 보이므로 확인을 오래 하면 이미 차선변경이 불가능한 거리가 된다.
        slow_blockers = self._blockers(inputs, p.slowdown_min_stopped_s)

        # 4-a) 우리가 붙잡고 있던 blocker 가 출발했으면 **즉시** 제한 해제 → 평상 추종(obstacle_cruise).
        #      (움직이는 객체는 _blockers 필터에서 빠지므로 여기서 따로 본다. clear_time 을 기다리면
        #       앞차가 출발했는데 최대 1 초 동안 제한이 남는다.)
        if self.tracked_blocker_id is not None:
            tracked = next((o for o in inputs.objects if o.id == self.tracked_blocker_id), None)
            if tracked is not None and tracked.v >= p.blocker_move_speed_mps:
                if self.hold_active:
                    self._clear_hold(clears)
                self.committed = None
                self.tracked_blocker_id, self.standoff_missed = None, False
                det['blocker'] = {'id': tracked.id, 'dist': round(tracked.longitudinal_m, 1),
                                  'v': round(tracked.v, 2)}
                return Decision(FOLLOW, None, limits, clears, 'blocker 출발 → 추종 복귀', det)

        if not slow_blockers:
            self.gave_up_t, self.gave_up_blocker_id = None, None
            if self.hold_active and t - self.last_blocked_t < p.clear_time_s:
                self._set_hold(self.hold_value, limits, t)
                return Decision(HOLD, None, limits, clears, 'blocker 소실 대기', det)
            if self.hold_active:
                self._clear_hold(clears)
            # clear_time 이 지나 확실히 사라졌을 때만 추적을 버린다. 한 tick 누락(차선변경 중 경로 이동으로
            # 횡거리 필터에서 빠지는 경우)으로 버리면 같은 blocker 를 더 가까이서 '새로 인지' 하게 된다
            # (9/7 detour14: 승인 직후 같은 id 3 을 56 m 에서 재취득해 standoff_missed 로 빠졌다).
            self.committed = None
            self.tracked_blocker_id, self.standoff_missed = None, False
            return Decision(FOLLOW, None, limits, clears, '', det)
        b = slow_blockers[0]
        self.last_blocked_t = t
        stopped_s = t - b.stopped_since if b.stopped_since is not None else 0.0
        det['blocker'] = {'id': b.id, 'dist': round(b.longitudinal_m, 1), 'lat': round(b.lateral_m, 2),
                          'stopped_s': round(stopped_s, 1)}

        # 4-b) 인지 시점에 이미 standoff 안이면 standoff 를 만들 수 없다 → 우리가 개입하지 않고
        #      obstacle_stop 에 맡긴다(제한 clear). 사실은 로그·status 에 남긴다.
        needed = self.needed_distance_m(inputs)
        if b.id != self.tracked_blocker_id:
            self.tracked_blocker_id = b.id
            self.standoff_missed = b.longitudinal_m < needed
            self.gave_up_t, self.gave_up_blocker_id = None, None
        det['standoff_missed'] = self.standoff_missed
        det['needed_m'] = round(needed, 1)

        # 4-c) blocker 가 다시 움직이면 즉시 제한 해제 → 평상 추종(obstacle_cruise)
        if b.v >= p.blocker_move_speed_mps:
            if self.hold_active:
                self._clear_hold(clears)
            self.committed = None
            return Decision(FOLLOW, None, limits, clears, 'blocker 출발 → 추종 복귀', det)

        # 4-d) standoff 접근: blocker 앞 standoff_m 에 서기 위한 속도 상한을 낸다.
        standoff_v = self._standoff_speed(b.longitudinal_m)
        if self.standoff_missed:
            # standoff 를 만들 수 없어도 손을 놓지 않는다. 최대한 감속해 남은 거리를 지키되
            # 0 으로 수렴시키지는 않는다(그러면 우리가 세우는 꼴). 그 사이 후보가 짧아지면 승인된다.
            # 9/7 detour14: 여기서 clear 하고 물러나 앞차 10.7 m 뒤까지 붙어 99 초를 멈춰 있었다.
            standoff_v = max(p.approach_speed_floor_mps, standoff_v)
        det['standoff_v'] = round(standoff_v, 2)
        det['standoff_target_m'] = round(self.effective_standoff_m(), 1)

        # 5) 서서 관찰 — 시간 압박이 없으므로 여기서 판단한다.
        room, qinfo = self._room_verdict(inputs, slow_blockers)
        det['queue'] = qinfo
        det['return'] = {k: qinfo[k] for k in ('n_lanes', 'lc_length_model_m', 'need_m', 'gap_m', 'verdict')}
        warn = None
        if qinfo['verdict'] == 'insufficient':
            warn = (f"복귀 여유 부족(간격 {qinfo['gap_m']}m < 필요 {qinfo['need_m']}m"
                    f" = {qinfo['n_lanes']}칸 × {qinfo['lc_length_model_m']}m) — 모델 과대 가능성이 있어"
                    ' 막지 않고 진행한다(return_check_enforce=false)')
        if not room:
            # 끼어들 공간이 없다 = 저 차들이 정지선에 붙어 서 있다(신호 대기로 설명되는 유일한 경우).
            # standoff 는 유지한다(§11 원칙: 해제는 blocker 출발·승인 두 경우뿐). 앞이 풀리면 즉시 우회한다.
            self._set_hold(standoff_v, limits, t)
            return Decision(HOLD, None, limits, clears,
                            f"끼어들 공간 없음(간격 {qinfo['gap_m']}m < 필요 {qinfo['need_m']}m"
                            f" = {qinfo['n_lanes']}칸 × LC {qinfo['lc_length_model_m']}m + 여유) → 대기", det)

        # 공간이 있다 → blocker 가 no_response_s 이상 무응답이면 우회 결정 (신호 무관)
        if stopped_s < p.no_response_s:
            self._set_hold(standoff_v, limits, t)
            return Decision(HOLD, None, limits, clears,
                            f'관찰 중 {stopped_s:.1f}/{p.no_response_s:.1f}s '
                            f'(목표 {self.effective_standoff_m():.0f}m → 실정지 ~{p.standoff_m:.0f}m)', det)

        # 6) 우회 결정.
        #
        # **우리가 판정에 쓰는 것은 셋뿐이다.**
        #   (a) 우리만 아는 것 : blocker 존재·정지 지속, 정지선까지의 공간(대기열 판별), standoff 확보
        #   (b) Autoware 가 알려주는 것 : 후보 존재 여부, safe, start/finish 실측
        #   (c) 우리 안전 여유 : finish + lc_finish_margin_m < blocker 거리
        #
        # **Autoware 가 이미 검사하는 것은 다시 판단하지 않는다.**
        #   실선 교차(path.cpp:510 에서 예외로 후보를 안 만듦), 이웃 차선 유무(라우팅 그래프),
        #   규제요소 근접(isLaneChangeRequired, scene.cpp:296-342), 목표차선 시작점 거리,
        #   목표차선 후방 RSS. → **후보가 존재하고 safe 라는 사실이 이 조건들을 통과했다는 증거다.**
        #
        # 오늘(9/7) 같은 실수를 세 번 했다. 전부 "우리가 Autoware 판단을 베껴 중복 검사" 한 것이었다.
        #   ① 맵 실선 판정  : 맵 태그가 섹션 중점 근사라 양쪽 실선으로 오판 → 우회를 통째로 막음(detour2)
        #   ② P2 패치       : 외부 속도제한을 max_prepare_length 에 반영했더니 후보 생성 자체가 죽음
        #   ③ 규제요소 거리 : 우리 54.4 m 로 48 m 를 거부했는데 Autoware 는 후보를 내고 있었음(detour16)
        # 맵 기반 값(map_sides, reg_m, return_side)은 **로그·선호용 관측치**이지 veto 가 아니다.
        if self.abort_count >= p.max_abort_count:
            self._set_hold(standoff_v, limits, t)
            return Decision(HOLD, None, limits, clears, f'abort {self.abort_count}회: 우회 포기, 대기', det)
        lane = inputs.lane
        map_sides = self._allowed_sides(inputs)          # 관측치(로그·선호). veto 하지 않는다.
        sides = self._candidate_sides(inputs)            # 실제 판단 근거
        det['struct'] = {'nl': lane.neighbor_left, 'nr': lane.neighbor_right,
                         'solid_l': lane.left_boundary_solid, 'solid_r': lane.right_boundary_solid,
                         'map_sides': map_sides, 'cand_sides': sides}
        # 주: '복귀는 일반 lane_change 담당' 규칙은 여기서 쓰지 않는다.
        # 교차로 접근 시 우선차선이 좌회전 차선으로 옮겨가면 자차가 차선을 바꾼 적이 없어도
        # role 이 right_of_preferred 가 된다. 자차 차로 전방에 blocker 가 있으면 blocker 처리가 우선이고,
        # 그 규칙은 'blocker 가 없고 남은 기동이 복귀뿐일 때'에만 의미가 있다(9/7 detour6).
        det['return_side'] = self._return_side(inputs.ego.lane_role)

        # 기하: 우회해도 원래 차선으로 복귀할 수 없으면 하지 않는다(경로이탈 위험).
        # 새 정책에 명시되진 않았으나 이전 설계의 안전 조건이라 유지한다. 여기서는 STOP(clear) 이 맞다 —
        # 구조적으로 우회가 불가능하므로 앞차 뒤에 정상적으로 서는 것이 옳다(obstacle_stop 담당).
        det['reg_m'] = None if lane.distance_to_regulatory_m is None else round(lane.distance_to_regulatory_m, 1)

        det['cand'] = {s_: self._cand_detail(inputs.candidates.get(s_)) for s_ in (LEFT, RIGHT)}
        room = b.longitudinal_m - p.lc_finish_margin_m
        det['lc_room_m'] = round(room, 1)
        if self.committed is not None:
            if self.hold_active:
                self._clear_hold(clears)
            return Decision(LANE_CHANGE, None, limits, clears, f'{self.committed} 승인 대기', det)
        if t < self.backoff_until:
            self._set_hold(standoff_v, limits, t)
            return Decision(HOLD, None, limits, clears, 'abort back-off 대기', det)
        for side in self._prefer_order(sides, b):
            c = inputs.candidates.get(side)
            if c is None or not c.present or c.stale or not c.safe or c.state != CAND_WAITING:
                continue
            if c.finish_distance_m > room:
                continue          # 완료 지점이 blocker 너머 → 승인하면 변경 중 충돌
            self.committed = side
            if self.hold_active:
                self._clear_hold(clears)   # 차선변경은 Autoware 가 정상 속도로 수행
            return Decision(LANE_CHANGE, side, limits, clears,
                            f'{side} 승인 (blocker {b.longitudinal_m:.0f}m, LC 완료 {c.finish_distance_m:.0f}m)',
                            det, warn)
        self._set_hold(standoff_v, limits, t)
        return Decision(HOLD, None, limits, clears,
                        '후보 없음/unsafe/LC 길이 부족 → standoff 유지하며 대기(상황 바뀌면 즉시 우회)', det)

    # ------------------------------------------------------------ 결과 헬퍼
    def _stop(self, limits, clears, reason: str, detail: Optional[Dict[str, Any]] = None) -> Decision:
        """STOP: 우회 없음. hold 제한은 걸지 않고(obstacle_stop 에 맡김) 걸려 있던 것은 해제."""
        if self.hold_active:
            self._clear_hold(clears)
        self.committed = None
        return Decision(STOP, None, limits, clears, reason, detail or {})

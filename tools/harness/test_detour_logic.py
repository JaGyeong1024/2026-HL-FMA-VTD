"""판단 노드 순수 로직 하네스 (todo0906 묶음 2 사양).

각 테스트는 '조건 → 기대 Decision' 한 쌍이다. 구현은 detour_logic.DetourLogic.step 에 들어간다.
"""
import pytest

from vtd_autoware_bridge.detour_logic import (
    DetourLogic, Params, EgoState, ObjectInfo, LaneInfo, SignalInfo, CandidateInfo, Inputs,
    FOLLOW, ASSESS, HOLD, LANE_CHANGE, STOP, LEFT, RIGHT,
)

P = Params()
T0 = 100.0


def car(longitudinal, stopped_for=5.0, cls='CAR', lateral=0.0, t=T0, v=0.0):
    return ObjectInfo(id=1, cls=cls, v=v, longitudinal_m=longitudinal, lateral_m=lateral,
                      stopped_since=(t - stopped_for) if stopped_for is not None else None)


def ped(longitudinal, lateral, toward=0.0, v=1.0):
    return ObjectInfo(id=9, cls='PEDESTRIAN', v=v, longitudinal_m=longitudinal, lateral_m=lateral,
                      stopped_since=None, lateral_speed_toward_path_mps=toward)


def lane(left=True, right=True, reg=200.0, stopline=200.0, left_solid=False, right_solid=False):
    return LaneInfo(neighbor_left=left, neighbor_right=right,
                    left_boundary_solid=left_solid, right_boundary_solid=right_solid,
                    distance_to_regulatory_m=reg, distance_to_stopline_m=stopline)


def cand(safe=True, state='WAITING', start=6.0, finish=25.0, present=True, stale=False):
    return CandidateInfo(present=present, safe=safe, state=state,
                         start_distance_m=start, finish_distance_m=finish, stale=stale)


def inputs(objects=(), lane_=None, signal=None, cands=None, t=T0, v=12.0, role='preferred',
           respawn=False, stale=False):
    return Inputs(t=t, ego=EgoState(t=t, v=v, lane_role=role), objects=list(objects),
                  lane=lane_ or lane(), signal=signal or SignalInfo(None, True),
                  candidates=cands if cands is not None else {LEFT: cand(), RIGHT: cand()},
                  respawn=respawn, stale=stale)


def run(logic, *steps):
    d = None
    for s in steps:
        d = logic.step(s)
    return d


# ---------------------------------------------------------------- blocker 판정
def test_no_objects_is_follow():
    d = DetourLogic(P).step(inputs())
    assert d.state == FOLLOW and d.approve is None and d.limits == []


def test_pedestrian_on_lane_is_not_blocker():
    d = DetourLogic(P).step(inputs([ped(30.0, 0.0, v=0.0)]))
    assert d.state != HOLD and d.approve is None
    assert all(s != P.sender_hold for s, _ in d.limits)


def test_stopped_less_than_3s_ignored():
    d = DetourLogic(P).step(inputs([car(60.0, stopped_for=1.0)]))
    assert d.state == FOLLOW and d.approve is None


def test_moving_vehicle_ignored():
    d = DetourLogic(P).step(inputs([car(60.0, stopped_for=None, v=8.0)]))
    assert d.state == FOLLOW


def test_vehicle_outside_lane_margin_ignored():
    d = DetourLogic(P).step(inputs([car(60.0, lateral=3.5)]))
    assert d.state == FOLLOW


def test_vehicle_beyond_detection_ignored():
    d = DetourLogic(P).step(inputs([car(P.detection_distance_m + 5.0)]))
    assert d.state == FOLLOW


# ---------------------------------------------------------------- ASSESS 4조건 → STOP / HOLD
def test_no_neighbor_lane_is_stop():
    d = DetourLogic(P).step(inputs([car(60.0)], lane_=lane(left=False, right=False)))
    assert d.state == STOP and d.approve is None
    assert all(s != P.sender_hold for s, _ in d.limits)


def test_solid_boundary_blocks_that_side():
    d = DetourLogic(P).step(inputs([car(60.0)], lane_=lane(left=False, right=True, right_solid=True)))
    assert d.state == STOP and d.approve is None


def test_regulatory_within_clearance_is_stop():
    d = DetourLogic(P).step(inputs([car(60.0)], lane_=lane(reg=P.regulatory_clearance_m - 1.0)))
    assert d.state == STOP and d.approve is None


def test_stopline_distance_below_threshold_is_stop():
    # 정지차~정지선 = stopline − longitudinal < 28m → 복귀 불가
    d = DetourLogic(P).step(inputs([car(60.0)], lane_=lane(reg=200.0, stopline=60.0 + P.stopline_min_distance_m - 1.0)))
    assert d.state == STOP and d.approve is None


def test_stopline_distance_at_threshold_allows():
    d = DetourLogic(P).step(inputs([car(60.0)], lane_=lane(reg=200.0, stopline=60.0 + P.stopline_min_distance_m)))
    assert d.state in (HOLD, LANE_CHANGE)


def test_red_signal_queue_is_stop_no_approval():
    # 적신호 STOP factor 가 정지차 너머(정지선)에 있음 = 대기열
    sig = SignalInfo(red_stop_factor_distance_m=90.0, available=True)
    d = DetourLogic(P).step(inputs([car(60.0)], signal=sig))
    assert d.state == STOP and d.approve is None


def test_signal_unavailable_and_short_stopline_no_bypass():
    # 신호 정보 없음 AND 정지선 거리 임계 미만 → 우회 없음 (AND 규칙)
    sig = SignalInfo(red_stop_factor_distance_m=None, available=False)
    d = DetourLogic(P).step(inputs([car(60.0)], signal=sig, lane_=lane(stopline=60.0 + 10.0)))
    assert d.state == STOP and d.approve is None


def test_green_and_long_stopline_bypass_allowed():
    sig = SignalInfo(red_stop_factor_distance_m=None, available=True)
    d = DetourLogic(P).step(inputs([car(60.0)], signal=sig, lane_=lane(stopline=60.0 + 40.0)))
    assert d.state == LANE_CHANGE and d.approve in (LEFT, RIGHT)


# ---------------------------------------------------------------- HOLD / 승인
def test_unsafe_candidate_holds_at_approach_speed():
    d = DetourLogic(P).step(inputs([car(60.0)], cands={LEFT: cand(safe=False), RIGHT: cand(safe=False)}))
    assert d.state == HOLD and d.approve is None
    assert (P.sender_hold, P.approach_speed_mps) in d.limits


def test_hold_limit_is_zero_inside_hold_distance():
    d = DetourLogic(P).step(inputs([car(40.0)], cands={LEFT: cand(safe=False), RIGHT: cand(safe=False)}))
    lim = dict(d.limits)
    assert d.state == HOLD and lim[P.sender_hold] == 0.0


def test_safe_candidate_is_approved_once():
    logic = DetourLogic(P)
    d1 = logic.step(inputs([car(60.0)]))
    assert d1.state == LANE_CHANGE and d1.approve in (LEFT, RIGHT)
    d2 = logic.step(inputs([car(60.0)], cands={d1.approve: cand(state='RUNNING'), (RIGHT if d1.approve == LEFT else LEFT): cand()}, t=T0 + 0.2))
    assert d2.approve is None and P.sender_hold in d2.clears


def test_stale_candidate_not_approved():
    d = DetourLogic(P).step(inputs([car(60.0)], cands={LEFT: cand(stale=True), RIGHT: cand(stale=True)}))
    assert d.approve is None and d.state == HOLD


def test_right_of_preferred_lane_only_left_allowed():
    d = DetourLogic(P).step(inputs([car(60.0)], role='right_of_preferred'))
    assert d.approve != RIGHT


def test_abort_triggers_backoff_then_stop_after_limit():
    logic = DetourLogic(P)
    t = T0
    d = logic.step(inputs([car(60.0)], t=t)); side = d.approve
    other = RIGHT if side == LEFT else LEFT
    aborting = {side: cand(state='ABORTING'), other: cand()}
    d = logic.step(inputs([car(60.0)], cands=aborting, t=t + 1.0))
    # back-off 동안 재승인 금지
    d = logic.step(inputs([car(60.0)], t=t + 2.0))
    assert d.approve is None
    d = logic.step(inputs([car(60.0)], t=t + 1.0 + P.abort_backoff_s + 0.1))
    assert d.approve is not None                       # back-off 후 1회 재시도
    d = logic.step(inputs([car(60.0)], cands=aborting, t=t + 13.0))
    d = logic.step(inputs([car(60.0)], t=t + 13.0 + P.abort_backoff_s + 0.1))
    if P.max_abort_count <= 2:
        assert d.state == STOP and d.approve is None   # 상한 초과 → STOP


# ---------------------------------------------------------------- 보행자 전방 감속
def test_pedestrian_near_lane_edge_limits_speed_separate_sender():
    d = DetourLogic(P).step(inputs([ped(30.0, 1.75 + 1.0)]))
    lim = dict(d.limits)
    assert lim.get(P.sender_ped) == pytest.approx(P.ped_speed_limit_mps)
    assert P.sender_hold not in lim


def test_pedestrian_far_from_lane_no_limit():
    d = DetourLogic(P).step(inputs([ped(30.0, 1.75 + P.ped_lateral_margin_m + 1.0)]))
    assert P.sender_ped not in dict(d.limits)


def test_pedestrian_moving_toward_lane_limits_even_if_far():
    d = DetourLogic(P).step(inputs([ped(30.0, 1.75 + P.ped_lateral_margin_m + 1.0, toward=1.5)]))
    assert dict(d.limits).get(P.sender_ped) == pytest.approx(P.ped_speed_limit_mps)


def test_pedestrian_beyond_lookahead_no_limit():
    d = DetourLogic(P).step(inputs([ped(P.ped_lookahead_m + 5.0, 2.0)]))
    assert P.sender_ped not in dict(d.limits)


def test_pedestrian_limit_released_after_delay():
    logic = DetourLogic(P)
    logic.step(inputs([ped(30.0, 2.0)], t=T0))
    d = logic.step(inputs([], t=T0 + 0.5))
    assert P.sender_ped not in d.clears                   # 1s 전엔 유지
    d = logic.step(inputs([], t=T0 + P.ped_release_delay_s + 0.1))
    assert P.sender_ped in d.clears


def test_hold_and_pedestrian_limits_coexist():
    d = DetourLogic(P).step(inputs([car(60.0), ped(30.0, 2.0)],
                                   cands={LEFT: cand(safe=False), RIGHT: cand(safe=False)}))
    lim = dict(d.limits)
    assert P.sender_hold in lim and P.sender_ped in lim


# ---------------------------------------------------------------- 리스폰 / stale
def test_respawn_clears_limits_resets_and_holds_zero():
    logic = DetourLogic(P)
    logic.step(inputs([car(60.0)], cands={LEFT: cand(safe=False), RIGHT: cand(safe=False)}, t=T0))
    d = logic.step(inputs([], respawn=True, t=T0 + 1.0))
    assert set(d.clears) >= {P.sender_hold, P.sender_ped} or dict(d.limits).get(P.sender_hold) == 0.0
    assert dict(d.limits).get(P.sender_hold) == 0.0
    d = logic.step(inputs([], t=T0 + 1.0 + P.respawn_hold_s + 0.1))
    assert P.sender_hold in d.clears and d.state == FOLLOW


def test_respawn_resets_abort_counter():
    logic = DetourLogic(P)
    d = logic.step(inputs([car(60.0)], t=T0)); side = d.approve
    other = RIGHT if side == LEFT else LEFT
    for k in range(P.max_abort_count + 1):
        logic.step(inputs([car(60.0)], cands={side: cand(state='ABORTING'), other: cand()}, t=T0 + 20 * k + 1))
        logic.step(inputs([car(60.0)], t=T0 + 20 * k + 1 + P.abort_backoff_s + 0.1))
    logic.step(inputs([], respawn=True, t=T0 + 200.0))
    d = logic.step(inputs([car(60.0)], t=T0 + 200.0 + P.respawn_hold_s + 0.1))
    assert d.state in (LANE_CHANGE, HOLD)


def test_stale_inputs_no_approval_and_keep_hold():
    logic = DetourLogic(P)
    logic.step(inputs([car(40.0)], cands={LEFT: cand(safe=False), RIGHT: cand(safe=False)}, t=T0))
    d = logic.step(inputs([car(40.0)], stale=True, t=T0 + 0.2))
    assert d.approve is None
    assert dict(d.limits).get(P.sender_hold) == 0.0      # stale 동안 정지 유지


def test_blocker_gone_releases_hold_after_clear_time():
    logic = DetourLogic(P)
    logic.step(inputs([car(60.0)], cands={LEFT: cand(safe=False), RIGHT: cand(safe=False)}, t=T0))
    d = logic.step(inputs([], t=T0 + 2.0))
    assert P.sender_hold in d.clears and d.state == FOLLOW

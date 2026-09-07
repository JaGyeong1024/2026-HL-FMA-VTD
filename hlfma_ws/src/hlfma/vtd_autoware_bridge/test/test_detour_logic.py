"""판단 노드 순수 로직 하네스 — 9/7 정책 교체판.

정책: 자차 차로 전방 정지차를 보면 신호와 무관하게 동일 처리 →
  우회 가능 거리(standoff_m)를 남기고 감속·정지 → **서서 관찰** →
  적색이면 대기, 아니면 무응답 no_response_s 후 우회 결정 → 후보 safe & finish+여유<거리 면 승인.
시간 압박(인지 80 m / 1.5 초 예산)이 사라지는 것이 이 설계의 요점이다.
"""
import pytest

from vtd_autoware_bridge.detour_logic import (
    DetourLogic, Params, EgoState, ObjectInfo, LaneInfo, SignalInfo, CandidateInfo, Inputs,
    FOLLOW, ASSESS, HOLD, LANE_CHANGE, STOP, LEFT, RIGHT,
)

P = Params()
T0 = 100.0
EFF = P.standoff_m + P.standoff_margin_m   # 프로파일이 겨냥하는 지점(오버슈트 보정 포함)
# 실제 인지 거리(detour9: 76.6 m)에 맞춘다. detection_distance_m(80) 안이면서 EFF 보다 멀어야 한다.
FAR = 76.0
LONG = P.no_response_s + 1.0       # 무응답 판정을 넘긴 정지 시간


def car(longitudinal=FAR, stopped_for=LONG, cls='CAR', lateral=0.0, t=T0, v=0.0):
    return ObjectInfo(id=1, cls=cls, v=v, longitudinal_m=longitudinal, lateral_m=lateral,
                      stopped_since=(t - stopped_for) if stopped_for is not None else None)


def ped(longitudinal, lateral, toward=0.0, v=1.0):
    return ObjectInfo(id=9, cls='PEDESTRIAN', v=v, longitudinal_m=longitudinal, lateral_m=lateral,
                      stopped_since=None, lateral_speed_toward_path_mps=toward)


def lane(left=True, right=True, reg=500.0, stopline=500.0, left_solid=False, right_solid=False):
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


def standoff_v(d):
    import math
    return min(P.approach_speed_mps,
               math.sqrt(2.0 * P.standoff_decel_mps2 * max(0.0, d - EFF)))


UNSAFE = {LEFT: cand(safe=False), RIGHT: cand(safe=False)}
NONE_C = {LEFT: cand(present=False), RIGHT: cand(present=False)}


# ---------------------------------------------------------------- blocker 판정
def test_no_objects_is_follow():
    d = DetourLogic(P).step(inputs())
    assert d.state == FOLLOW and d.approve is None and d.limits == []


def test_pedestrian_is_not_blocker():
    d = DetourLogic(P).step(inputs([ped(30.0, 0.0, v=0.0)]))
    assert all(s != P.sender_hold for s, _ in d.limits)


def test_moving_vehicle_ignored():
    d = DetourLogic(P).step(inputs([car(stopped_for=None, v=8.0)]))
    assert d.state == FOLLOW


def test_vehicle_outside_lane_margin_ignored():
    d = DetourLogic(P).step(inputs([car(lateral=3.5)]))
    assert d.state == FOLLOW


def test_vehicle_beyond_detection_ignored():
    d = DetourLogic(P).step(inputs([car(P.detection_distance_m + 5.0)]))
    assert d.state == FOLLOW


def test_stopped_below_slowdown_gate_ignored():
    d = DetourLogic(P).step(inputs([car(stopped_for=P.slowdown_min_stopped_s / 2.0)]))
    assert d.state == FOLLOW


# ---------------------------------------------------------------- 감속 → standoff 정지
def test_approach_profile_targets_effective_standoff():
    """프로파일은 오버슈트를 감안해 standoff_m 보다 **뒤**를 겨냥한다.
    9/7 detour9 실측: 목표 48 m 인데 실제 31.4 m 에 정지(오버슈트 16.6 m)."""
    lg = DetourLogic(P)
    assert lg.effective_standoff_m() == pytest.approx(P.standoff_m + P.standoff_margin_m)
    assert lg._standoff_speed(EFF + 40.0) == pytest.approx(P.approach_speed_mps)
    assert 0.0 < lg._standoff_speed(EFF + 3.0) < P.approach_speed_mps
    assert lg._standoff_speed(EFF) == pytest.approx(0.0)
    # 오버슈트를 겪어도 실제 정지 지점이 standoff_m 이상이어야 한다
    assert EFF - P.standoff_margin_m >= P.standoff_m


def test_overshoot_margin_covers_measured_worst_case():
    """실측 오버슈트 11~16.6 m 를 덮어야 한다."""
    assert P.standoff_margin_m >= 16.6


def test_observing_before_no_response_holds_at_standoff_profile():
    short = P.no_response_s / 2.0
    d = DetourLogic(P).step(inputs([car(FAR, stopped_for=short)]))
    assert d.state == HOLD and d.approve is None
    assert dict(d.limits).get(P.sender_hold) == pytest.approx(standoff_v(FAR))
    assert '관찰' in d.reason


def test_standoff_missed_hands_over_to_obstacle_stop():
    """인지 시점에 이미 standoff 안이면 standoff 를 만들 수 없다 → 개입하지 않고 넘긴다."""
    d = DetourLogic(P).step(inputs([car(EFF - 10.0)]))
    assert d.state == STOP and d.approve is None
    assert all(s != P.sender_hold for s, _ in d.limits)
    assert d.detail.get('standoff_missed') is True


def test_standoff_kept_once_established_even_when_close():
    """standoff 를 만들 수 있었던 blocker 는 가까워져도 계속 우리가 관리한다."""
    lg = DetourLogic(P)
    lg.step(inputs([car(FAR, stopped_for=0.3)], t=T0))
    d = lg.step(inputs([car(EFF + 1.0, stopped_for=1.0, t=T0 + 1.0)], t=T0 + 1.0))
    assert d.state == HOLD and d.detail.get('standoff_missed') is False


# ---------------------------------------------------------------- 서서 관찰: 적색 / 무신호
def RED_SIG(color='RED'):
    return SignalInfo(red_stop_factor_distance_m=None, available=True, color=color, color_age_s=0.0)


def GREEN_SIG():
    return SignalInfo(red_stop_factor_distance_m=None, available=True, color='GREEN', color_age_s=0.0)


def test_red_signal_waits_without_detour():
    """적색이면 정당한 대기열이다. 우회하지 않고 standoff 제한을 유지한 채 기다린다."""
    sig = RED_SIG()
    d = DetourLogic(P).step(inputs([car()], signal=sig))
    assert d.state == HOLD and d.approve is None
    assert P.sender_hold in dict(d.limits)
    assert '적신호' in d.reason


def test_signal_cycling_flips_red_queue_both_ways():
    """9/7 detour9: 신호가 1(적)→5(녹+화살표)→1→5→2(황) 로 순환하는데 red_queue 가 true 로 고착됐다.
    색을 1차 근거로 삼고 양방향 동일 디바운스를 적용해 따라 바뀌어야 한다."""
    lg = DetourLogic(P)
    t = T0
    assert lg.step(inputs([car(t=t)], signal=RED_SIG(), t=t)).state == HOLD          # 적 → 대기
    # 초록으로 바뀌면 디바운스 후 우회 판단으로 넘어간다
    t += P.signal_debounce_s + 0.1
    lg.step(inputs([car(t=t)], signal=GREEN_SIG(), t=t))
    t += P.signal_debounce_s + 0.1
    d = lg.step(inputs([car(t=t)], signal=GREEN_SIG(), t=t))
    assert d.state == LANE_CHANGE and d.approve is not None
    assert d.detail['signal']['color'] == 'GREEN' and d.detail['red_queue'] is False
    # 다시 적색이면 같은 조건으로 되돌아온다 (한 방향 고착 아님)
    for _ in range(3):
        t += P.signal_debounce_s + 0.1
        d = lg.step(inputs([car(t=t)], signal=RED_SIG(), t=t))
    assert d.detail['red_queue'] is True and '적신호' in d.reason


def test_amber_counts_as_red():
    d = DetourLogic(P).step(inputs([car()], signal=RED_SIG('AMBER')))
    assert d.state == HOLD and '적신호' in d.reason


def test_no_signal_area_stopped_car_is_detour_target():
    """신호가 아예 없는 구간의 정차차도 우회 대상이어야 한다(정보 없음 = 비적색)."""
    sig = SignalInfo(red_stop_factor_distance_m=None, available=False, color=None, color_age_s=None)
    d = DetourLogic(P).step(inputs([car()], signal=sig))
    assert d.state == LANE_CHANGE and d.approve is not None
    assert d.detail['red_queue'] is False


def test_signal_stale_holds_previous_briefly_then_releases():
    """신호 **메시지 자체가** stale 일 때만 직전값을 짧게 유지하고, 그 뒤에는 비적색으로 본다."""
    lg = DetourLogic(P)
    t = T0
    assert lg.step(inputs([car(t=t)], signal=RED_SIG(), t=t)).state == HOLD
    stale_sig = SignalInfo(red_stop_factor_distance_m=None, available=False, color=None, color_age_s=5.0)
    t += 0.2
    d = lg.step(inputs([car(t=t)], signal=stale_sig, t=t))
    assert d.detail['red_queue'] is True            # 짧은 동안은 유지
    t += P.signal_hold_s + P.signal_debounce_s + 0.3
    lg.step(inputs([car(t=t)], signal=stale_sig, t=t))
    t += P.signal_debounce_s + 0.1
    d = lg.step(inputs([car(t=t)], signal=stale_sig, t=t))
    assert d.detail['red_queue'] is False           # 이후에는 비적색


def test_signal_detail_carries_basis():
    d = DetourLogic(P).step(inputs([car()], signal=RED_SIG()))
    sd = d.detail['signal']
    assert sd['color'] == 'RED' and sd['fresh'] is True and 'color=RED' in sd['basis']


def test_green_and_no_response_triggers_detour():
    d = DetourLogic(P).step(inputs([car()], signal=GREEN_SIG()))
    assert d.state == LANE_CHANGE and d.approve in (LEFT, RIGHT)


def test_no_signal_info_also_triggers_detour_after_no_response():
    """관련 신호가 아예 없어도 무응답이면 동일하게 우회한다(신호 유무로 갈라지지 않는다)."""
    sig = SignalInfo(red_stop_factor_distance_m=None, available=False, color=None)
    d = DetourLogic(P).step(inputs([car()], signal=sig))
    assert d.state == LANE_CHANGE and d.approve in (LEFT, RIGHT)


# ---------------------------------------------------------------- blocker 출발 → 추종 복귀
def test_blocker_moving_clears_limit_and_follows():
    lg = DetourLogic(P)
    lg.step(inputs([car(FAR, stopped_for=0.3)], t=T0))
    moving = ObjectInfo(id=1, cls='CAR', v=P.blocker_move_speed_mps + 0.2, longitudinal_m=FAR,
                        lateral_m=0.0, stopped_since=T0 - 5.0)
    d = lg.step(inputs([moving], t=T0 + 1.0))
    assert d.state == FOLLOW and P.sender_hold in d.clears
    assert '출발' in d.reason


def test_blocker_gone_releases_hold_after_clear_time():
    lg = DetourLogic(P)
    lg.step(inputs([car(FAR, stopped_for=0.3)], cands=UNSAFE, t=T0))
    d = lg.step(inputs([], t=T0 + 2.0))
    assert P.sender_hold in d.clears and d.state == FOLLOW


# ---------------------------------------------------------------- 우회 결정 → 승인
def test_safe_candidate_approved_and_limit_cleared():
    """승인되면 제한을 해제해 Autoware 가 정상 속도로 차선변경한다."""
    lg = DetourLogic(P)
    lg.step(inputs([car(FAR, stopped_for=0.3)], cands=UNSAFE, t=T0))    # 먼저 제한이 걸린 상태
    d = lg.step(inputs([car(FAR, t=T0 + 0.2)], t=T0 + 0.2))
    assert d.state == LANE_CHANGE and d.approve in (LEFT, RIGHT)
    assert P.sender_hold in d.clears


def test_candidate_finishing_beyond_blocker_not_approved():
    far_c = cand(safe=True, state='WAITING', start=26.0, finish=98.5)
    d = DetourLogic(P).step(inputs([car(FAR)], cands={LEFT: far_c, RIGHT: far_c}))
    assert d.approve is None and d.state == HOLD


def test_unsafe_candidate_holds():
    d = DetourLogic(P).step(inputs([car(FAR)], cands=UNSAFE))
    assert d.state == HOLD and d.approve is None
    assert P.sender_hold in dict(d.limits)


def test_stale_candidate_not_approved():
    d = DetourLogic(P).step(inputs([car(FAR)], cands={LEFT: cand(stale=True), RIGHT: cand(stale=True)}))
    assert d.approve is None and d.state == HOLD


def test_map_structure_does_not_veto_when_candidate_exists():
    """9/7 detour2: 맵이 양쪽 실선으로 오판해 우회를 막았다. 후보 존재가 구조 조건의 증거다."""
    only_right = {LEFT: cand(present=False), RIGHT: cand(safe=True, state='WAITING', start=8.0, finish=40.0)}
    d = DetourLogic(P).step(inputs([car(FAR)],
                                   lane_=lane(left_solid=True, right_solid=True), cands=only_right))
    assert d.state == LANE_CHANGE and d.approve == RIGHT


def test_blocker_present_ignores_return_duty_rule():
    """9/7 detour6: 교차로 접근 시 우선차선이 좌회전 차선으로 옮겨가면 자차가 차선을 바꾼 적 없어도
    role 이 right_of_preferred 가 된다. blocker 가 있으면 blocker 처리가 우선이고 standoff 를 지킨다."""
    d = DetourLogic(P).step(inputs([car(FAR)], role='right_of_preferred', cands=NONE_C))
    assert d.state == HOLD and d.approve is None
    assert P.sender_hold in dict(d.limits)          # 제한을 풀지 않는다
    assert P.sender_hold not in d.clears


def test_regulatory_within_clearance_keeps_standoff():
    d = DetourLogic(P).step(inputs([car(FAR)], lane_=lane(reg=P.regulatory_clearance_m - 1.0)))
    assert d.state == HOLD and d.approve is None
    assert dict(d.limits).get(P.sender_hold) == pytest.approx(standoff_v(FAR))


def test_stopline_gap_below_threshold_keeps_standoff():
    d = DetourLogic(P).step(inputs([car(FAR)],
                                   lane_=lane(stopline=FAR + P.stopline_min_distance_m - 1.0)))
    assert d.state == HOLD and d.approve is None
    assert P.sender_hold in dict(d.limits)


# ------------------------------------------- standoff 유지 원칙 (9/7 detour6)
@pytest.mark.parametrize('name,kw', [
    ('적신호 대기열', dict(signal=SignalInfo(None, True, 'RED', 0.0))),
    ('후보 없음', dict(cands=NONE_C)),
    ('후보 unsafe', dict(cands=UNSAFE)),
    ('후보 stale', dict(cands={LEFT: cand(stale=True), RIGHT: cand(stale=True)})),
    ('LC 길이 부족', dict(cands={LEFT: cand(finish=98.5), RIGHT: cand(finish=98.5)})),
    ('규제요소 근접', dict(lane_=lane(reg=P.regulatory_clearance_m - 1.0))),
    ('정지선 여유 부족', dict(lane_=lane(stopline=FAR + P.stopline_min_distance_m - 1.0))),
    ('복귀 방향뿐', dict(role='right_of_preferred', cands=NONE_C)),
])
def test_cannot_detour_now_keeps_standoff_limit(name, kw):
    """'지금은 불가' 는 제한을 푸는 이유가 아니다. 풀면 blocker 코앞까지 기어가 우회 여유를 잃는다."""
    d = DetourLogic(P).step(inputs([car(FAR)], **kw))
    assert d.state == HOLD, name
    assert P.sender_hold in dict(d.limits), name
    assert P.sender_hold not in d.clears, name


def test_only_blocker_departure_and_approval_clear_the_limit():
    """제한 해제는 ① blocker 출발 ② 우회 승인, 두 경우뿐이다."""
    # ① 출발
    lg = DetourLogic(P)
    lg.step(inputs([car(FAR, stopped_for=0.3)], cands=UNSAFE, t=T0))
    moving = ObjectInfo(id=1, cls='CAR', v=P.blocker_move_speed_mps + 0.2, longitudinal_m=FAR,
                        lateral_m=0.0, stopped_since=T0 - 5.0)
    d = lg.step(inputs([moving], t=T0 + 0.4))
    assert d.state == FOLLOW and P.sender_hold in d.clears
    # ② 승인
    lg2 = DetourLogic(P)
    lg2.step(inputs([car(FAR, stopped_for=0.3)], cands=UNSAFE, t=T0))
    d = lg2.step(inputs([car(FAR, t=T0 + 0.2)], t=T0 + 0.2))
    assert d.state == LANE_CHANGE and d.approve is not None and P.sender_hold in d.clears


def test_backoff_after_abort_keeps_standoff():
    lg = DetourLogic(P)
    side = lg.step(inputs([car(FAR)], t=T0)).approve
    other = RIGHT if side == LEFT else LEFT
    lg.step(inputs([car(FAR, t=T0 + 1.0)], cands={side: cand(state='ABORTING'), other: cand()}, t=T0 + 1.0))
    d = lg.step(inputs([car(FAR, t=T0 + 2.0)], t=T0 + 2.0))
    assert d.state == HOLD and P.sender_hold in dict(d.limits)
    assert P.sender_hold not in d.clears


def test_right_of_preferred_lane_only_left_allowed():
    d = DetourLogic(P).step(inputs([car(FAR)], role='right_of_preferred'))
    assert d.approve != RIGHT


def test_abort_backoff_then_retry():
    lg = DetourLogic(P)
    t = T0
    side = lg.step(inputs([car(FAR)], t=t)).approve
    other = RIGHT if side == LEFT else LEFT
    aborting = {side: cand(state='ABORTING'), other: cand()}
    lg.step(inputs([car(FAR, t=t + 1.0)], cands=aborting, t=t + 1.0))
    d = lg.step(inputs([car(FAR, t=t + 2.0)], t=t + 2.0))
    assert d.approve is None and d.state == HOLD          # back-off 중엔 대기
    d = lg.step(inputs([car(FAR, t=t + 1.0 + P.abort_backoff_s + 0.1)],
                       t=t + 1.0 + P.abort_backoff_s + 0.1))
    assert d.approve is not None                          # 이후 재시도


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
    lg = DetourLogic(P)
    lg.step(inputs([ped(30.0, 2.0)], t=T0))
    assert P.sender_ped not in lg.step(inputs([], t=T0 + 0.5)).clears
    assert P.sender_ped in lg.step(inputs([], t=T0 + P.ped_release_delay_s + 0.1)).clears


def test_hold_and_pedestrian_limits_coexist():
    d = DetourLogic(P).step(inputs([car(FAR), ped(30.0, 2.0)], cands=UNSAFE))
    lim = dict(d.limits)
    assert P.sender_hold in lim and P.sender_ped in lim


# ---------------------------------------------------------------- 리스폰 / stale
def test_respawn_clears_limits_resets_and_holds_zero():
    lg = DetourLogic(P)
    lg.step(inputs([car(FAR)], cands=UNSAFE, t=T0))
    d = lg.step(inputs([], respawn=True, t=T0 + 1.0))
    assert dict(d.limits).get(P.sender_hold) == 0.0
    d = lg.step(inputs([], t=T0 + 1.0 + P.respawn_hold_s + 0.1))
    assert P.sender_hold in d.clears and d.state == FOLLOW


def test_stale_inputs_no_approval_and_keep_hold():
    lg = DetourLogic(P)
    lg.step(inputs([car(FAR)], cands=UNSAFE, t=T0))
    d = lg.step(inputs([car(FAR)], stale=True, t=T0 + 0.2))
    assert d.approve is None
    assert dict(d.limits).get(P.sender_hold) == 0.0


# ---------------------------------------------------------------- 관측성
def test_detail_carries_decision_basis():
    d = DetourLogic(P).step(inputs([car(FAR)]))
    assert d.detail.get('blocker', {}).get('dist') == FAR
    assert 'cand_sides' in d.detail.get('struct', {})
    assert d.detail.get('standoff_missed') is False

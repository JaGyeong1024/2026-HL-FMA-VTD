"""판단 노드 순수 로직 하네스 — 9/7 정책 교체판.

정책: 자차 차로 전방 정지차를 보면 신호와 무관하게 동일 처리 →
  우회 가능 거리(standoff_m)를 남기고 감속·정지 → **서서 관찰** →
  적색이면 대기, 아니면 무응답 no_response_s 후 우회 결정 → 후보 safe & finish+여유<거리 면 승인.
시간 압박(인지 80 m / 1.5 초 예산)이 사라지는 것이 이 설계의 요점이다.
"""
import dataclasses

import pytest

from vtd_autoware_bridge.detour_logic import (
    DetourLogic, Params, EgoState, ObjectInfo, LaneInfo, CandidateInfo, Inputs,
    FOLLOW, ASSESS, HOLD, LANE_CHANGE, STOP, LEFT, RIGHT,
)

P = Params()
PE = dataclasses.replace(P, return_check_enforce=True)   # 복귀 여유 부족을 veto 하는 모드
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


def lane_at_stopline(front_d=None, gap=5.0, **kw):
    """막힘 앞끝이 정지선에서 gap 만큼 앞 → 복귀 불가(gap < need_m())."""
    return lane(stopline=(FAR if front_d is None else front_d) + gap, **kw)


def cand(safe=True, state='WAITING', start=6.0, finish=25.0, present=True, stale=False):
    return CandidateInfo(present=present, safe=safe, state=state,
                         start_distance_m=start, finish_distance_m=finish, stale=stale)


def inputs(objects=(), lane_=None, cands=None, t=T0, v=12.0, role='preferred',
           respawn=False, stale=False):
    return Inputs(t=t, ego=EgoState(t=t, v=v, lane_role=role), objects=list(objects),
                  lane=lane_ or lane(),
                  candidates=cands if cands is not None else {LEFT: cand(), RIGHT: cand()},
                  respawn=respawn, stale=stale)


def need_m(n=1):
    """복귀에 필요한 길이 = n칸 × lc_length(approach_speed) + 정지 여유 (하한 stopline_min_distance_m)."""
    lg = DetourLogic(P)
    return max(P.stopline_min_distance_m, n * lg.lc_length_m(P.approach_speed_mps) + P.stopline_stop_margin_m)


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


def test_standoff_missed_keeps_decelerating_instead_of_giving_up():
    """9/7 detour14: standoff 를 못 만든다고 손을 놓아 앞차 10.7 m 뒤에서 99 초를 멈춰 있었다.
    이제는 최대한 감속해 남은 거리를 지키되 0 으로 수렴시키지 않는다."""
    near = P.standoff_needed_fallback_m - 10.0
    d = DetourLogic(P).step(inputs([car(near)], cands=NONE_C))
    assert d.detail.get('standoff_missed') is True
    assert d.state == HOLD                                  # STOP 으로 손 놓지 않는다
    assert dict(d.limits).get(P.sender_hold) == pytest.approx(P.approach_speed_floor_mps)
    assert P.sender_hold not in d.clears


def test_needed_distance_uses_candidate_finish_not_profile_target():
    """판정 기준은 프로파일 목표(EFF)가 아니라 '우회에 실제로 필요한 거리'(후보 finish + 여유)다.
    9/7 detour14: 70 m 인지인데 목표 73 m 기준으로 포기했다 — 그때 후보 finish 는 38 m 였다."""
    short = {LEFT: cand(present=False), RIGHT: cand(safe=True, state='WAITING', start=5.0, finish=38.0)}
    lg = DetourLogic(P)
    assert lg.needed_distance_m(inputs([], cands=short)) == pytest.approx(38.0 + P.lc_finish_margin_m)
    d = lg.step(inputs([car(70.0)], cands=short))           # 70 m > 필요 41 m → 포기하지 않는다
    assert d.detail.get('standoff_missed') is False
    assert d.state in (HOLD, LANE_CHANGE)


def test_needed_distance_falls_back_without_candidate():
    lg = DetourLogic(P)
    assert lg.needed_distance_m(inputs([], cands=NONE_C)) == pytest.approx(P.standoff_needed_fallback_m)


def test_standoff_kept_once_established_even_when_close():
    """standoff 를 만들 수 있었던 blocker 는 가까워져도 계속 우리가 관리한다."""
    lg = DetourLogic(P)
    lg.step(inputs([car(FAR, stopped_for=0.3)], t=T0))
    d = lg.step(inputs([car(EFF + 1.0, stopped_for=1.0, t=T0 + 1.0)], t=T0 + 1.0))
    assert d.state == HOLD and d.detail.get('standoff_missed') is False


# ------------------------------------------- 대기열 판정 (신호를 보지 않는다, 두 분기)
def test_queue_no_room_to_stopline_waits():
    """막힘 앞끝이 정지선에 붙어 있으면 끼어들 공간이 없다 → 대기.
    (신호 대기 중인 차들이 바로 이 모습이다. 신호를 보지 않아도 결과가 같다.)"""
    d = DetourLogic(PE).step(inputs([car()], lane_=lane_at_stopline()))
    q = d.detail['queue']
    assert q['basis'] == 'no_room_to_stopline' and q['verdict'] == 'wait'
    assert d.state == HOLD and '공간 없음' in d.reason
    assert P.sender_hold in dict(d.limits)          # standoff 는 유지한다


def test_queue_room_available_detours():
    """복귀에 필요한 길이만큼 여유가 있으면 신호와 무관하게 우회 대상이다."""
    d = DetourLogic(P).step(inputs([car()], lane_=lane(stopline=FAR + need_m() + 5.0)))
    q = d.detail['queue']
    assert q['basis'] == 'room_available' and q['verdict'] == 'detour'
    assert d.state == LANE_CHANGE and d.approve in (LEFT, RIGHT)


def test_return_need_scales_with_lane_count():
    """복귀 칸 수가 늘면 필요 길이도 그만큼 늘어난다(상수가 아니다)."""
    two = lane(stopline=FAR + need_m(1) + 5.0)
    two.lanes_to_preferred = {'right': 2}
    d = DetourLogic(PE).step(inputs([car()], lane_=two))
    assert d.detail['return']['n_lanes'] == 2
    assert d.detail['return']['need_m'] == pytest.approx(need_m(2), abs=0.2)
    assert d.state == HOLD          # 1칸 기준으로는 충분했지만 2칸에는 부족
    wide = lane(stopline=FAR + need_m(2) + 5.0)
    wide.lanes_to_preferred = {'right': 2}
    assert DetourLogic(PE).step(inputs([car()], lane_=wide)).state == LANE_CHANGE


def test_return_detail_fields():
    d = DetourLogic(P).step(inputs([car()], lane_=lane(stopline=FAR + need_m() + 5.0)))
    r = d.detail['return']
    assert set(r) == {'n_lanes', 'lc_length_model_m', 'need_m', 'gap_m', 'verdict'}
    assert r['lc_length_model_m'] == pytest.approx(DetourLogic(P).lc_length_m(P.approach_speed_mps), abs=0.1)


def test_lc_length_grows_with_speed():
    lg = DetourLogic(P)
    assert lg.lc_length_m(2.0) < lg.lc_length_m(4.0) < lg.lc_length_m(8.0)
    assert lg.shift_time_s() == pytest.approx(4.31, abs=0.05)   # 폭 3.5, jerk 1.5, acc 1.2


def test_queue_no_stopline_detours():
    d = DetourLogic(P).step(inputs([car()], lane_=lane(stopline=None)))
    assert d.detail['queue']['basis'] == 'no_stopline'
    assert d.state == LANE_CHANGE


def test_queue_boundary_at_threshold_allows_detour():
    """경계는 gap >= need 에서 통과한다. 정확히 need 를 쓰면 부동소수 오차로 갈리므로 1 mm 를 더한다
    (기하가 아니라 부동소수 문제다 — 판정식은 그대로 두고 테스트만 경계를 비켜 쓴다)."""
    d = DetourLogic(P).step(inputs([car()], lane_=lane(stopline=FAR + need_m() + 0.001)))
    assert d.detail['queue']['basis'] == 'room_available'
    assert d.detail['return']['gap_m'] >= d.detail['return']['need_m']


def test_queue_just_below_threshold_waits():
    d = DetourLogic(PE).step(inputs([car()], lane_=lane(stopline=FAR + need_m() - 1.0)))
    assert d.detail['queue']['basis'] == 'no_room_to_stopline'


def test_queue_gap_uses_frontmost_stopped_vehicle():
    """간격은 가장 가까운 blocker 가 아니라 **정지 차량군 중 가장 앞선 것** 기준이다."""
    near = car(56.0)
    front = ObjectInfo(id=2, cls='CAR', v=0.0, longitudinal_m=79.0, lateral_m=0.0,
                       stopped_since=T0 - LONG)          # 감지 한계 80 m 안
    sl = lane(stopline=84.0)                             # 앞선 차 기준 5 m (가까운 차 기준이면 28 m)
    d = DetourLogic(PE).step(inputs([near, front], lane_=sl))
    q = d.detail['queue']
    assert q['front_blocker_to_stopline_m'] == pytest.approx(5.0)
    assert q['basis'] == 'no_room_to_stopline'


def test_queue_verdict_has_no_signal_dependency():
    """판정 입력에 신호가 없다 — 같은 기하면 항상 같은 답이다."""
    lg = DetourLogic(PE)
    for _ in range(3):
        d = lg.step(inputs([car()], lane_=lane_at_stopline()))
        assert d.detail['queue']['basis'] == 'no_room_to_stopline'
    assert 'signal' not in d.detail and 'red_queue' not in d.detail


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


def test_regulatory_proximity_does_not_veto():
    """규제요소 근접은 Autoware 가 isLaneChangeRequired(scene.cpp:296-342)에서 이미 검사한다.
    가까우면 후보를 아예 안 만들므로, **후보가 있다는 사실이 통과의 증거**다.
    우리가 베낀 값으로 중복 검사하면 값이 어긋날 때 멀쩡한 우회를 막는다
    (9/7 detour16: 우리 54.4 m 기준으로 48 m 를 거부했는데 Autoware 는 후보를 내고 있었다)."""
    d = DetourLogic(P).step(inputs([car(FAR)], lane_=lane(reg=1.0)))   # 규제요소 코앞
    assert d.state == LANE_CHANGE and d.approve in (LEFT, RIGHT)
    assert d.detail['reg_m'] == pytest.approx(1.0)                     # 관측치로는 남는다


def test_no_candidate_still_holds_regardless_of_map_values():
    """맵 기반 값(이웃·실선·규제요소)은 어떤 조합이어도 veto 가 아니다. 후보 유무가 판단한다."""
    hostile = lane(left=False, right=False, reg=1.0, left_solid=True, right_solid=True)
    d = DetourLogic(P).step(inputs([car(FAR)], lane_=hostile, cands=NONE_C))
    assert d.state == HOLD                                   # 후보가 없어서 대기일 뿐
    d = DetourLogic(P).step(inputs([car(FAR)], lane_=hostile))
    assert d.state == LANE_CHANGE and d.approve is not None   # 후보가 있으면 승인


def test_stopline_gap_below_threshold_keeps_standoff():
    d = DetourLogic(PE).step(inputs([car(FAR)],
                                   lane_=lane(stopline=FAR + P.stopline_min_distance_m - 1.0)))
    assert d.state == HOLD and d.approve is None
    assert P.sender_hold in dict(d.limits)


# ------------------------------------------- standoff 유지 원칙 (9/7 detour6)
@pytest.mark.parametrize('name,kw', [
    ('끼어들 공간 없음', dict(lane_=lane_at_stopline())),
    ('후보 없음', dict(cands=NONE_C)),
    ('후보 unsafe', dict(cands=UNSAFE)),
    ('후보 stale', dict(cands={LEFT: cand(stale=True), RIGHT: cand(stale=True)})),
    ('LC 길이 부족', dict(cands={LEFT: cand(finish=98.5), RIGHT: cand(finish=98.5)})),
    ('복귀 방향뿐', dict(role='right_of_preferred', cands=NONE_C)),
])
def test_cannot_detour_now_keeps_standoff_limit(name, kw):
    """'지금은 불가' 는 제한을 푸는 이유가 아니다. 풀면 blocker 코앞까지 기어가 우회 여유를 잃는다."""
    d = DetourLogic(PE).step(inputs([car(FAR)], **kw))
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


# ------------------------------------------- 복귀 검사: 기본은 경고, enforce 시 veto
def test_return_insufficient_warns_but_allows_by_default():
    """모델이 실측 대비 과대(13.7 m/s 에서 118 vs 99)하고, 복귀 실패 비용(리스폰 1회)이 미완주보다 작다.
    따라서 기본값은 막지 않고 경고만 남긴다."""
    tight = lane(stopline=FAR + 5.0)                     # 복귀 여유가 한참 부족
    d = DetourLogic(P).step(inputs([car()], lane_=tight))
    assert d.state == LANE_CHANGE and d.approve in (LEFT, RIGHT)   # 막지 않는다
    assert d.detail['return']['verdict'] == 'insufficient'
    assert d.warn and '복귀 여유 부족' in d.warn


def test_return_insufficient_vetoes_when_enforced():
    tight = lane(stopline=FAR + 5.0)
    d = DetourLogic(PE).step(inputs([car()], lane_=tight))
    assert d.state == HOLD and d.approve is None
    assert d.detail['return']['verdict'] == 'wait'


def test_return_sufficient_has_no_warning():
    d = DetourLogic(P).step(inputs([car()], lane_=lane(stopline=FAR + need_m() + 5.0)))
    assert d.detail['return']['verdict'] == 'detour'
    assert d.warn is None


def test_return_status_fields_for_calibration():
    """다음 주행에서 모델값과 실제 복귀 성공 여부를 대조하기 위한 관측 필드."""
    d = DetourLogic(P).step(inputs([car()], lane_=lane(stopline=FAR + 5.0)))
    r = d.detail['return']
    assert set(r) == {'n_lanes', 'lc_length_model_m', 'need_m', 'gap_m', 'verdict'}
    assert r['gap_m'] == pytest.approx(5.0)

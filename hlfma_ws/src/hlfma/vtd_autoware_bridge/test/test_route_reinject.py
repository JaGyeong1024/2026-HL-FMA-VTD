"""route_node A13(리스폰 재주입) 하네스: 지나온 점 필터 + 재주입 절차."""
from vtd_autoware_bridge.route_reinject import (
    MatchedPoint, filter_passed_points, Reinjector, ServiceResult,
)

# 경로 세그먼트 열: 세그먼트 k = [preferred, 이웃...]
SEGS = [[100], [101, 111], [102, 112], [103], [104], [105]]
PTS = [
    MatchedPoint(0, 100, 5.0),    # 시작
    MatchedPoint(1, 101, 20.0),   # 교차로 1 진입
    MatchedPoint(2, 112, 10.0),   # 교차로 1 진출 (이웃 lanelet 에 매칭)
    MatchedPoint(3, 104, 30.0),
    MatchedPoint(4, 105, 40.0),   # 종료
]


def test_keeps_points_after_ego_segment():
    assert filter_passed_points(PTS, SEGS, ego_segment_idx=2, ego_s=5.0) == [2, 3, 4]


def test_same_segment_compares_s():
    assert filter_passed_points(PTS, SEGS, ego_segment_idx=2, ego_s=15.0) == [3, 4]


def test_respawn_backwards_reincludes_points():
    # 자차가 세그먼트 4 까지 갔다가 세그먼트 0 시작점으로 리스폰
    assert filter_passed_points(PTS, SEGS, ego_segment_idx=0, ego_s=0.0) == [0, 1, 2, 3, 4]


def test_start_point_behind_ego_dropped():
    assert filter_passed_points(PTS, SEGS, ego_segment_idx=0, ego_s=10.0) == [1, 2, 3, 4]


def test_last_point_always_kept():
    assert filter_passed_points(PTS, SEGS, ego_segment_idx=5, ego_s=45.0) == [4]


def test_unmatched_lanelet_kept_conservatively():
    pts = PTS + [MatchedPoint(5, 999, 0.0)]
    kept = filter_passed_points(pts, SEGS, ego_segment_idx=4, ego_s=0.0)
    assert 5 in kept and 4 in kept


# ---------------------------------------------------------------- 재주입 절차
class FakeServices:
    def __init__(self, results):
        self.results = dict(results)
        self.calls = []

    def __call__(self, name, req):
        self.calls.append(name)
        return self.results.get(name, ServiceResult(True))


def test_sequence_stop_change_autonomous():
    svc = FakeServices({})
    r = Reinjector(svc)
    assert r.run(change_request=object()) is True
    assert svc.calls == [Reinjector.STOP, Reinjector.CHANGE, Reinjector.AUTO]
    assert r.attempts == 1


def test_change_failure_stops_before_autonomous():
    svc = FakeServices({Reinjector.CHANGE: ServiceResult(False, 3, 'route_is_not_set')})
    r = Reinjector(svc)
    assert r.run(change_request=object()) is False
    assert svc.calls == [Reinjector.STOP, Reinjector.CHANGE]


def test_stop_failure_aborts_immediately():
    svc = FakeServices({Reinjector.STOP: ServiceResult(False, 1, 'x')})
    r = Reinjector(svc)
    assert r.run(change_request=object()) is False
    assert svc.calls == [Reinjector.STOP]


def test_respawn_event_resets_attempts():
    svc = FakeServices({Reinjector.CHANGE: ServiceResult(False)})
    r = Reinjector(svc)
    r.run(object()); r.run(object()); r.run(object())
    assert r.attempts == 3
    r.on_respawn()
    assert r.attempts == 0

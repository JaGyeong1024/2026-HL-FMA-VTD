"""리스폰 재주입(A13) 순수 로직 — route_node 가 사용. ROS 의존 없음.

  filter_passed_points : 현재 경로(mission_planner /planning/mission_planning/route 세그먼트 열) 기준으로
                         자차가 이미 지나온 CSV 점을 제외한다.
  Reinjector           : change_to_stop → change_route_points → change_to_autonomous 절차.
                         서비스 클라이언트는 주입(테스트에서 가짜 객체).

★ 인터페이스 스텁 — 구현 전. 테스트(test/test_route_reinject.py)가 계약을 정의한다.
"""
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence


@dataclass
class MatchedPoint:
    idx: int            # CSV seq 순서 인덱스 (0-based)
    lanelet_id: int     # 매칭된 lanelet
    s: float            # 그 lanelet 안의 호길이


def filter_passed_points(points: Sequence[MatchedPoint], route_segments: Sequence[Sequence[int]],
                         ego_segment_idx: int, ego_s: float) -> List[int]:
    """남길 CSV 점 idx 목록(오름차순).
    - route_segments[k] = k 번째 세그먼트의 lanelet id 들(preferred + 이웃)
    - 점의 lanelet 이 속한 세그먼트 인덱스 > ego_segment_idx 이면 유지
    - 같은 세그먼트면 s > ego_s 일 때 유지
    - 경로 세그먼트에 없는 lanelet(매칭 실패)은 유지(보수적)
    - 마지막 점(종료점)은 항상 유지
    """
    raise NotImplementedError('route_reinject.filter_passed_points — 구현 전 (하네스 red)')


@dataclass
class ServiceResult:
    success: bool
    code: int = 0
    message: str = ''


class Reinjector:
    """call(service_name, request) -> ServiceResult 를 주입받아 절차를 수행한다."""
    STOP = '/api/operation_mode/change_to_stop'
    CHANGE = '/api/routing/change_route_points'
    AUTO = '/api/operation_mode/change_to_autonomous'

    def __init__(self, call: Callable[[str, object], ServiceResult], log: Optional[Callable[[str], None]] = None):
        self.call = call
        self.log = log or (lambda m: None)
        self.attempts = 0

    def on_respawn(self) -> None:
        """리스폰 이벤트: attempts 리셋."""
        raise NotImplementedError('route_reinject.Reinjector.on_respawn — 구현 전 (하네스 red)')

    def run(self, change_request: object) -> bool:
        """STOP → CHANGE → AUTO 순서로 호출. 어느 단계든 실패하면 즉시 False 로 중단(다음 단계 호출 금지).
        attempts 는 run 호출마다 +1."""
        raise NotImplementedError('route_reinject.Reinjector.run — 구현 전 (하네스 red)')

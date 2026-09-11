"""리스폰 재주입(A13) 순수 로직 — route_node 가 사용. ROS 의존 없음.

  filter_passed_points : 현재 경로(mission_planner /planning/mission_planning/route 세그먼트 열) 기준으로
                         자차가 이미 지나온 CSV 점을 제외한다.
  Reinjector           : change_to_stop → change_route_points → change_to_autonomous 절차.
                         서비스 클라이언트는 주입(테스트에서 가짜 객체).

근거 (adv_a13_scripts.md):
  D-1 set_route_points 는 SET 상태에서 무조건 거부 → change_route_points(/api/routing/change_route_points, 타입 SetRoutePoints).
  D-2 AUTONOMOUS 중 재라우팅은 승인 모듈(LC·회피) 잔존 시 거부 → change_to_stop 으로 먼저 빠져나온 뒤 재라우팅,
      정지 상태 engage 는 즉시 통과(allow_autonomous_in_stopped true, check_engage_condition false).
  D-4 지나온 CSV 점을 그대로 넣으면 뒤로 가는 경로/루프 → 세그먼트 열 기준 필터.
"""
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence


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
    seg_of: Dict[int, int] = {}
    for k, seg in enumerate(route_segments):
        for lid in seg:
            seg_of.setdefault(int(lid), k)
    pts = list(points)
    if not pts:
        return []
    last_idx = max(pt.idx for pt in pts)
    kept = []
    for pt in pts:
        if pt.idx == last_idx:
            kept.append(pt.idx)
            continue
        k = seg_of.get(int(pt.lanelet_id))
        if k is None:                       # 경로에 없는 lanelet → 보수적으로 유지
            kept.append(pt.idx)
        elif k > ego_segment_idx:
            kept.append(pt.idx)
        elif k == ego_segment_idx and pt.s > ego_s:
            kept.append(pt.idx)
    return sorted(set(kept))


@dataclass
class ServiceResult:
    success: bool
    code: int = 0
    message: str = ''


class Reinjector:
    """call(service_name, request) -> ServiceResult 를 주입받아 절차를 수행한다.

    request 는 CHANGE 단계에만 change_request 를 넘기고, STOP/AUTO 는 None 을 넘긴다
    (호출자가 서비스 타입에 맞는 빈 요청을 만든다).
    """
    STOP = '/api/operation_mode/change_to_stop'
    CHANGE = '/api/routing/change_route_points'
    AUTO = '/api/operation_mode/change_to_autonomous'

    def __init__(self, call: Callable[[str, object], ServiceResult], log: Optional[Callable[[str], None]] = None):
        self.call = call
        self.log = log or (lambda m: None)
        self.attempts = 0

    def on_respawn(self) -> None:
        """리스폰 이벤트: attempts 리셋 (이벤트마다 새로 시도)."""
        self.attempts = 0

    def run(self, change_request: object) -> bool:
        """STOP → CHANGE → AUTO 순서로 호출. 어느 단계든 실패하면 즉시 False 로 중단(다음 단계 호출 금지).
        attempts 는 run 호출마다 +1."""
        self.attempts += 1
        for name, req in ((self.STOP, None), (self.CHANGE, change_request), (self.AUTO, None)):
            r = self.call(name, req)
            if r is None or not r.success:
                code = getattr(r, 'code', None)
                msg = getattr(r, 'message', '')
                self.log(f'재주입 실패 (attempt {self.attempts}) {name}: code={code} {msg}')
                return False
            self.log(f'재주입 {name}: 성공')
        return True

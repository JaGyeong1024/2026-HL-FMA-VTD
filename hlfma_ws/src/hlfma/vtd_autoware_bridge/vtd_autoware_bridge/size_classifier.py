"""VTD objects 크기(L×W×H) → 객체 분류.

VTD TCP 9910 objects 에는 종류가 없어 크기로 가른다. 규칙과 경계값의 근거는
docs/대회정보/object_bbox_based_classification.md (9/12 VTD 실측 11종). 경계값은 실측 사이 빈 구간의 중간이다.
규칙은 위에서부터 차례로 적용한다. 값은 브리지 파라미터 object_class.* 로 바꿀 수 있다.

9/1 첫 브리지부터 쓰던 규칙(L<1.2·W<1.2 → 보행자, L<2.8 → 이륜, 그 외 차량)은 라바콘을 보행자로
분류해 road_user_stop 이 풀리지 않는 정지를 걸었다(9/12 교착). 되돌리려면 classify() 를 그 세 줄로 바꾼다.
"""
from dataclasses import dataclass

UNKNOWN = 'unknown'
CAR = 'car'
MOTORCYCLE = 'motorcycle'  # 현재 미사용 (이륜은 BICYCLE 로 낸다, 아래 규칙 3)
BICYCLE = 'bicycle'
PEDESTRIAN = 'pedestrian'


@dataclass(frozen=True)
class SizeThresholds:
    static_max_height: float = 0.6     # [m] 이보다 낮으면 정적 장애물. 라바콘 0.32 / 빈 휠체어 0.92
    car_min_length: float = 3.5        # [m] 이 이상이면 차량. 승용차 4.4
    two_wheel_min_length: float = 1.5  # [m] 이륜 길이 하한. 휠체어 1.01 / 자전거 1.90
    two_wheel_max_length: float = 2.8  # [m] 이륜 길이 상한. 오토바이 2.0 / 승용차 4.4
    ped_max_length: float = 1.3        # [m] 보행자 길이 상한. 휠체어 탑승 1.01 / 자전거 1.90
    ped_max_width: float = 0.8         # [m] 보행자 폭 상한. 성인 남성 0.70
    ped_min_height: float = 1.2        # [m] 보행자 높이 하한. 빈 자전거 1.10 / 남자 어린이 1.35


def classify(length, width, height, th=SizeThresholds()):
    """크기로 분류해 UNKNOWN / CAR / MOTORCYCLE / PEDESTRIAN 중 하나를 돌려준다."""
    if height < th.static_max_height:
        return UNKNOWN      # 1. 낮은 정지물 (라바콘)
    if length >= th.car_min_length:
        return CAR          # 2. 차량
    if th.two_wheel_min_length <= length <= th.two_wheel_max_length:
        # 3. 자전거·오토바이. 크기로는 둘을 못 가른다(자전거 1.90x0.70x1.80 / 오토바이 2.0x0.6x1.7).
        #    9/12 결정: BICYCLE 로 낸다. 시나리오의 이륜은 자전거이고, 자전거 라벨이어야
        #    도로이용자 정지(road_user_stop)·큰 회피 여유·접근 감속·직선 예측이 걸린다
        #    (MOTORCYCLE 이면 셋 다 안 걸려 03:24 bag 에서 튀어나온 자전거를 42 km/h 로 0.4 m 지나쳤다).
        #    달리는 이륜차는 BICYCLE 라벨로도 obstacle_cruise 추종 대상이라 추종·추월 거동은 같다.
        #    되돌리려면 MOTORCYCLE 을 돌려준다.
        return BICYCLE
    if length <= th.ped_max_length and width <= th.ped_max_width and height >= th.ped_min_height:
        return PEDESTRIAN   # 4. 어린이~성인, 휠체어 탑승자
    return UNKNOWN          # 5. 그 외 (빈 휠체어 등)

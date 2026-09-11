"""size_classifier: docs/대회정보/object_bbox_based_classification.md 실측 11종이 기대 분류로 가는지."""
import pytest

from vtd_autoware_bridge.size_classifier import (
    CAR, BICYCLE, PEDESTRIAN, UNKNOWN, SizeThresholds, classify,
)

# (이름, L, W, H, 기대 분류) — 문서 §1 실측값
MEASURED = [
    ('성인 남성', 0.60, 0.70, 1.80, PEDESTRIAN),
    ('성인 여성', 0.55, 0.63, 1.62, PEDESTRIAN),
    ('남자 어린이', 0.50, 0.60, 1.35, PEDESTRIAN),
    ('여자 어린이', 0.50, 0.60, 1.42, PEDESTRIAN),
    ('자전거 탄 사람', 1.90, 0.70, 1.80, BICYCLE),
    ('빈 자전거', 1.90, 0.65, 1.10, BICYCLE),
    ('휠체어 탄 사람', 1.01, 0.70, 1.80, PEDESTRIAN),
    ('빈 휠체어', 1.01, 0.62, 0.92, UNKNOWN),
    ('오토바이(탑승)', 2.0, 0.6, 1.7, BICYCLE),
    ('승용차', 4.4, 1.8, 1.4, CAR),
    ('라바콘', 0.30, 0.30, 0.32, UNKNOWN),
]


@pytest.mark.parametrize('name,length,width,height,expected', MEASURED, ids=[m[0] for m in MEASURED])
def test_measured_objects(name, length, width, height, expected):
    assert classify(length, width, height) == expected


def test_unmeasured_gaps_fall_to_unknown():
    # 규칙 5: 이륜과 차량 사이(2.8 < L < 3.5), 보행자와 이륜 사이(1.3 < L < 1.5)
    assert classify(3.2, 1.7, 1.5) == UNKNOWN
    assert classify(1.4, 0.7, 1.7) == UNKNOWN


def test_thresholds_are_overridable():
    # 브리지 파라미터 object_class.* 로 경계를 옮기면 결과가 따라 바뀐다
    assert classify(3.2, 1.7, 1.5, SizeThresholds(car_min_length=3.0)) == CAR
    assert classify(1.01, 0.62, 0.92, SizeThresholds(ped_min_height=0.9)) == PEDESTRIAN

#!/usr/bin/env bash
# 실기 모드 토픽 계약 점검: "구독자는 있는데 발행자가 없는 토픽"을 나열한다.
# 브리지가 Autoware가 기대하는 이름으로 발행하는지 확인하는 용도 (개발계획_0902 §1 R2).
# 사용: ./start_autoware.sh mock (또는 실기) 을 띄운 뒤 30초쯤 후에  bash tools/check_topic_contract.sh
# 출력이 비어 있으면 계약 충족. /parameter_events, /rosout 등 ROS 기본 토픽은 제외.
set -o pipefail
source /opt/ros/jazzy/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/hlfma_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"

echo "== 발행자 없는 구독 토픽 (ROS_DOMAIN_ID=$ROS_DOMAIN_ID) =="
n=0
while read -r topic; do
  case "$topic" in /parameter_events|/rosout|/clock|/tf_static|/diagnostics|/initialpose*) continue;; esac
  # 선택적·디버그 토픽 (발행자 없어도 동작에 무관): rviz 입력, 비활성 모듈의 planning_factors, 처리시간·지연 디버그, 미사용 인터페이스
  case "$topic" in
    */debug/*|*processing_time*|*latency*|/joint_states|/control/command/actuation_cmd|/planning/mission_planning/goal|/planning/mission_planning/checkpoint|/planning/planning_factors/*|*/virtual_wall*|*/markers*|/api/external/*|/remote/*|/planning/scenario_planning/parking/*|*/freespace*) continue;;
    # 선택 입력 (rviz 패널·외부 API·미장착 센서·차량 부가장치): 없어도 플래닝·제어에 영향 없음 (9/2 mock 회귀에서 확인)
    /rviz/*|*/input/lateral_offset|/planning/scenario_planning/max_velocity_default|/sensing/imu/*|/system/driving_mode/*|/system/hazard_lights_cmd|/system/mrm/pull_over_manager/*|/vehicle/calibration/*|/vehicle/doors/*|/vehicle/status/battery_charge) continue;;
  esac
  info="$(ros2 topic info "$topic" 2>/dev/null)"
  pubs="$(echo "$info" | sed -n 's/^Publisher count: //p')"
  subs="$(echo "$info" | sed -n 's/^Subscription count: //p')"
  if [ "${pubs:-0}" = "0" ] && [ "${subs:-0}" != "0" ]; then
    echo "  $topic  (구독 $subs)"
    n=$((n+1))
  fi
done < <(ros2 topic list 2>/dev/null)
echo "== 총 $n 개 =="
[ "$n" -eq 0 ] && echo "계약 충족: 모든 구독 토픽에 발행자 있음"
exit 0

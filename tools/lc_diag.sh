#!/usr/bin/env bash
# 차선변경 진단 — 스택 실행 중(engage 후) 실행. 차선변경 판단·제어 신호를 한 번에 뽑는다.
# usage: bash tools/lc_diag.sh
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/hlfma_ws/install/setup.bash" 2>/dev/null
BP=/planning/scenario_planning/lane_driving/behavior_planning
echo "════════ 1. 차선변경 관여 노드 (실행 중인 것) ════════"
ros2 node list 2>/dev/null | grep -iE "behavior_path|behavior_velocity|motion_velocity|path_optimizer|velocity_smoother|planning_validator|scenario_selector|trajectory_follower|vehicle_cmd_gate|operation_mode" | sort

echo; echo "════════ 2. 차선변경 관련 토픽 ════════"
ros2 topic list 2>/dev/null | grep -iE "lane_change|planning_factors|cooperate|behavior_planning/path|turn_indicators|steering" | sort

echo; echo "════════ 3. ego 상태 ════════"
timeout 3 ros2 topic echo --once /localization/kinematic_state 2>/dev/null | grep -A3 -E "position:|twist:" | grep -E "x:|y:" | head -3

echo; echo "════════ 4. 판단: 차선변경 의도(방향지시등) ════════"
echo "[behavior_path 출력 turn_indicators]"; timeout 3 ros2 topic echo --once "$BP/behavior_path_planner/output/turn_indicators_cmd" 2>/dev/null | grep command
echo "[gate 최종 turn_indicators (→VTD)]"; timeout 3 ros2 topic echo --once /control/command/turn_indicators_cmd 2>/dev/null | grep command
echo "  (command: 1=DISABLE, 2=LEFT, 3=RIGHT — LEFT/RIGHT면 차선변경 시도 중)"

echo; echo "════════ 5. 판단: 차선변경 planning_factor (사유) ════════"
for t in $(ros2 topic list 2>/dev/null | grep -iE "planning_factors/lane_change|cooperate_status/lane_change"); do
  echo "-- $t"; timeout 3 ros2 topic echo --once "$t" 2>/dev/null | grep -E "behavior|distance|status|safe|reason|cooperate" | head -6
done

echo; echo "════════ 6. behavior_path 출력 경로 길이(끊김 확인) ════════"
timeout 4 ros2 topic echo --once "$BP/path" 2>/dev/null | python3 -c "
import sys,re,math
b=sys.stdin.read()
pts=re.findall(r'x:\s*([-\d.e]+)\s*\n\s*y:\s*([-\d.e]+)',b)
if pts:
  s=0; px,py=float(pts[0][0]),float(pts[0][1])
  for x,y in pts: x,y=float(x),float(y); s+=math.hypot(x-px,y-py); px,py=x,y
  print(f'  경로 점 {len(pts)}개, 끝점까지 {s:.1f}m  {\"⚠짧음(차선변경에서 끊김 의심)\" if s<60 else \"\"}')
else: print('  경로 비어있음')
"

echo; echo "════════ 7. 제어: control_cmd (조향/속도/가속) ════════"
echo "[trajectory_follower 원출력]"; timeout 3 ros2 topic echo --once /control/trajectory_follower/control_cmd 2>/dev/null | grep -E "steering_tire_angle|velocity|acceleration" | head -4
echo "[gate 최종 (→VTD)]"; timeout 3 ros2 topic echo --once /control/command/control_cmd 2>/dev/null | grep -E "steering_tire_angle|velocity|acceleration" | head -4
echo "  (steering_tire_angle 이 계속 ~0 이고 velocity=0 이면 횡방향 기동 없음 = 차선변경 미시도)"

echo; echo "════════ 8. 최종 궤적 정지점 ════════"
timeout 4 ros2 topic echo --once /planning/trajectory 2>/dev/null | python3 -c "
import sys,re,math
b=sys.stdin.read()
pts=re.findall(r'x:\s*([-\d.e]+)\s*\n\s*y:\s*([-\d.e]+)',b); vs=re.findall(r'longitudinal_velocity_mps:\s*([-\d.e]+)',b)
if pts and vs:
  z=next((i for i,v in enumerate(vs) if float(v)<0.1),None)
  if z is not None:
    s=0; px,py=float(pts[0][0]),float(pts[0][1])
    for i in range(min(z+1,len(pts))): x,y=float(pts[i][0]),float(pts[i][1]); s+=math.hypot(x-px,y-py); px,py=x,y
    print(f'  정지점: 궤적 {z}번째 점, ego로부터 {s:.1f}m')
  else: print('  궤적에 정지점 없음(속도 유지)')
"

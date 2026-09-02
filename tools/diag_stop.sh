#!/usr/bin/env bash
# 정지 사유 진단: mock + 실기 구성 기동 → 경로 → engage → 상태 덤프
set -o pipefail
ROOT="$HOME/2026-HL-FMA-VTD"; OUT="$HOME/hlfma/logs/diag_stop"; mkdir -p "$OUT"
export ROS_DOMAIN_ID=43
source /opt/ros/jazzy/setup.bash; source "$ROOT/hlfma_ws/install/setup.bash"
export LD_LIBRARY_PATH="$HOME/acados/lib:${LD_LIBRARY_PATH:-}"
pkill -9 -f "[-][-]ros-args" 2>/dev/null; pkill -f mock_vtd.py 2>/dev/null; pkill -f start_autoware 2>/dev/null; sleep 2
python3 "$ROOT/mock_vtd.py" --tl 3 ${MOCK_ARGS:-} > "$OUT/mock.log" 2>&1 &
sleep 1
cd "$ROOT"; RVIZ=false AUTO_ENGAGE=false ROUTE_CSV="$ROOT/docs/대회정보/route_example.csv" ./start_autonomous.sh mock > "$OUT/autoware.log" 2>&1 &
AW=$!
sleep 100
E() { echo "### $1"; shift; timeout 12 "$@" 2>&1 | head -${N:-40}; }
E "ego pose/z" ros2 topic echo /localization/kinematic_state --once --field pose.pose.position
E "route state" ros2 topic echo /api/routing/state --once --qos-durability transient_local --qos-reliability reliable --field state
E "mrm_state (before engage)" ros2 topic echo /system/fail_safe/mrm_state --once
E "operation_mode (before)" ros2 topic echo /api/operation_mode/state --once --qos-durability transient_local --qos-reliability reliable
echo "### engage"; timeout 15 ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode '{}' | grep -o "success=[A-Za-z]*.*"
sleep 10
E "operation_mode (after)" ros2 topic echo /api/operation_mode/state --once --qos-durability transient_local --qos-reliability reliable
E "mrm_state (after)" ros2 topic echo /system/fail_safe/mrm_state --once
E "control_cmd" ros2 topic echo /control/command/control_cmd --once --field longitudinal
E "trajectory_follower cmd" ros2 topic echo /control/trajectory_follower/control_cmd --once --field longitudinal
N=30 E "trajectory first points (vel)" bash -c "ros2 topic echo /planning/scenario_planning/trajectory --once --field points | grep -E 'longitudinal_velocity_mps|acceleration_mps2' | head -20"
N=12 E "trajectory 크기" bash -c "ros2 topic echo /planning/scenario_planning/trajectory --once | grep -c 'longitudinal_velocity_mps'"
N=12 E "behavior path vel (first 10)" bash -c "ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once | grep -E 'longitudinal_velocity_mps' | head -10"
N=12 E "path_with_lane_id vel (first 10)" bash -c "ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id --once | grep -E 'longitudinal_velocity_mps' | head -10"
echo "### planning_factors (데이터 있는 것)"
for t in $(ros2 topic list | grep planning_factors); do
  d=$(timeout 3 ros2 topic echo "$t" --once 2>/dev/null | grep -cE "behavior:|lane_id|distance")
  [ "${d:-0}" -gt 0 ] && { echo "--- $t"; timeout 3 ros2 topic echo "$t" --once 2>/dev/null | grep -E "module|behavior|distance|velocity|detail|lane" | head -12; }
done
E "velocity_factors" ros2 topic echo /api/planning/velocity_factors --once
N=40 E "diag ERROR 항목" bash -c "timeout 8 ros2 topic echo /diagnostics --qos-reliability best_effort 2>/dev/null | grep -B2 -A1 'level: \\\"\\\\x02\\\"' | grep name | sort -u | head -30"
N=40 E "behavior_path 경고(최근)" bash -c "grep -E 'out of route|planner_manager|planning_validator' $OUT/autoware.log | tail -5 | sed -E 's/\\[[0-9.]+\\]//'"
N=30 E "brige route/goal log" bash -c "grep -E '점 [0-9]:|goal:|경로 설정|경로 거부|TlRouter' $HOME/hlfma/logs/bridge_latest.log | head -12"
E "gate: is_emergency / mode" bash -c "ros2 topic echo /control/vehicle_cmd_gate/is_filter_activated --once; ros2 topic echo /api/fail_safe/mrm_state --once"
echo "### 60s 주행 관찰"; sleep 60; grep "pos=" "$OUT/mock.log" | awk "NR%5==0" | tail -8; E "velocity" ros2 topic echo /vehicle/status/velocity_status --once --field longitudinal_velocity
echo "### 최근 WARN/ERROR"
grep -E "\[(WARN|ERROR)\]" "$OUT/autoware.log" | tail -25 | sed -E "s/\[[0-9.]+\]//"
kill $AW 2>/dev/null; sleep 5; pkill -9 -f "[-][-]ros-args" 2>/dev/null; pkill -f mock_vtd.py 2>/dev/null
echo "### done"

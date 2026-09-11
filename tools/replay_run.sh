#!/usr/bin/env bash
# VTD 없이: planning 스택만 올리고 백에서 입력을 흘려 behavior_path_planner 를 돌린다.
#   usage: replay_run.sh <bag> [start_s] [duration_s]
set -o pipefail
ROOT=/home/a/2026-HL-FMA-VTD-NG
BAG="${1:?bag 경로 필요}"
START="${2:-0}"
DUR="${3:-0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
source /opt/ros/jazzy/setup.bash
source "$ROOT/hlfma_ws/install/setup.bash"

TS=$(date +%m%d_%H%M%S)
OUT="$ROOT/test/replay_$TS"
mkdir -p "$OUT"
export ROS_LOG_DIR="$OUT/roslog"

# 직전 런의 프로세스가 하나라도 남으면 다음 스택의 map_container 가
# PointCloudMapLoaderNode 인스턴스화에서 멈춰 vector_map 이 영영 안 나온다(실측 5회 연속).
# pkill 패턴 하나로는 다 못 잡아서 PID 를 모아 확실히 죽인다.
# 주의: pkill -f 는 이 스크립트 자신의 커맨드라인도 매치할 수 있으므로 자기/부모 PID 는 제외한다.
echo "[replay] 잔존 프로세스 정리"
ME=$$; PP=$PPID
VICTIMS=$( { pgrep -f "autoware[.]launch[.]xml"; \
             pgrep -f "rclcpp_components/component_container"; \
             pgrep -f "$ROOT/hlfma_ws/install/"; \
             pgrep -f "bag_replay[.]py"; \
             pgrep -f "ros2 bag record"; } 2>/dev/null | sort -u )
for pid in $VICTIMS; do
  [ "$pid" = "$ME" ] || [ "$pid" = "$PP" ] || kill -KILL "$pid" 2>/dev/null
done
sleep 6
# 죽은 참가자가 남긴 fastrtps SHM 파일이 쌓이면 새 프로세스가 포트를 못 잠가
# 디스커버리가 통째로 깨진다(실측: 2330개 쌓였을 때 ros2 node list 가 0건).
# 이 시점엔 ROS 프로세스가 없으므로 지워도 안전하다.
find /dev/shm -maxdepth 1 -name "fastrtps_*" -delete 2>/dev/null
find /dev/shm -maxdepth 1 -name "_port*_el" -delete 2>/dev/null
# 직전 스택이 완전히 사라지기 전에 새로 띄우면 map_container 가 PointCloudMapLoaderNode
# 인스턴스화에서 멈춰 vector_map 이 영영 안 나온다(실측 4회 연속). 넉넉히 기다린다.
sleep 8

echo "[replay] Autoware 기동 (브리지 없음)"
setsid ros2 launch autoware_launch autoware.launch.xml \
  map_path:="$ROOT/map" vehicle_model:=hlfma_vehicle sensor_model:=sample_sensor_kit rviz:=false \
  launch_perception:=false launch_localization:=false launch_sensing:=false \
  launch_sensing_driver:=false launch_vehicle_interface:=false \
  launch_system:=true system_run_mode:=planning_simulation \
  launch_system_monitor:=false launch_dummy_diag_publisher:=true \
  is_simulation:=true > "$OUT/autoware.log" 2>&1 &
AW=$!
echo "$AW" > "$OUT/aw.pid"

# ros2 node list 는 DDS 디스커버리에 의존해 느리고 잘 실패한다. 노드가 스스로 찍는
# 로그 줄을 보는 편이 빠르고 확실하다.
echo "[replay] behavior_path_planner 대기"
# behavior_path_planner 의 "waiting for" 는 노드가 뜨자마자 나와서 너무 이르다.
# 맵이 아직 로딩 중이면(mission_planner 가 "waiting lanelet map") 재생 앞부분을 통째로
# 흘려버려 자차가 훌쩍 앞으로 뛴 상태에서 계획이 시작된다. 맵 로드 완료 신호인
# mission_planner 의 "waiting odometry" 를 기다린다.
# 두 조건이 모두 필요하다.
#   - mission_planner "waiting odometry" : 맵 로드 완료 (이전엔 로딩 중에 재생을 시작해
#     앞부분을 통째로 흘렸다)
#   - behavior_path_planner "waiting for route" : 플래너 노드 기동 완료 (인스턴스화가
#     간헐적으로 100초 넘게 걸리거나 멈추는 일이 있다)
READY=0
for i in $(seq 1 240); do
  if grep -q "mission_planner\]: waiting odometry" "$OUT/autoware.log" 2>/dev/null && \
     grep -q "behavior_path_planner\]: waiting for" "$OUT/autoware.log" 2>/dev/null; then
    echo "[replay] 준비됨 (${i}s)"; READY=1; break
  fi
  sleep 1
done
if [ "$READY" != "1" ]; then
  echo "[replay] 준비 실패 - 스택이 안 떴다. 이 런은 버릴 것." >&2
fi

# RTC 승인자. 이게 없으면 차선변경이 영영 승인되지 않아 current_route_lanelet 이
# 목표 차로로 못 넘어가고, getLaneletSequence 가 차로 끝에서 잘려 경로가 2점으로 무너진다.
echo "[replay] blocked_route_detour 기동"
ros2 run vtd_autoware_bridge blocked_route_detour --ros-args \
  -p hold_distance_m:=45.0 -p approach_speed_mps:=4.0 > "$OUT/detour.log" 2>&1 &
echo "$!" > "$OUT/detour.pid"
sleep 2

echo "[replay] 출력 녹화 시작"
# setsid 로 띄우면 $! 가 래퍼 PID 라 SIGINT 가 레코더에 안 닿아 mcap 이 미완성으로 남는다.
ros2 bag record -o "$OUT/bag" \
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/lane_change_left/processing_time_ms \
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/total_time/processing_time_ms \
  /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id \
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/bound \
  /localization/kinematic_state \
  /perception/object_recognition/objects \
  /planning/cooperate_status/lane_change_left \
  /planning/cooperate_status/external_request_lane_change_right \
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/internal_state \
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/lane_change_left \
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/external_request_lane_change_right \
  /system/operation_mode/state \
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/turn_signal_info \
  /planning/scenario_planning/lane_driving/behavior_planning/path_reference \
  /planning/path_candidate/lane_change_left \
  /planning/path_candidate/external_request_lane_change_right \
  /planning/cooperate_status/lane_change_right \
  /planning/cooperate_status/external_request_lane_change_left \
  > "$OUT/record.log" 2>&1 &
echo "$!" > "$OUT/rec.pid"
sleep 3

echo "[replay] 백 재생: $BAG  start=$START dur=$DUR"
python3 "$ROOT/tools/bag_replay.py" "$BAG" --start "$START" --duration "$DUR" \
  --force-autonomous 2>&1 | tee "$OUT/replay.log" &
REPLAY=$!
# 자차 상태가 흐르기 시작하면 AUTONOMOUS 로 전환한다. 실주행과 같은 조건을 만들기 위함이며
# (planner_manager 의 route snap 스킵이 AUTONOMOUS 를 본다), 제어 명령은 받는 곳이 없어 무해하다.
sleep 8
timeout 20 ros2 service call /api/operation_mode/change_to_autonomous \
  autoware_adapi_v1_msgs/srv/ChangeOperationMode {} > "$OUT/engage.log" 2>&1
echo "[replay] engage: $(tail -2 "$OUT/engage.log" | tr "\n" " ")"
wait $REPLAY

sleep 2
echo "[replay] 정리"
kill -INT "$(cat "$OUT/rec.pid")" 2>/dev/null
kill -INT "$(cat "$OUT/detour.pid")" 2>/dev/null
sleep 3
kill -INT "$AW" 2>/dev/null
sleep 5
pkill -KILL -f "autoware[.]launch[.]xml" 2>/dev/null
pkill -KILL -f "$ROOT/hlfma_ws/install/" 2>/dev/null
pkill -KILL -f "rclcpp_components/component_container" 2>/dev/null
ros2 bag reindex "$OUT/bag" -s mcap >/dev/null 2>&1
echo "[replay] OUT=$OUT"

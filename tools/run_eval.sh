#!/usr/bin/env bash
# VTD 실주행 1회 = 기동 -> engage -> 녹화 -> 종료. 분석은 tools/eval_run.py 가 한다.
#   usage: run_eval.sh [duration_s] [label]
set -o pipefail
ROOT=/home/a/2026-HL-FMA-VTD-NG
DUR="${1:-150}"
LABEL="${2:-run}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
source /opt/ros/jazzy/setup.bash
source "$ROOT/hlfma_ws/install/setup.bash"

TS=$(date +%m%d_%H%M%S)
OUT="$ROOT/test/vtd_${TS}_${LABEL}"
mkdir -p "$OUT"

echo "[eval] 잔존 정리"
ME=$$; PP=$PPID
for p in $( { pgrep -f "autoware[.]launch[.]xml"; pgrep -f "rclcpp_components/component_container"; \
              pgrep -f "$ROOT/hlfma_ws/install/"; pgrep -f "ros2 bag record"; \
              pgrep -f "vtd_autoware_bridge"; pgrep -f "start_autonomous"; } 2>/dev/null | sort -u ); do
  [ "$p" = "$ME" ] || [ "$p" = "$PP" ] || kill -KILL "$p" 2>/dev/null
done
sleep 6
find /dev/shm -maxdepth 1 -name "fastrtps_*" -delete 2>/dev/null
find /dev/shm -maxdepth 1 -name "_port*_el" -delete 2>/dev/null

echo "[eval] 스택 기동"
cd "$ROOT"
setsid ./start_autonomous.sh > "$OUT/stack.log" 2>&1 &
echo $! > "$OUT/stack.pid"

# WaitingForEngage 는 맵·경로·자차 위치가 모두 준비돼야 나온다. VTD 시나리오가
# 아직 시작 전이면 브리지가 "VTD 수신 오류(timed out)" 를 반복하며 여기서 대기한다.
# 시뮬을 켜는 데 시간이 걸릴 수 있으므로 넉넉히 기다린다.
READY=0
for i in $(seq 1 3000); do
  if grep -q "AutowareState: .*=> WaitingForEngage" "$OUT/stack.log" 2>/dev/null; then
    echo "[eval] 준비됨 (${i}s)"; READY=1; break
  fi
  if [ $((i % 30)) -eq 0 ]; then
    echo "[eval] 대기 ${i}s (VTD 수신 오류 $(grep -c "VTD 수신 오류" "$HOME/hlfma/logs/bridge_latest.log" 2>/dev/null || echo 0)건)"
  fi
  sleep 1
done
if [ "$READY" != "1" ]; then
  echo "[eval] 준비 실패 - VTD 데이터가 안 온다. 이 런은 버릴 것." >&2
fi

echo "[eval] 녹화 시작"
ros2 bag record -o "$OUT/bag" \
  /localization/kinematic_state \
  /perception/object_recognition/objects \
  /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id \
  /planning/scenario_planning/trajectory \
  /planning/cooperate_status/lane_change_left \
  /planning/cooperate_status/lane_change_right \
  /planning/cooperate_status/external_request_lane_change_left \
  /planning/cooperate_status/external_request_lane_change_right \
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/internal_state \
  /vehicle/status/steering_status \
  /control/command/control_cmd \
  > "$OUT/record.log" 2>&1 &
echo $! > "$OUT/rec.pid"
sleep 2

echo "[eval] engage"
./start_hlfma.sh > "$OUT/engage.log" 2>&1
tail -2 "$OUT/engage.log"

echo "[eval] ${DUR}s 주행"
sleep "$DUR"

echo "[eval] 종료"
kill -INT "$(cat "$OUT/rec.pid")" 2>/dev/null
sleep 4
for p in $( { pgrep -f "autoware[.]launch[.]xml"; pgrep -f "rclcpp_components/component_container"; \
              pgrep -f "$ROOT/hlfma_ws/install/"; pgrep -f "vtd_autoware_bridge"; \
              pgrep -f "start_autonomous"; } 2>/dev/null | sort -u ); do
  [ "$p" = "$ME" ] || [ "$p" = "$PP" ] || kill -KILL "$p" 2>/dev/null
done
sleep 3
ros2 bag reindex "$OUT/bag" -s mcap >/dev/null 2>&1
cp -f "$HOME/hlfma/logs/autoware_latest.log" "$OUT/autoware.log" 2>/dev/null
cp -f "$HOME/hlfma/logs/bridge_latest.log" "$OUT/bridge.log" 2>/dev/null
echo "[eval] OUT=$OUT"

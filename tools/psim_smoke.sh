#!/usr/bin/env bash
# psim 헤드리스 스모크: 기동 → 120초 후 노드/토픽 스냅샷 → 종료
OUT=/tmp/claude-1000/-home-a-autoware/ddb88ad1-97a2-474f-bb75-105c5c850c7d/scratchpad
source /opt/ros/jazzy/setup.bash
source "$HOME/autoware/install/setup.bash"
export LD_LIBRARY_PATH="$HOME/acados/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=43

ros2 daemon stop >/dev/null 2>&1

ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:="$HOME/2026-HL-FMA-VTD/map" \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  rviz:=false \
  > "$OUT/psim_launch.log" 2>&1 &
LAUNCH_PID=$!

sleep 120

ros2 node list --no-daemon > "$OUT/psim_nodes.txt" 2>&1
ros2 topic list --no-daemon > "$OUT/psim_topics.txt" 2>&1
timeout 15 ros2 topic info /map/vector_map > "$OUT/psim_map_check.txt" 2>&1
grep -q "Publisher count: [1-9]" "$OUT/psim_map_check.txt" && echo MAP-PUBLISHED >> "$OUT/psim_map_check.txt" || echo MAP-MISSING >> "$OUT/psim_map_check.txt"

# 종료 전 사망 프로세스 스냅샷 (teardown 이전 것만 의미 있음)
grep -c "process has died" "$OUT/psim_launch.log" > "$OUT/psim_died_before_teardown.txt" 2>&1

kill -INT $LAUNCH_PID 2>/dev/null
sleep 12
kill -TERM $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null
pkill -f planning_simulator 2>/dev/null
sleep 3
pkill -9 -f "[-][-]ros-args" 2>/dev/null   # 고아 노드 방지 (자기매칭 없는 패턴)
ros2 daemon stop >/dev/null 2>&1
echo SMOKE-DONE

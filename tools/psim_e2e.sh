#!/usr/bin/env bash
# psim 헤드리스 E2E: 기동 → initialpose → goal → trajectory 발행 확인
# 좌표: demo_example_822m.csv 1→2번점 (LivingLab, local=VTD world)
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
  > "$OUT/e2e_launch.log" 2>&1 &
LAUNCH_PID=$!

sleep 150
ros2 daemon start >/dev/null 2>&1; sleep 8
ros2 node list > "$OUT/e2e_nodes.txt" 2>&1

# initialpose: (4.933,-24.564) heading 1.704rad → q(z,w)=(0.7527,0.6584)
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: map}, pose: {pose: {position: {x: 4.933, y: -24.564, z: 0.0}, orientation: {z: 0.7527, w: 0.6584}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}' >/dev/null 2>&1
sleep 12
timeout 10 ros2 topic echo /localization/kinematic_state --once --field pose.pose.position > "$OUT/e2e_kinematic.txt" 2>&1 && echo LOCALIZATION-OK >> "$OUT/e2e_kinematic.txt" || echo LOCALIZATION-MISSING >> "$OUT/e2e_kinematic.txt"

# 현재 라우팅 상태 + 기존 route 내용 확인 (transient_local QoS)
timeout 10 ros2 topic echo /api/routing/state --once --qos-durability transient_local --qos-reliability reliable > "$OUT/e2e_state_before.txt" 2>&1
timeout 10 ros2 topic echo /planning/mission_planning/route --once --qos-durability transient_local --qos-reliability reliable --field goal_pose > "$OUT/e2e_preexisting_route.txt" 2>&1

# clear → set: goal (-1.696,24.829) heading 0.509rad → q(z,w)=(0.2518,0.9678)
timeout 15 ros2 service call /api/routing/clear_route autoware_adapi_v1_msgs/srv/ClearRoute '{}' > "$OUT/e2e_clear_svc.txt" 2>&1
sleep 2
timeout 20 ros2 service call /api/routing/set_route_points autoware_adapi_v1_msgs/srv/SetRoutePoints '{header: {frame_id: map}, goal: {position: {x: -1.696, y: 24.829, z: 0.0}, orientation: {z: 0.7527, w: 0.6584}}, option: {allow_goal_modification: true}}' > "$OUT/e2e_route_svc.txt" 2>&1
sleep 10

timeout 15 ros2 topic echo /planning/mission_planning/route --once --qos-durability transient_local --qos-reliability reliable --field header > "$OUT/e2e_route.txt" 2>&1 && echo ROUTE-OK >> "$OUT/e2e_route.txt" || echo ROUTE-MISSING >> "$OUT/e2e_route.txt"

# trajectory: 디스커버리 플레이크 대비 3회 재시도
: > "$OUT/e2e_traj.txt"
for i in 1 2 3; do
  if timeout 15 ros2 topic echo /planning/scenario_planning/trajectory --once --field header >> "$OUT/e2e_traj.txt" 2>&1; then
    echo TRAJECTORY-OK >> "$OUT/e2e_traj.txt"; break
  fi
  [ $i -eq 3 ] && echo TRAJECTORY-MISSING >> "$OUT/e2e_traj.txt"
  sleep 5
done

# engage → 자율주행 전환 → 실제 주행 확인
timeout 15 ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode '{}' > "$OUT/e2e_engage_svc.txt" 2>&1
sleep 15
timeout 10 ros2 topic echo /localization/kinematic_state --once --field twist.twist.linear > "$OUT/e2e_speed.txt" 2>&1
timeout 10 ros2 topic echo /api/operation_mode/state --once --qos-durability transient_local --qos-reliability reliable --field mode > "$OUT/e2e_opmode.txt" 2>&1
grep -c "process has died" "$OUT/e2e_launch.log" > "$OUT/e2e_died.txt"

kill -INT $LAUNCH_PID 2>/dev/null
sleep 12
kill -TERM $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null
pkill -f planning_simulator 2>/dev/null
sleep 3
pkill -9 -f "[-][-]ros-args" 2>/dev/null   # 고아 노드 방지 (자기매칭 없는 패턴)
ros2 daemon stop >/dev/null 2>&1
echo E2E-DONE

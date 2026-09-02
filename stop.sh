#!/usr/bin/env bash
# 전체 정지: 브리지·route_node·Autoware·rviz 를 확실히 종료 (start_autonomous.sh 종료가 미진할 때)
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkill -INT -f "vtd_autoware_bridge|vtd_route_node|autoware.launch.xml|planning_simulator.launch.xml" 2>/dev/null
for i in $(seq 1 16); do pgrep -f "autoware.launch|vtd_autoware_bridge" >/dev/null || break; sleep 0.5; done
pkill -KILL -f "vtd_autoware_bridge|vtd_route_node|autoware.launch.xml|planning_simulator.launch.xml" 2>/dev/null
pkill -KILL -f "rclcpp_components/component_container" 2>/dev/null
pkill -KILL -f "robot_state_publisher .*ros-args" 2>/dev/null
pkill -KILL -f "$ROOT/hlfma_ws/install/" 2>/dev/null
pkill -f "rviz2" 2>/dev/null
sleep 1
n=$(pgrep -f "ros-args|autoware.launch|component_container|vtd_autoware" | wc -l)
echo "정지 완료. 잔존 프로세스 $n개"
[ "$n" -gt 0 ] && pgrep -af "ros-args|component_container" | grep -v pgrep | cut -c1-80

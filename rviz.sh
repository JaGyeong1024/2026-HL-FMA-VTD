#!/usr/bin/env bash
# rviz 만 별도 실행 (start_autonomous.sh 가 떠 있을 때). 종료해도 주행에 영향 없음.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
source /opt/ros/jazzy/setup.bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/hlfma_ws/install/setup.bash"
CFG="$(ros2 pkg prefix autoware_launch)/share/autoware_launch/rviz/autoware.rviz"
exec rviz2 -d "$CFG" -s "$(ros2 pkg prefix autoware_launch)/share/autoware_launch/rviz/image/autoware.png"

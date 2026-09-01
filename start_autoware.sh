#!/usr/bin/env bash
# HL FMA — 브리지 + Autoware 원커맨드 기동
#
# usage:
#   ./start_autoware.sh              # 실기 VTD(192.168.50.11) 연동 모드
#   ./start_autoware.sh psim         # planning_simulator (VTD 없이 맵·플래닝만 확인)
#   ./start_autoware.sh <vtd_host>   # 다른 호스트
#
# 종료: Ctrl+C (브리지도 같이 종료됨)
#
# 전제: ~/autoware 빌드 완료, ~/2026-HL-FMA-VTD/ros2_ws 빌드 완료
# 구성: 인지·측위·센싱·차량IF는 브리지가 대체 → launch에서 끔
# is_simulation:=true 필수: 신호등 모듈이 "데이터 없는 신호등"을 실환경에선 정지,
# 시뮬에선 통과로 처리함. 우리는 다음 신호등에만 state를 주므로 true여야 함 (scene.cpp isStopSignal)
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-192.168.50.11}"

source /opt/ros/jazzy/setup.bash
source "$HOME/autoware/install/setup.bash"
source "$ROOT/ros2_ws/install/setup.bash"

COMMON_ARGS=(
  map_path:="$ROOT/map"
  vehicle_model:=sample_vehicle
  sensor_model:=sample_sensor_kit
)

if [ "$MODE" = "psim" ]; then
  # VTD 없이 Autoware 자체 시뮬레이터로 맵·플래닝·컨트롤 확인
  exec ros2 launch autoware_launch planning_simulator.launch.xml "${COMMON_ARGS[@]}"
fi

# ── 실기 연동 모드 ──────────────────────────────────────────────
cleanup() { pkill -f "vtd_autoware_bridge" 2>/dev/null; }
trap cleanup EXIT

pkill -f "vtd_autoware_bridge" 2>/dev/null
ros2 launch vtd_autoware_bridge bridge.launch.xml vtd_host:="$MODE" \
  > /tmp/bridge.log 2>&1 &
echo "[bridge] 시작 (host=$MODE, log=/tmp/bridge.log)"
sleep 2
grep -m1 "TlRouter\|연결" /tmp/bridge.log 2>/dev/null || true

exec ros2 launch autoware_launch autoware.launch.xml \
  "${COMMON_ARGS[@]}" \
  launch_perception:=false \
  launch_localization:=false \
  launch_sensing:=false \
  launch_sensing_driver:=false \
  launch_vehicle_interface:=false \
  launch_system:=false \
  launch_dummy_diag_publisher:=false \
  is_simulation:=true \
  rviz:=true

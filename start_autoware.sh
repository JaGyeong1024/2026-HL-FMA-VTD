#!/usr/bin/env bash
# HL FMA — 브리지 + Autoware 기동 (단일 워크스페이스 hlfma_ws)
#
# usage:
#   ./start_autoware.sh                       # 실기: VTD 192.168.50.11 (대회장·연구실 동일 IP)
#   ./start_autoware.sh mock                  # 시뮬 PC 없이: 별도 터미널에서 python3 mock_vtd.py 를 먼저 띄울 것
#   ./start_autoware.sh psim                  # Autoware 내장 planning_simulator (브리지 없음, 맵·플래닝만)
#   ./start_autoware.sh <host>                # 다른 VTD 호스트
#
# 환경변수 (선택):
#   ROUTE_CSV=/path/to/route.csv   경로 자동 주입 (route_node). 기본: $HOME/hlfma/route/route_config.yaml 의 csv_path
#   ROUTE_CSV=none                 경로 주입 안 함 (rviz 2D Goal Pose 수동)
#   AUTO_ENGAGE=true               경로 SET 후 자율주행 전환 자동 (기본 false: 사람이 확인 후 engage)
#   RVIZ=false                     rviz 끔 (본선 기동 시간 단축)
#
# 종료: Ctrl+C (브리지도 같이 종료)
#
# 구성 근거 (개발계획_0902 §3):
#  - perception/localization/sensing/vehicle_interface 는 브리지가 대체 → launch에서 끔
#  - system 은 켠다 (vehicle_cmd_gate의 mrm_state 하트비트, ADAPI operation_mode availability 가 여기서 나옴).
#    psim과 같은 planning_simulation 모드 + dummy_diag_publisher. system_monitor 는 빌드 제외라 반드시 false.
#  - is_simulation:=true 필수: 신호등 모듈이 "데이터 없는 신호등"을 시뮬에선 통과로 처리 (우리는 다음 정지선에만 데이터를 줌)
#  - set -u 금지: ROS setup.bash 가 미정의 변수를 참조
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-192.168.50.11}"
source "$ROOT/hlfma_ws/env.sh"   # ROS + hlfma_ws 오버레이 + acados + ROS_DOMAIN_ID

RVIZ="${RVIZ:-true}"
COMMON_ARGS=(
  map_path:="$ROOT/map"
  vehicle_model:=hlfma_vehicle
  sensor_model:=sample_sensor_kit
  rviz:="$RVIZ"
)

if [ "$MODE" = "psim" ]; then
  exec ros2 launch autoware_launch planning_simulator.launch.xml "${COMMON_ARGS[@]}"
fi

# ── 실기 / mock ────────────────────────────────────────────────
VTD_HOST="$MODE"
[ "$MODE" = "mock" ] && VTD_HOST="127.0.0.1"

# 경로 CSV: 환경변수 > route_config.yaml > 없음
ROUTE_CSV="${ROUTE_CSV:-}"
if [ -z "$ROUTE_CSV" ] && [ -f "$HOME/hlfma/route/route_config.yaml" ]; then
  ROUTE_CSV="$(sed -n 's/^csv_path:[[:space:]]*//p' "$HOME/hlfma/route/route_config.yaml" | head -1)"
fi
[ "$ROUTE_CSV" = "none" ] && ROUTE_CSV=""
if [ -n "$ROUTE_CSV" ] && [ ! -f "$ROUTE_CSV" ]; then
  echo "[route] CSV 없음: $ROUTE_CSV" >&2; exit 1
fi

cleanup() { pkill -f "vtd_autoware_bridge" 2>/dev/null; pkill -f "vtd_route_node" 2>/dev/null; }
trap cleanup EXIT
cleanup

mkdir -p "$HOME/hlfma/logs"
BRIDGE_LOG="$HOME/hlfma/logs/bridge_$(date +%m%d_%H%M%S).log"
ln -sfn "$BRIDGE_LOG" "$HOME/hlfma/logs/bridge_latest.log"
ros2 launch vtd_autoware_bridge bridge.launch.xml \
  vtd_host:="$VTD_HOST" \
  route_csv:="$ROUTE_CSV" \
  auto_engage:="${AUTO_ENGAGE:-false}" \
  > "$BRIDGE_LOG" 2>&1 &
echo "[bridge] host=$VTD_HOST route_csv=${ROUTE_CSV:-없음} auto_engage=${AUTO_ENGAGE:-false} log=$BRIDGE_LOG"
sleep 3
grep -m3 "맵 로드\|VTD 연결\|경로 CSV" "$BRIDGE_LOG" 2>/dev/null || true

exec ros2 launch autoware_launch autoware.launch.xml \
  "${COMMON_ARGS[@]}" \
  launch_perception:=false \
  launch_localization:=false \
  launch_sensing:=false \
  launch_sensing_driver:=false \
  launch_vehicle_interface:=false \
  launch_system:=true \
  system_run_mode:=planning_simulation \
  launch_system_monitor:=false \
  launch_dummy_diag_publisher:=true \
  is_simulation:=true

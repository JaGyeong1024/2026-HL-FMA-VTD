#!/usr/bin/env bash
# HL FMA — 한 번에 기동+출발: 브리지 + Autoware 기동 후 경로 SET·자율주행 가능 확인 → engage (start_hlfma.sh 호출)
#
# usage:
#   ./start_autonomous.sh                     # 실기: VTD 192.168.50.11 (대회장·연구실 동일 IP)
#   ./start_autonomous.sh mock                # 시뮬 PC 없이: 별도 터미널에서 python3 mock_vtd.py 를 먼저 띄울 것
#   ./start_autonomous.sh psim                # Autoware 내장 planning_simulator (브리지 없음, 맵·플래닝만)
#   ./start_autonomous.sh <host>              # 다른 VTD 호스트
#
#   기본 흐름:  터미널1 ./start_autonomous.sh  (기동 → 경로 SET → engage 까지 자동. rviz 는 별도 터미널 ./rviz.sh)
#   수동 출발:  ENGAGE=false ./start_autonomous.sh  →  확인 후 터미널2 ./start_hlfma.sh
#
# 환경변수 (선택):
#   ENGAGE=false                   기동만 하고 출발(engage)은 사람이 ./start_hlfma.sh 로 (기본 true)
#   ROUTE_CSV=/path/to/route.csv   경로 자동 주입 (route_node). 기본: $HOME/hlfma/route/route_config.yaml 의 csv_path
#   ROUTE_CSV=none                 경로 주입 안 함 (rviz 2D Goal Pose 수동)
#   AUTO_ENGAGE=true               (구) route_node 가 SET 직후 즉시 engage. 자율주행 가능 여부를 안 기다리므로 기본 false 유지
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
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"

source /opt/ros/jazzy/setup.bash
source "$ROOT/hlfma_ws/install/setup.bash"
# acados(MPC) 는 /etc/ld.so.conf.d/acados.conf + ldconfig 로 시스템 등록됨 (env 불필요)

COMMON_ARGS=(
  map_path:="$ROOT/map"
  vehicle_model:=hlfma_vehicle
  sensor_model:=sample_sensor_kit
  rviz:=false
)

if [ "$MODE" = "psim" ]; then
  ros2 launch autoware_launch planning_simulator.launch.xml "${COMMON_ARGS[@]}" & AW_PID=$!; wait "$AW_PID"; exit 0
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

# 이미 떠 있는 Autoware/브리지가 있으면 중단 (VTD 9910 은 동시 접속 1개, ROS 그래프 중복 방지)
if pgrep -f "autoware.launch.xml|planning_simulator.launch.xml" >/dev/null; then
  echo "[start] Autoware 가 이미 실행 중입니다. 먼저 종료하세요:  pkill -f ros-args ; pkill -f vtd_autoware_bridge" >&2
  exit 1
fi
# 종료 훅: Ctrl+C(SIGINT)/종료 시 브리지·route_node·Autoware 노드를 확실히 정리한다.
# ros2 launch 가 자식에 SIGINT 를 보내지만 component_container 일부가 늦게 죽어 다음 실행을
# 방해하므로(ROS 그래프 잔존), 여기서 프로세스 그룹째 정리한다.
AW_PID=""
cleanup() {
  trap - EXIT INT TERM
  echo "[start] 종료 정리..." >&2
  [ -n "$AW_PID" ] && kill -INT "$AW_PID" 2>/dev/null
  pkill -INT -f "vtd_autoware_bridge|vtd_route_node" 2>/dev/null
  # component_container 등이 8초 안에 안 죽으면 강제
  for i in $(seq 1 16); do pgrep -f "autoware.launch.xml|vtd_autoware_bridge" >/dev/null || break; sleep 0.5; done
  pkill -KILL -f "vtd_autoware_bridge|vtd_route_node" 2>/dev/null
  pkill -KILL -f "autoware.launch.xml" 2>/dev/null
  pkill -KILL -f "rclcpp_components/component_container" 2>/dev/null
  pkill -KILL -f "robot_state_publisher .*ros-args" 2>/dev/null
  # 이 launch 가 띄운 잔존 노드(설치 경로로 식별) 정리
  pkill -KILL -f "$ROOT/hlfma_ws/install/" 2>/dev/null
  exit 0
}
trap cleanup EXIT INT TERM
# 시작 전, 지난 실행의 잔존물이 있으면 정리
pkill -KILL -f "vtd_autoware_bridge|vtd_route_node" 2>/dev/null

mkdir -p "$HOME/hlfma/logs"
RUN_TS="$(date +%m%d_%H%M%S)"
BRIDGE_LOG="$HOME/hlfma/logs/bridge_${RUN_TS}.log"
AW_LOG="$HOME/hlfma/logs/autoware_${RUN_TS}.log"
# 노드별 ROS 로그(launch.log, 각 노드 stderr)를 실행 단위 디렉터리에 모음 → record.sh 가 통째로 복사
export ROS_LOG_DIR="$HOME/hlfma/logs/ros_${RUN_TS}"
mkdir -p "$ROS_LOG_DIR"
ln -sfn "$BRIDGE_LOG" "$HOME/hlfma/logs/bridge_latest.log"
ln -sfn "$AW_LOG" "$HOME/hlfma/logs/autoware_latest.log"
# 실행 정보 매니페스트: record.sh 가 읽어 로그 위치·VTD 호스트를 안다
cat > "$HOME/hlfma/logs/run_latest.env" <<EOF
RUN_TS=$RUN_TS
VTD_HOST=$VTD_HOST
MODE=$MODE
ROUTE_CSV=$ROUTE_CSV
BRIDGE_LOG=$BRIDGE_LOG
AW_LOG=$AW_LOG
ROS_LOG_DIR=$ROS_LOG_DIR
ENGAGE=${ENGAGE:-true}
GIT_REV=$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null)
EOF
ros2 launch vtd_autoware_bridge bridge.launch.xml \
  vtd_host:="$VTD_HOST" \
  route_csv:="$ROUTE_CSV" \
  auto_engage:="${AUTO_ENGAGE:-false}" \
  > "$BRIDGE_LOG" 2>&1 &
echo "[bridge] host=$VTD_HOST route_csv=${ROUTE_CSV:-없음} auto_engage=${AUTO_ENGAGE:-false} log=$BRIDGE_LOG"
sleep 3
grep -m3 "맵 로드\|VTD 연결\|경로 CSV" "$BRIDGE_LOG" 2>/dev/null || true

ros2 launch autoware_launch autoware.launch.xml \
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
  is_simulation:=true > >(tee -i "$AW_LOG") 2>&1 &
AW_PID=$!
echo "[autoware] log=$AW_LOG  ros_log_dir=$ROS_LOG_DIR"

# 출발: 경로 SET → 자율주행 가능 → engage. 실패하면 로그에 남고 Autoware 는 계속 떠 있음(수동 ./start_hlfma.sh 가능)
if [ "${ENGAGE:-true}" != "false" ]; then
  ENGAGE_LOG="$HOME/hlfma/logs/engage_${RUN_TS}.log"
  ( "$ROOT/start_hlfma.sh" 2>&1 | tee "$ENGAGE_LOG" ) &
  echo "[engage] 자동 출발 대기 중 (log=$ENGAGE_LOG). 취소: ENGAGE=false 로 재기동"
else
  echo "[engage] ENGAGE=false — 출발은 ./start_hlfma.sh 로"
fi
wait "$AW_PID"

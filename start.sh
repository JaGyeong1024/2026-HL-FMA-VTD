#!/usr/bin/env bash
# HL FMA — 기동부터 출발까지 한 번에.
#   브리지 + Autoware 기동 → 경로 주입 → 경로 SET 대기 → 자율주행 가능 대기 → engage 서비스콜
#
# usage:
#   ./start.sh                     # 실기: VTD 192.168.50.11 (대회장·연구실 동일 IP)
#   ./start.sh mock                # 시뮬 PC 없이: 별도 터미널에서 python3 mock_vtd.py 를 먼저 띄울 것
#   ./start.sh psim                # Autoware 내장 planning_simulator (브리지 없음, 맵·플래닝만)
#   ./start.sh <host>              # 다른 VTD 호스트
#
#   기본 흐름:  터미널1 ./start.sh  (기동 → 경로 SET → engage 까지 자동. rviz 는 별도 터미널 ./rviz.sh)
#   기동만:     ENGAGE=false ./start.sh
#
# 환경변수 (선택):
#   ENGAGE=false                   기동만 하고 출발은 사람이 (기본 true)
#   (LANE_PLAN/DETOUR 인자는 NG 병합(7dea5f7)으로 제거됨 — NG 판 bridge.launch.xml 에 없다)
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

# 이미 떠 있는 "이 디렉터리의" Autoware/브리지가 있으면 중단 (VTD 9910 은 동시 접속 1개, ROS 그래프 중복 방지).
# 다른 클론(-NG/-MY)의 스택은 건드리지 않는다 — 동시 주행은 ROS_DOMAIN_ID 를 다르게 해서만 가능.
if pgrep -f "map_path:=$ROOT/map" >/dev/null || pgrep -f "$ROOT/hlfma_ws/install/vtd_autoware_bridge" >/dev/null; then
  echo "[start] 이 디렉터리($ROOT)의 Autoware/브리지가 이미 실행 중입니다. 먼저 ./stop.sh 로 종료하세요." >&2
  exit 1
fi
# 종료 훅: Ctrl+C(SIGINT)/종료 시 브리지·route_node·Autoware 노드를 확실히 정리한다.
# 두 launch 를 setsid 로 각자 프로세스 그룹에 띄우고, 그룹째 신호를 보낸다(다른 클론 프로세스는 영향 없음).
# component_container 일부가 늦게 죽으면 그룹 KILL, 마지막으로 이 install 경로로 식별되는 잔존 노드 정리.
AW_PID=""; BR_PID=""
cleanup() {
  trap - EXIT INT TERM
  echo "[start] 종료 정리..." >&2
  for pid in "$AW_PID" "$BR_PID"; do [ -n "$pid" ] && kill -INT -- "-$pid" 2>/dev/null; done
  for i in $(seq 1 16); do
    alive=0; for pid in "$AW_PID" "$BR_PID"; do [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null && alive=1; done
    [ "$alive" = "0" ] && break; sleep 0.5
  done
  for pid in "$AW_PID" "$BR_PID"; do [ -n "$pid" ] && kill -KILL -- "-$pid" 2>/dev/null; done
  pkill -KILL -f "$ROOT/hlfma_ws/install/" 2>/dev/null
  exit 0
}
trap cleanup EXIT INT TERM
# 시작 전, 지난 실행의 "이 디렉터리" 잔존물이 있으면 정리
pkill -KILL -f "$ROOT/hlfma_ws/install/vtd_autoware_bridge" 2>/dev/null

mkdir -p "$HOME/hlfma/logs"
RUN_TS="$(date +%m%d_%H%M%S)"
BRIDGE_LOG="$HOME/hlfma/logs/bridge_${RUN_TS}.log"
AW_LOG="$HOME/hlfma/logs/autoware_${RUN_TS}.log"
export ROS_LOG_DIR="$HOME/hlfma/logs/ros_${RUN_TS}"
mkdir -p "$ROS_LOG_DIR"
ln -sfn "$BRIDGE_LOG" "$HOME/hlfma/logs/bridge_latest.log"
ln -sfn "$AW_LOG" "$HOME/hlfma/logs/autoware_latest.log"
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
setsid ros2 launch vtd_autoware_bridge bridge.launch.xml \
  vtd_host:="$VTD_HOST" \
  map_osm:="$ROOT/map/lanelet2_map.osm" \
  route_csv:="$ROUTE_CSV" \
  auto_engage:="${AUTO_ENGAGE:-false}" \
  > "$BRIDGE_LOG" 2>&1 < /dev/null &
BR_PID=$!
echo "[bridge] host=$VTD_HOST route_csv=${ROUTE_CSV:-없음} auto_engage=${AUTO_ENGAGE:-false} log=$BRIDGE_LOG"
sleep 3
grep -m3 "맵 로드\|VTD 연결\|경로 CSV" "$BRIDGE_LOG" 2>/dev/null || true

setsid ros2 launch autoware_launch autoware.launch.xml \
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
  is_simulation:=true > >(tee -i "$AW_LOG") 2>&1 < /dev/null &
AW_PID=$!
echo "[autoware] log=$AW_LOG  ros_log_dir=$ROS_LOG_DIR"

# 출발: 경로 SET → 자율주행 가능 → engage.
# 실패해도 Autoware 는 계속 떠 있다(로그 확인 후 수동 서비스콜 가능).
if [ "${ENGAGE:-true}" != "false" ]; then
  ENGAGE_LOG="$HOME/hlfma/logs/engage_${RUN_TS}.log"
  ( python3 - <<'PY' 2>&1 | tee "$ENGAGE_LOG"
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from autoware_adapi_v1_msgs.msg import RouteState, OperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode

rclpy.init(); n = Node('start_engage')
latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
st = {'route': None, 'avail': None, 'mode': None}
n.create_subscription(RouteState, '/api/routing/state',
                      lambda m: st.update(route=m.state), latched)
n.create_subscription(OperationModeState, '/api/operation_mode/state',
                      lambda m: st.update(avail=m.is_autonomous_mode_available, mode=m.mode), latched)

def spin(sec):
    t = time.time()
    while rclpy.ok() and time.time() - t < sec:
        rclpy.spin_once(n, timeout_sec=0.02)

# 서비스 클라이언트를 미리 만들어 둔다 — 조건이 맞자마자 바로 쏘기 위해.
cli = n.create_client(ChangeOperationMode, '/api/operation_mode/change_to_autonomous')

# 경로 SET 대기 (최대 120s, 콜드부트 여유). RouteState: 2=SET, 3=ARRIVED
print('[engage] 경로 SET 대기...')
t0 = time.time(); last = -5.0
while rclpy.ok() and time.time() - t0 < 120:
    spin(0.05)
    if st['route'] == 2:
        break
    el = time.time() - t0
    if el - last >= 5:
        last = el
        print(f"  t={el:.0f}s routing={st['route']} avail={st['avail']} (2=SET)")
if st['route'] != 2:
    print(f"[engage] 경로가 SET(2) 이 아님: state={st['route']}. 브리지 로그를 확인하세요.",
          file=sys.stderr)
    sys.exit(1)

# 자율주행 가능 대기 (최대 60s)
t0 = time.time()
while rclpy.ok() and not st['avail'] and time.time() - t0 < 60:
    spin(0.05)
if not st['avail']:
    print('[engage] 자율주행 가능 상태가 아니다 — 그래도 요청한다(사유를 응답에서 본다).',
          file=sys.stderr)

print(f"[engage] routing=SET, is_autonomous_mode_available={st['avail']} → 서비스콜")
if not cli.wait_for_service(timeout_sec=10.0):
    print('[engage] change_to_autonomous 서비스가 없다', file=sys.stderr)
    sys.exit(1)
fut = cli.call_async(ChangeOperationMode.Request())
t0 = time.time()
while rclpy.ok() and not fut.done() and time.time() - t0 < 15:
    rclpy.spin_once(n, timeout_sec=0.1)
r = fut.result()
if r is None:
    print('[engage] 서비스 응답 없음', file=sys.stderr)
    sys.exit(1)
print(f"[engage] success={r.status.success} code={r.status.code} '{r.status.message}'")
spin(3)
print(f"[engage] operation mode={st['mode']} (2=AUTONOMOUS)")
if not r.status.success or st['mode'] != 2:
    print('[engage] FAILED_FLAG')   # 아래 bash 가 이 표시를 보고 진단그래프 사유를 붙인다
print('[engage] 정지: ./stop.sh')
n.destroy_node(); rclpy.shutdown()
PY
  # 방식 b (2026-09-10): 실패해도 출발은 시도하되, 실패했으면 진단그래프 사유를 같이 남긴다.
  #   avail=False 로 60초 타임아웃 후 강행하는 경로가 있어서, 사유가 없으면 "왜 안 됐는지"가
  #   로그에 안 남고 스크립트는 그냥 진행한다(실측 01:35 오염 주행). 원인을 바로 보이게 한다.
  if grep -q "FAILED_FLAG" "$ENGAGE_LOG" 2>/dev/null; then
    {
      echo "----- engage 실패: 진단그래프 사유 (autoware 로그에서) -----"
      awk '/target mode is not available/{buf=""; keep=1} keep{buf=buf $0 ORS} /^\s*$/{if(keep){last=buf; keep=0}} END{printf "%s", last}' "$AW_LOG" 2>/dev/null | tail -25
      echo "----- ego 시작 앵커 (VTD 리셋 여부 확인) -----"
      grep "시작 앵커" "$BRIDGE_LOG" 2>/dev/null | tail -1
      echo "  (시나리오 시작점이 아니면 VTD 리셋 필요 — 직전 주행 종료 위치에서 시작한 것)"
      echo "----- 중복 노드 -----"
      grep "is duplicated" "$AW_LOG" 2>/dev/null | tail -5
      echo "----- 참고: ./stop.sh 후 ros2 daemon stop/start 로 그래프를 비우고 재시도 -----"
    } | tee -a "$ENGAGE_LOG"
  fi
  ) &
  echo "[engage] 자동 출발 대기 중 (log=$ENGAGE_LOG). 취소: ENGAGE=false 로 재기동"
else
  echo "[engage] ENGAGE=false — 기동만 함. 출발은 수동 서비스콜로."
fi
wait "$AW_PID"

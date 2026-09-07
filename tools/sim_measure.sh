#!/usr/bin/env bash
# 시뮬(VTD) 실주행 계측 — 시뮬 PC 는 건드리지 않고 제어기에서 연결·주행·기록만 한다.
# usage: bash tools/sim_measure.sh <태그> [주행초=300]
#   env: DOMAIN(기본 53, 다른 클론과 격리) DETOUR(기본 false) ROUTE(기본 route_config.yaml)
# 산출: ~/hlfma/logs/sim/<태그>_<시각>/ — bag/(계측 토픽), autoware.log, bridge.log, ready.txt, engage.txt, meta.txt
set -o pipefail
TAG="${1:?태그를 지정하세요}"; RUN_SEC="${2:-300}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$HOME/hlfma/logs/sim/${TAG}_$(date +%m%d_%H%M%S)"; mkdir -p "$OUT"
export ROS_DOMAIN_ID="${DOMAIN:-53}"
export DETOUR="${DETOUR:-false}"
export LANE_SEQ="${LANE_SEQ:-false}"
export LANE_PLAN="${LANE_PLAN:-false}"
source /opt/ros/jazzy/setup.bash; source "$ROOT/hlfma_ws/install/setup.bash"
note() { echo "  $*" | tee -a "$OUT/result.txt"; }
echo "== [$TAG] 로그: $OUT  (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, DETOUR=$DETOUR, LANE_SEQ=$LANE_SEQ)" | tee -a "$OUT/result.txt"

# 1) 사전 확인 — VTD 연결·CSV·맵
timeout 3 nc -z 192.168.50.11 9910 || { note "[중단] VTD 9910 미연결"; exit 1; }
ROUTE="${ROUTE:-$(sed -n 's/^csv_path:[[:space:]]*//p' "$HOME/hlfma/route/route_config.yaml" | head -1)}"
[ -f "$ROUTE" ] || { note "[중단] CSV 없음 $ROUTE"; exit 1; }
pgrep -f "$ROOT/hlfma_ws/install/vtd_autoware_bridge" >/dev/null && { note "[중단] 이 디렉터리 스택이 이미 실행 중"; exit 1; }
{ echo "route=$ROUTE"; echo "git=$(git -C "$ROOT" rev-parse --short HEAD)"; echo "dirty=$(git -C "$ROOT" status --porcelain | wc -l)";
  echo "map_md5=$(md5sum "$ROOT/map/lanelet2_map.osm" | cut -d' ' -f1)"; echo "domain=$ROS_DOMAIN_ID detour=$DETOUR run_sec=$RUN_SEC";
  echo "start=$(date -Iseconds)"; } > "$OUT/meta.txt"
note "route=$(basename "$ROUTE") git=$(git -C "$ROOT" rev-parse --short HEAD)"

# 2) 스택 기동 (engage 는 사람이 아니라 이 스크립트가, 준비 확인 후)
cd "$ROOT"
ENGAGE=false ROUTE_CSV="$ROUTE" ./start_autonomous.sh > "$OUT/autoware.log" 2>&1 < /dev/null &
STACK_PID=$!
cleanup() {
  trap - EXIT INT TERM
  note "정리…"
  [ -n "$BAG_PID" ] && kill -TERM "$BAG_PID" 2>/dev/null
  sleep 2
  kill -TERM "$STACK_PID" 2>/dev/null
  for i in $(seq 1 40); do kill -0 "$STACK_PID" 2>/dev/null || break; sleep 0.5; done
  pkill -KILL -f "$ROOT/hlfma_ws/install/" 2>/dev/null
  echo "end=$(date -Iseconds)" >> "$OUT/meta.txt"
  note "완료: $OUT"
}
trap cleanup EXIT INT TERM
cp "$HOME/hlfma/logs/bridge_latest.log" /dev/null 2>/dev/null

# 3) 준비 대기 (route SET + 자율주행 가능), 최대 180 s
timeout 200 python3 - "$OUT" <<'PY' | tee "$OUT/ready.txt"
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from autoware_adapi_v1_msgs.msg import RouteState, OperationModeState
rclpy.init(); n = Node('sim_ready_probe')
q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
st = {'route': None, 'avail': None, 'mode': None}
n.create_subscription(RouteState, '/api/routing/state', lambda m: st.update(route=m.state), q)
n.create_subscription(OperationModeState, '/api/operation_mode/state',
                      lambda m: st.update(avail=m.is_autonomous_mode_available, mode=m.mode), q)
t0 = time.time(); last = -10
while time.time() - t0 < 180:
    rclpy.spin_once(n, timeout_sec=0.2)
    if st['route'] == 2 and st['avail']: break
    if time.time() - t0 - last >= 10:
        last = time.time() - t0
        print(f"t={last:.0f}s route={st['route']} avail={st['avail']} mode={st['mode']}", flush=True)
print(f"READY route={st['route']} avail={st['avail']} mode={st['mode']} after={time.time()-t0:.0f}s")
sys.exit(0 if (st['route'] == 2 and st['avail']) else 1)
PY
RC=${PIPESTATUS[0]}
tail -1 "$OUT/ready.txt" | sed 's/^/  /' | tee -a "$OUT/result.txt"
[ "$RC" = "0" ] || { note "[중단] 준비 실패 — engage 하지 않음"; exit 1; }

# 4) 계측 토픽 bag (recorder 노드명 고유화: 중복이면 MRM 비상정지)
TOPICS=(/localization/kinematic_state /planning/trajectory /control/command/control_cmd
        /control/command/turn_indicators_cmd /vehicle/status/velocity_status /vehicle/status/steering_status
        /planning/mission_planning/route /planning/scenario_planning/max_velocity
        /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
        /perception/object_recognition/objects /perception/traffic_light_recognition/traffic_signals
        /api/operation_mode/state /api/routing/state /system/emergency/hazard_status
        /api/fail_safe/mrm_state /vtd/respawn /vtd/raw_rx /rosout
        /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/output/is_reroute_available)
EXTRA=$(ros2 topic list 2>/dev/null | grep -E "planning_factors/|cooperate_status/|^/detour/|avoidance_debug_message_array|behavior_path_planner/debug/internal_state|behavior_path_planner/debug/static_obstacle_avoidance$" | tr '\n' ' ')
ros2 bag record --node-name "rec_${TAG}_$(date +%H%M%S)" -o "$OUT/bag" ${TOPICS[*]} $EXTRA \
    > "$OUT/bag_record.log" 2>&1 < /dev/null &
BAG_PID=$!
sleep 3; note "bag 기록 시작 (pid $BAG_PID)"

# 5) engage
timeout 20 ros2 service call /api/operation_mode/change_to_autonomous \
    autoware_adapi_v1_msgs/srv/ChangeOperationMode '{}' > "$OUT/engage.txt" 2>&1
note "engage: $(grep -o 'success=[A-Za-z]*' "$OUT/engage.txt" | head -1)"

# 6) 주행 관찰
t=0
while [ "$t" -lt "$RUN_SEC" ]; do
  sleep 10; t=$((t+10))
  v=$(timeout 3 ros2 topic echo /vehicle/status/velocity_status --once --field longitudinal_velocity 2>/dev/null | head -1)
  m=$(timeout 3 ros2 topic echo /api/operation_mode/state --once --qos-durability transient_local --qos-reliability reliable --field mode 2>/dev/null | head -1)
  echo "  t=${t}s v=${v:-?} mode=${m:-?}" | tee -a "$OUT/result.txt"
done
note "주행 관찰 종료 (${RUN_SEC}s)"

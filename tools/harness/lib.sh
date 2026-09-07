#!/usr/bin/env bash
# 하네스 공통 단계. 각 h*.sh 가 source 한다. 함수 하나 = 단계 하나.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROS_DOMAIN_ID="${HARNESS_DOMAIN:-53}"
source /opt/ros/jazzy/setup.bash
source "$ROOT/hlfma_ws/install/setup.bash"
EGO_FRONT=3.808   # 후륜축→앞범퍼 [m] (대회정보 §4)

harness_init() {            # $1=케이스 이름 → OUT 디렉터리
  CASE="$1"; OUT="$HOME/hlfma/logs/harness/${CASE}_$(date +%m%d_%H%M%S)"; mkdir -p "$OUT"
  echo "== [$CASE] 로그: $OUT  (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
  : > "$OUT/result.txt"
}
pass() { echo "  [PASS] $*" | tee -a "$OUT/result.txt"; }
fail() { echo "  [FAIL] $*" | tee -a "$OUT/result.txt"; }
note() { echo "  [INFO] $*" | tee -a "$OUT/result.txt"; }

mock_start() {              # $@ = mock_vtd.py 인자. trace 는 자동
  pkill -f "mock_vtd.py --port 991[0]" 2>/dev/null; sleep 0.5
  ( cd "$ROOT" && nohup python3 mock_vtd.py --port 9910 --trace "$OUT/trace.csv" "$@" > "$OUT/mock.log" 2>&1 < /dev/null & )
  for i in $(seq 1 20); do nc -z 127.0.0.1 9910 2>/dev/null && break; sleep 0.5; done
  nc -z 127.0.0.1 9910 || { fail "mock 9910 미기동"; return 1; }
  echo "$*" > "$OUT/mock_args.txt"; note "mock: $*"
}
stack_start() {             # $1=route csv. engage 는 하지 않음(ENGAGE=false). 스크립트를 직접 백그라운드로 (서브셸 금지: pid 가 스크립트여야 INT 로 정리 훅이 돈다)
  ROUTE="$1"
  pushd "$ROOT" > /dev/null
  ENGAGE=false ROUTE_CSV="$ROUTE" ./start_autonomous.sh mock > "$OUT/autoware.log" 2>&1 < /dev/null &
  STACK_PID=$!; popd > /dev/null
  echo "$STACK_PID" > "$OUT/stack.pid"; note "stack pid $STACK_PID route=$ROUTE"
}
stack_wait_ready() {        # $1=timeout s. routing SET + autonomous available 대기. ready.txt 에 기록
  local to="${1:-150}"
  timeout $((to + 30)) python3 - "$to" > "$OUT/ready.txt" 2>&1 <<'PY'
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from autoware_adapi_v1_msgs.msg import RouteState, OperationModeState
to=float(sys.argv[1]); rclpy.init(); n=Node('harness_wait')
q=QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
st={'route':None,'avail':None,'mode':None}
n.create_subscription(RouteState,'/api/routing/state',lambda m: st.update(route=m.state),q)
n.create_subscription(OperationModeState,'/api/operation_mode/state',lambda m: st.update(avail=m.is_autonomous_mode_available,mode=m.mode),q)
t0=time.time(); last=-5
while time.time()-t0<to:
    rclpy.spin_once(n,timeout_sec=0.2)
    if st['route']==2 and st['avail']: break
    if time.time()-t0-last>=5: last=time.time()-t0; print(f"t={last:.0f}s route={st['route']} avail={st['avail']} mode={st['mode']}", flush=True)
print(f"READY route={st['route']} avail={st['avail']} mode={st['mode']} after={time.time()-t0:.0f}s")
sys.exit(0 if (st['route']==2 and st['avail']) else 1)
PY
  local rc=$?; tail -1 "$OUT/ready.txt" | sed 's/^/  /'; return $rc
}
capture_start() {           # 판정용 토픽 bag 기록 (선택 토픽만). $@ 추가 토픽
  local topics=(/vehicle/status/velocity_status /api/operation_mode/state /api/routing/state /localization/kinematic_state
                /control/command/control_cmd /control/trajectory_follower/control_cmd /control/command/turn_indicators_cmd /planning/mission_planning/route
                /planning/scenario_planning/max_velocity /planning/trajectory
                /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
                /perception/object_recognition/objects /diagnostics_graph/status
                /api/fail_safe/mrm_state /system/emergency/hazard_status /system/fail_safe/mrm_state /rosout "$@")
  local extra; extra=$(timeout 15 ros2 topic list 2>/dev/null | grep -E "cooperate_status|planning_factors/" | tr '\n' ' ')
  # 노드 이름을 케이스마다 다르게: 기본 /rosbag2_recorder 가 둘 이상이면 duplicated_node_checker ERROR → MRM 비상정지 →
  # 자율주행 불가 (baseline 9/7 준비 실패의 원인). 직접 백그라운드 (서브셸이면 pid 가 틀려 종료 못 함)
  ros2 bag record --node-name "rec_$(basename "$OUT" | tr -c "A-Za-z0-9_\n" "_")" -o "$OUT/bag" ${topics[*]} $extra > "$OUT/bag_record.log" 2>&1 < /dev/null &
  echo $! > "$OUT/bag.pid"
  note "bag 기록 시작 (pid $(cat "$OUT/bag.pid"))"
}
engage() {                  # 자율주행 전환. 응답과 벽시계를 events.json 에
  local wall; wall=$(date +%s.%N)
  timeout 20 ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode '{}' > "$OUT/engage.txt" 2>&1
  local ok; ok=$(grep -o "success=[A-Za-z]*" "$OUT/engage.txt" | head -1)
  timeout 10 python3 - "$OUT" "$wall" "$ok" <<'PY'
import json, sys, os
p=os.path.join(sys.argv[1],'events.json'); d=json.load(open(p)) if os.path.exists(p) else {}
d['engage_wall']=float(sys.argv[2]); d['engage_result']=sys.argv[3]; json.dump(d,open(p,'w'),indent=1)
PY
  note "engage: $ok"
}
event() {                   # $1=이름 : 벽시계 기록 (metrics 가 trace wall 과 대조)
  timeout 10 python3 - "$OUT" "$1" "$(date +%s.%N)" <<'PY'
import json, sys, os
p=os.path.join(sys.argv[1],'events.json'); d=json.load(open(p)) if os.path.exists(p) else {}
d.setdefault('marks',{})[sys.argv[2]]=float(sys.argv[3]); json.dump(d,open(p,'w'),indent=1)
PY
}
run_for() {                 # $1=초. 2 s 마다 mock 상태 한 줄
  local sec="$1"; local t=0
  while [ "$t" -lt "$sec" ]; do sleep 2; t=$((t+2)); [ $((t%10)) = 0 ] && grep "pos=" "$OUT/mock.log" | tail -1 | sed 's/^/  /'; done
}
set_velocity_limit() {      # $1=m/s. 외부 속도제한 후보(sender=harness)를 transient_local 로 1회 발행하고 종료 (ros2 topic pub 는 QoS 불일치 시 무한대기)
  timeout 20 python3 - "$1" <<'PY' >> "$OUT/result.txt" 2>&1
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from autoware_internal_planning_msgs.msg import VelocityLimit
rclpy.init(); n = Node('harness_vel_limit')
q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
pub = n.create_publisher(VelocityLimit, '/planning/scenario_planning/max_velocity_candidates', q)
m = VelocityLimit(); m.stamp = n.get_clock().now().to_msg(); m.max_velocity = float(sys.argv[1]); m.use_constraints = False; m.sender = 'harness'
t0 = time.time()
while time.time() - t0 < 5.0 and pub.get_subscription_count() == 0: rclpy.spin_once(n, timeout_sec=0.2)
pub.publish(m); rclpy.spin_once(n, timeout_sec=0.5)
print(f"  [INFO] 속도 상한 {m.max_velocity} m/s 발행 (구독자 {pub.get_subscription_count()})")
n.destroy_node(); rclpy.shutdown()
PY
  tail -1 "$OUT/result.txt"
}
snapshot_state() {          # 현재 route/mode/속도 한 줄
  local r m v
  r=$(timeout 5 ros2 topic echo /api/routing/state --once --qos-durability transient_local --qos-reliability reliable --field state 2>/dev/null | head -1)
  m=$(timeout 5 ros2 topic echo /api/operation_mode/state --once --qos-durability transient_local --qos-reliability reliable --field mode 2>/dev/null | head -1)
  v=$(timeout 5 ros2 topic echo /vehicle/status/velocity_status --once --field longitudinal_velocity 2>/dev/null | head -1)
  echo "route=$r mode=$m v=$v"
}
stack_stop() {              # bag → 스택(정리 훅) → 잔존 강제(이 클론 경로로 한정) → mock 순으로 종료
  bag_stop
  # TERM 을 쓴다: 비대화형 셸이 백그라운드로 띄운 스크립트는 SIGINT 가 무시 상태로 상속되어 trap INT 가 안 걸린다(9/7 실측: INT 후 90 s 잔존 25).
  [ -n "$STACK_PID" ] && kill -TERM "$STACK_PID" 2>/dev/null
  for i in $(seq 1 60); do kill -0 "$STACK_PID" 2>/dev/null || break; sleep 0.5; done
  kill -KILL "$STACK_PID" 2>/dev/null
  harness_kill_leftovers
  pkill -f "mock_vtd.py --port 991[0]" 2>/dev/null; sleep 1
  cp "$HOME/hlfma/logs/bridge_latest.log" "$OUT/bridge.log" 2>/dev/null
  grep -E "경로 설정|경로 거부|차선변경:|goal:|점 [0-9]+:|리스폰|점프|WARN|ERROR" "$OUT/bridge.log" 2>/dev/null | head -40 > "$OUT/bridge_summary.txt"
}
bag_stop() {                # recorder 종료: TERM → 5 s 대기 → KILL. (INT 만으론 안 죽는 경우가 있었음 9/7)
  local pids; pids="$(pgrep -f "ros2 bag recor[d] .*-o $OUT/bag")"
  [ -f "$OUT/bag.pid" ] && pids="$pids $(cat "$OUT/bag.pid")"
  for p in $pids; do kill -TERM "$p" 2>/dev/null; done
  for i in $(seq 1 10); do alive=0; for p in $pids; do kill -0 "$p" 2>/dev/null && alive=1; done; [ "$alive" = 0 ] && break; sleep 0.5; done
  for p in $pids; do kill -KILL "$p" 2>/dev/null; done
}
harness_kill_leftovers() {  # 이 클론의 launch 그룹·install 경로 노드만 정리 (다른 클론·rviz 는 안 건드림)
  for p in $(pgrep -f "map_path:=$ROOT/ma[p]"); do g=$(ps -o pgid= -p "$p" | tr -d " "); [ -n "$g" ] && kill -INT -- "-$g" 2>/dev/null; done
  sleep 3
  for p in $(pgrep -f "map_path:=$ROOT/ma[p]"); do g=$(ps -o pgid= -p "$p" | tr -d " "); [ -n "$g" ] && kill -KILL -- "-$g" 2>/dev/null; done
  pkill -KILL -f "$ROOT/hlfma_ws/instal[l]/" 2>/dev/null
  # 이 하네스의 recorder 잔존 (어느 케이스든) — 다른 클론·사용자 recorder 는 경로가 달라 제외
  for p in $(pgrep -f "ros2 bag recor[d] .*-o $HOME/hlfma/logs/harness/"); do kill -TERM "$p" 2>/dev/null; done; sleep 2
  for p in $(pgrep -f "ros2 bag recor[d] .*-o $HOME/hlfma/logs/harness/"); do kill -KILL "$p" 2>/dev/null; done
  local n; n=$(pgrep -fc "map_path:=$ROOT/ma[p]|$ROOT/hlfma_ws/instal[l]/")
  [ "$n" = "0" ] || echo "  [WARN] 잔존 프로세스 $n (이 클론)"
}
metrics() {                 # $@ = metrics.py 추가 인자 → metrics.json
  timeout 120 python3 "$ROOT/tools/harness/metrics.py" "$OUT" "$@" | tee "$OUT/metrics.txt"
}
mget() { timeout 10 python3 -c "import json,sys; d=json.load(open('$OUT/metrics.json')); v=d$1; print('' if v is None else v)" 2>/dev/null; }

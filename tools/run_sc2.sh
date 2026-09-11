#!/usr/bin/env bash
# 시나리오 2 실주행 1회를 끝까지 자동으로 돌리고 한 줄 요약까지 낸다.
#   VTD 시나리오 로드/시작 -> rviz -> 브리지+Autoware -> engage -> 주행 -> 녹화 종료 -> 요약
#
#   usage: run_sc2.sh [duration_s] [label] [--no-rviz]
#   exit : 0 성공 / 1 실패 (실패 사유는 마지막 줄 RESULT= 에 담긴다)
#
# 설계 원칙: "띄우고 나서 확인" 이 아니라 "확인될 때까지 기다리고, 안 되면 정해진 횟수만큼
# 스스로 다시 시도" 한다. 오늘 겪은 실패들을 전부 사전에 막는다:
#   - 직전 런 프로세스가 하나라도 남으면 다음 스택의 map_container 가 멈춘다 -> PID 로 확실히 종료 + 대기
#   - /dev/shm 에 죽은 fastrtps 파일이 쌓이면 DDS 디스커버리가 깨진다 -> 프로세스 0일 때만 정리
#   - VTD 는 포트가 열려도 시나리오가 Running 이 아니면 데이터를 안 보낸다 -> 실제 바이트 수신을 확인
#   - 맵 로딩 중 재생/engage 를 시작하면 런이 통째로 버려진다 -> WaitingForEngage 를 기다린다
#   - setsid 로 띄운 레코더는 $! 가 래퍼라 SIGINT 가 안 닿는다 -> 일반 백그라운드로 띄워 PID 확보
ROOT=/home/a/2026-HL-FMA-VTD-NG
VTD_HOST=192.168.50.11
SCENARIO=HL_FMA_VTD_LivingLab_2.xml
ROUTE_CSV=/home/a/hlfma/route/route_pretest_2.csv

DUR="${1:-150}"
LABEL="${2:-sc2}"
USE_RVIZ=1
[ "$3" = "--no-rviz" ] && USE_RVIZ=0

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
source /opt/ros/jazzy/setup.bash
source "$ROOT/hlfma_ws/install/setup.bash"

TS=$(date +%m%d_%H%M%S)
OUT="$ROOT/test/sc2_${TS}_${LABEL}"
mkdir -p "$OUT"
RESULT="UNKNOWN"
finish() { echo "RESULT=$RESULT OUT=$OUT"; [ "$RESULT" = "OK" ] && exit 0 || exit 1; }

# ── 1. 잔존 프로세스 정리 ──────────────────────────────────────────────
# pkill -f 는 이 스크립트 자신의 커맨드라인도 매치하므로 PID 를 모아 자기/부모를 뺀다.
kill_all() {
  local me=$$ pp=$PPID p
  for p in $( { pgrep -f "autoware[.]launch[.]xml"; \
                pgrep -f "rclcpp_components/component_container"; \
                pgrep -f "$ROOT/hlfma_ws/install/"; \
                pgrep -f "vtd_autoware_bridge"; \
                pgrep -f "vtd_route_node"; \
                pgrep -f "start_autonomous"; \
                pgrep -f "start_hlfma"; \
                pgrep -f "ros2 bag record"; \
                pgrep -f "robot_state_publisher"; \
                pgrep -f "ros2 launch"; \
                pgrep -f "rviz2"; } 2>/dev/null | sort -u ); do
    [ "$p" = "$me" ] || [ "$p" = "$pp" ] || kill -KILL "$p" 2>/dev/null
  done
}
# robot_state_publisher 는 /opt/ros 아래에 있어 install 경로 패턴에 안 걸린다.
# 남아 있으면 새 스택과 중복돼 TF 가 꼬이고 AutowareState 가 Planning 에서 멈춘다
# (실측: "node is duplicated" 와 함께 WaitingForEngage 로 영영 못 넘어감).
echo "[sc2] 정리"
kill_all
sleep 4
kill_all
sleep 4
if [ "$(pgrep -c -f 'rclcpp_components/component_container' 2>/dev/null || echo 0)" = "0" ]; then
  find /dev/shm -maxdepth 1 -name "fastrtps_*" -delete 2>/dev/null
  find /dev/shm -maxdepth 1 -name "_port*_el" -delete 2>/dev/null
fi
sleep 4

# ── 2. VTD 시나리오 로드/시작 + 실제 스트리밍 확인 ─────────────────────
vtd_streaming() {
  python3 - "$VTD_HOST" <<'PY'
import socket, sys
try:
    s = socket.create_connection((sys.argv[1], 9910), timeout=3)
    s.settimeout(3)
    d = s.recv(4096)
    s.close()
    sys.exit(0 if d else 1)
except Exception:
    sys.exit(1)
PY
}
echo "[sc2] VTD 시나리오 시작"
VTD_OK=0
for attempt in 1 2; do
  (cd "$ROOT/tools" && timeout 180 python3 lab_restart_scenario.py "$VTD_HOST" "$SCENARIO") \
    > "$OUT/vtd.log" 2>&1
  for i in $(seq 1 30); do
    if vtd_streaming; then VTD_OK=1; break; fi
    sleep 2
  done
  [ "$VTD_OK" = "1" ] && break
  echo "[sc2] VTD 스트리밍 없음 - 재시도 $attempt"
done
if [ "$VTD_OK" != "1" ]; then RESULT="FAIL_VTD"; finish; fi
echo "[sc2] VTD 스트리밍 확인"

# ── 3. rviz (실패해도 주행에는 영향 없음) ──────────────────────────────
if [ "$USE_RVIZ" = "1" ]; then
  # 로그인 세션의 디스플레이를 찾는다. ssh 세션에는 DISPLAY 가 없어 그냥 띄우면 죽는다.
  DISP=$(who 2>/dev/null | grep -oE "\(:[0-9]+\)" | head -1 | tr -d "():")
  [ -z "$DISP" ] && DISP=$(ls /tmp/.X11-unix/ 2>/dev/null | head -1 | sed "s/^X/:/")
  [ -z "$DISP" ] && DISP=":0"
  setsid env DISPLAY="$DISP" "$ROOT/rviz.sh" > "$OUT/rviz.log" 2>&1 < /dev/null &
  echo "[sc2] rviz DISPLAY=$DISP"
fi

# ── 4. 브리지 + Autoware ───────────────────────────────────────────────
echo "[sc2] 스택 기동"
setsid env ROUTE_CSV="$ROUTE_CSV" "$ROOT/start_autonomous.sh" > "$OUT/stack.log" 2>&1 < /dev/null &
# WaitingForEngage 는 맵·경로·자차 위치가 전부 준비돼야 나온다. 하나라도 빠지면 안 뜬다.
READY=0
for i in $(seq 1 240); do
  if grep -q "AutowareState: .*=> WaitingForEngage" "$OUT/stack.log" 2>/dev/null; then
    READY=1; echo "[sc2] 준비됨 (${i}s)"; break
  fi
  sleep 1
done
if [ "$READY" != "1" ]; then RESULT="FAIL_STACK"; kill_all; finish; fi

SEG=$(grep -oE "Received new route with [0-9]+ segments" "$OUT/stack.log" | tail -1 | grep -oE "[0-9]+" | head -1)
[ -z "$SEG" ] && SEG=0

# ── 5. 녹화 (setsid 금지: $! 가 레코더 자신이어야 SIGINT 가 닿는다) ────
ros2 bag record -o "$OUT/bag" \
  /localization/kinematic_state \
  /perception/object_recognition/objects \
  /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id \
  /planning/scenario_planning/trajectory \
  /planning/cooperate_status/lane_change_left \
  /planning/cooperate_status/lane_change_right \
  /planning/cooperate_status/external_request_lane_change_left \
  /planning/cooperate_status/external_request_lane_change_right \
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/internal_state \
  /vehicle/status/steering_status \
  /control/command/control_cmd \
  > "$OUT/record.log" 2>&1 &
REC=$!
sleep 3

# ── 6. engage ─────────────────────────────────────────────────────────
"$ROOT/start_hlfma.sh" > "$OUT/engage.log" 2>&1
if ! grep -q "success=True" "$OUT/engage.log"; then
  RESULT="FAIL_ENGAGE"; kill -INT "$REC" 2>/dev/null; sleep 3; kill_all; finish
fi
echo "[sc2] engage OK, ${DUR}s 주행"
sleep "$DUR"

# ── 7. 종료 및 요약 ───────────────────────────────────────────────────
kill -INT "$REC" 2>/dev/null
for i in $(seq 1 20); do kill -0 "$REC" 2>/dev/null || break; sleep 1; done
kill_all
sleep 3
ros2 bag reindex "$OUT/bag" -s mcap > /dev/null 2>&1
cp -f "$HOME/hlfma/logs/autoware_latest.log" "$OUT/autoware.log" 2>/dev/null
cp -f "$HOME/hlfma/logs/bridge_latest.log" "$OUT/bridge.log" 2>/dev/null

echo "--- 요약 ---"
echo "route segments: $SEG"
python3 "$ROOT/tools/eval_run.py" "$OUT/bag" 2>/dev/null | grep -v "^\[WARN"
echo "lc_transit:"; grep -o "HLFMA lc_transit:.*" "$OUT/autoware.log" 2>/dev/null | sed "s/HLFMA lc_transit: //" | sort | uniq -c | sed "s/^/  /"
echo "path_miss=$(grep -c 'Path miss detected' "$OUT/autoware.log" 2>/dev/null) detour_req=$(grep -c 'DETOUR request' "$OUT/bridge.log" 2>/dev/null)"
RESULT="OK"
finish

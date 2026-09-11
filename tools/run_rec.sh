#!/usr/bin/env bash
# 기록 주행 — 기동·검증·녹화·종료·요약을 한 폴더에 남긴다.
#
#   usage:  tools/run_rec.sh <태그> [주행초=420]
#   env:    RESET=1   VTD 시나리오를 처음으로 되돌리고 시작 (tools/reset_sim.sh, **연구실 전용**)
#           DEBUG=0   플래너 디버그 로거(PLAN_*/LC_*) 끄기 (기본 1 = 켬. 대회 당일은 0)
#           PCAP=0    VTD 이더넷 pcap 생략 (기본 1)
#           RVIZ=1    rviz 도 띄움 (기본 0)
#
#   결과:  test/<MMDD_HHMMSS>_<태그>/
#     meta.txt        git rev·경로 CSV·환경
#     start.log       start.sh 출력 (engage 결과 포함)
#     bag/            정선 토픽 rosbag (아래 REC_RE)
#     trace.jsonl     tools/trace.py 계측 → score.txt (tools/score.py)
#     net/vtd.pcap    VTD 패킷 원본
#     logs/           bridge.log autoware.log engage.log ros/
#     result.txt      OK / ARRIVED / FAIL_* 와 종료 사유
#
#   종료: ARRIVED(/api/routing/state=3) 감지, 주행초 만료, 또는 Ctrl+C — 모두 같은 정리 절차.
set -o pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TAG="${1:?태그를 주세요 (예: sc1)}"; DUR="${2:-420}"
P="[rec]"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
source /opt/ros/jazzy/setup.bash
source "$ROOT/hlfma_ws/install/setup.bash"

# 중복 실행 방지 (부채 4-3: 두 번째 인스턴스의 stop.sh 가 첫 주행을 죽인다)
LOCK=/tmp/run_rec.lock
if [ -f "$LOCK" ] && kill -0 "$(cat "$LOCK")" 2>/dev/null; then
  echo "$P 이미 실행 중 (pid $(cat "$LOCK")). 중단." >&2; exit 1
fi
echo $$ > "$LOCK"

OUT="$ROOT/test/$(date +%m%d_%H%M%S)_$TAG"
mkdir -p "$OUT/net" "$OUT/logs"
echo "$OUT" > /tmp/run_rec.outdir
ROUTE_CSV="${ROUTE_CSV:-$(sed -n 's/^csv_path:[[:space:]]*//p' "$HOME/hlfma/route/route_config.yaml" | head -1)}"
{
  echo "start=$(date -Is)"; echo "git_rev=$(git -C "$ROOT" rev-parse --short HEAD)"
  echo "git_dirty=$(git -C "$ROOT" status --short | wc -l)"; echo "route=$ROUTE_CSV"
  echo "dur=$DUR debug=${DEBUG:-1} pcap=${PCAP:-1} reset=${RESET:-0} rviz=${RVIZ:-0}"
} > "$OUT/meta.txt"
echo "$P → $OUT"
echo "$P rev=$(git -C "$ROOT" rev-parse --short HEAD) route=$(basename "$ROUTE_CSV") dur=${DUR}s"

RESULT="INCOMPLETE"; REASON=""
BAG_PID=""; TRACE_PID=""; PCAP_PID=""; START_PID=""
finish() {
  trap - EXIT INT TERM
  echo "$P 정리: $RESULT ${REASON:+($REASON)}"
  [ -n "$TRACE_PID" ] && kill -INT "$TRACE_PID" 2>/dev/null
  [ -n "$BAG_PID" ]   && { kill -INT "$BAG_PID" 2>/dev/null; for i in $(seq 1 20); do kill -0 "$BAG_PID" 2>/dev/null || break; sleep 1; done; }
  [ -n "$PCAP_PID" ]  && { kill -INT "$PCAP_PID" 2>/dev/null; wait "$PCAP_PID" 2>/dev/null; }
  "$ROOT/stop.sh" > "$OUT/stop.log" 2>&1
  sleep 2
  # start.sh 가 남긴 매니페스트로 이번 판의 로그를 복사
  if [ -f "$HOME/hlfma/logs/run_latest.env" ]; then
    # shellcheck disable=SC1090
    source "$HOME/hlfma/logs/run_latest.env"
    cp -f "$BRIDGE_LOG" "$OUT/logs/bridge.log" 2>/dev/null
    cp -f "$AW_LOG" "$OUT/logs/autoware.log" 2>/dev/null
    cp -f "$HOME/hlfma/logs/engage_${RUN_TS}.log" "$OUT/logs/engage.log" 2>/dev/null
    cp -r "$ROS_LOG_DIR" "$OUT/logs/ros" 2>/dev/null
  fi
  [ -d "$OUT/bag" ] && ros2 bag reindex "$OUT/bag" > /dev/null 2>&1
  { echo "result=$RESULT"; echo "reason=$REASON"; echo "end=$(date -Is)"; } > "$OUT/result.txt"
  echo "end=$(date -Is) result=$RESULT" >> "$OUT/meta.txt"
  if [ -s "$OUT/trace.jsonl" ]; then
    python3 "$ROOT/tools/score.py" "$OUT/trace.jsonl" > "$OUT/score.txt" 2>&1
    echo "--- score.py ---"; cat "$OUT/score.txt"
  fi
  echo "$P 완료: $OUT ($(du -sh "$OUT" 2>/dev/null | cut -f1))  result=$RESULT"
  rm -f "$LOCK"
  exit 0
}
trap finish EXIT INT TERM

# ── 0. 사전 점검: VTD 9910 열림 / 이 디렉터리 스택 없음 ────────────────
if ! timeout 3 bash -c 'cat < /dev/null > /dev/tcp/192.168.50.11/9910' 2>/dev/null; then
  RESULT="FAIL_VTD"; REASON="192.168.50.11:9910 닫힘 — VTD 미기동"; finish
fi
if pgrep -f "map_path:=$ROOT/map" >/dev/null; then
  RESULT="FAIL_BUSY"; REASON="이 디렉터리의 Autoware 가 이미 떠 있음 (./stop.sh 먼저)"; finish
fi

# ── 1. (선택) VTD 시나리오 리셋 — 연구실 전용 ───────────────────────────
if [ "${RESET:-0}" = "1" ]; then
  echo "$P VTD 시나리오 리셋"; "$ROOT/tools/reset_sim.sh" > "$OUT/reset.log" 2>&1; sleep 2
fi

# ── 2. 플래너 디버그 로거 (behavior_planning.launch.xml 이 이 env 를 읽는다) ──
if [ "${DEBUG:-1}" = "1" ]; then
  export HLFMA_LOG_LEVEL=debug        # behavior_planning.launch.xml 의 플래너 로거 9종
  echo "$P 플래너 디버그 로거 ON"
fi

# ── 3. pcap (tcpdump 는 SIGINT 를 무시하도록 시작되므로 서브셸에서 되살린다) ──
if [ "${PCAP:-1}" = "1" ]; then
  IFACE="$(ip route get 192.168.50.11 2>/dev/null | sed -n 's/.* dev \([^ ]*\).*/\1/p' | head -1)"
  ( trap - INT; exec tcpdump -i "${IFACE:-any}" -n -s 0 -w "$OUT/net/vtd.pcap" "host 192.168.50.11" ) </dev/null >"$OUT/net/tcpdump.err" 2>&1 &
  PCAP_PID=$!; sleep 1
  kill -0 "$PCAP_PID" 2>/dev/null && echo "$P pcap: net/vtd.pcap ($IFACE)" || { echo "$P pcap 실패: $(tail -1 "$OUT/net/tcpdump.err")"; PCAP_PID=""; }
fi

# ── 4. rviz (선택) ───────────────────────────────────────────────────────
if [ "${RVIZ:-0}" = "1" ]; then
  DISP=$(who 2>/dev/null | grep -oE "\(:[0-9]+\)" | head -1 | tr -d "()"); [ -z "$DISP" ] && DISP=":1"
  setsid env DISPLAY="$DISP" "$ROOT/rviz.sh" > "$OUT/rviz.log" 2>&1 < /dev/null &
  echo "$P rviz DISPLAY=$DISP"
fi

# ── 5. 스택 기동 (start.sh: 브리지+Autoware → 경로 → engage 까지) ──────────
setsid env ROUTE_CSV="$ROUTE_CSV" "$ROOT/start.sh" > "$OUT/start.log" 2>&1 < /dev/null &
START_PID=$!
echo "$P 스택 기동 (pid $START_PID)"

# ── 6. bag — engage 전에 붙어야 출발 순간이 남는다 ─────────────────────────
sleep 8
REC_RE='^/localization/kinematic_state$'
REC_RE+='|^/tf$|^/tf_static$'
REC_RE+='|^/perception/object_recognition/objects$'
REC_RE+='|^/perception/traffic_light_recognition/traffic_signals$'
REC_RE+='|^/planning/scenario_planning/lane_driving/behavior_planning/(path|path_with_lane_id)$'
REC_RE+='|^/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/internal_state$'
REC_RE+='|^/planning/scenario_planning/lane_driving/motion_planning/.*/trajectory$'
REC_RE+='|^/planning/scenario_planning/(trajectory|max_velocity.*|current_max_velocity|clear_velocity_limit)$'
REC_RE+='|^/planning/trajectory$|^/planning/mission_planning/route$|^/planning/validation_status$'
REC_RE+='|^/planning/planning_factors/.*|^/planning/cooperate_(status|commands)/.*'
REC_RE+='|^/vehicle/status/.*'
REC_RE+='|^/control/command/(control_cmd|turn_indicators_cmd|hazard_lights_cmd)$'
REC_RE+='|^/system/(operation_mode/state|fail_safe/mrm_state|emergency/.*)$'
REC_RE+='|^/api/(routing|operation_mode|planning)/.*'
REC_RE+='|^/vtd/.*|^/diagnostics$'
REC_RE+='|^/control/.*'                                   # 제어기 내부(경사 보상 진단 등)
REC_RE+='|^/planning/.*virtual_wall.*|^/planning/velocity_factors'  # 정지 사유 가상벽
( trap - INT; exec ros2 bag record -o "$OUT/bag" -e "$REC_RE" ) </dev/null >"$OUT/record.log" 2>&1 &
BAG_PID=$!
echo "$P bag 시작 (pid $BAG_PID)"

# ── 7. trace.py (JG 계측 — score.py 입력) ──────────────────────────────────
setsid python3 "$ROOT/tools/trace.py" "$OUT/trace.jsonl" $((DUR + 120)) > "$OUT/trace.log" 2>&1 < /dev/null &
TRACE_PID=$!

# ── 8. 기동 검증 (최대 120초): autoware 로그의 "Loaded node" 로 제어 노드 4개 + bridge 시작 앵커 ──
#    ros2 node list 는 데몬 상태에 따라 비기도 하므로(부채 4-4) 로그를 1차 근거로 쓴다.
BR="$(sed -n "s/^BRIDGE_LOG=//p" "$HOME/hlfma/logs/run_latest.env" 2>/dev/null)"
AW="$(sed -n "s/^AW_LOG=//p" "$HOME/hlfma/logs/run_latest.env" 2>/dev/null)"
OK4=0; T0=$(date +%s)
while [ $(( $(date +%s) - T0 )) -lt 112 ]; do
  OK4=0
  for n in operation_mode_transition_manager shift_decider vehicle_cmd_gate controller_node_exe; do
    grep -aq "Loaded node.*$n" "$AW" 2>/dev/null && OK4=$((OK4+1))
  done
  [ "$OK4" -eq 4 ] && grep -aq "시작 앵커" "$BR" 2>/dev/null && break
  sleep 5
done
EL=$(( $(date +%s) - T0 + 8 ))
timeout 15 ros2 node list --no-daemon > "$OUT/nodes_${EL}s.txt" 2>&1
if [ "$OK4" -ne 4 ]; then
  RESULT="FAIL_STACK"; REASON="${EL}초까지 제어 노드 로드 $OK4/4 (autoware.log Loaded node 기준)"; finish
fi
if ! grep -aq "시작 앵커" "$BR" 2>/dev/null; then
  RESULT="FAIL_ROUTE"; REASON="${EL}초까지 route_node 시작 앵커 없음 (bridge.log 확인)"; finish
fi
echo "$P 기동 검증 통과 ${EL}s (제어 노드 4/4, 시작 앵커 있음)"

# ── 9. engage 결과 대기 (start.sh 가 최대 120s+60s 기다린다) ───────────────
ENG=""
for i in $(seq 1 200); do
  if grep -q "operation mode=2" "$OUT/start.log"; then ENG=OK; break; fi
  if grep -q "FAILED_FLAG\|경로가 SET(2) 이 아님\|서비스가 없다\|서비스 응답 없음" "$OUT/start.log"; then ENG=FAIL; break; fi
  sleep 1
done
if [ "$ENG" != "OK" ]; then
  RESULT="FAIL_ENGAGE"; REASON="$(grep -m1 "\[engage\]" "$OUT/start.log" | tail -1)"; finish
fi
T_GO=$(date +%s)
echo "$P engage OK — 주행 중 (최대 ${DUR}s, ARRIVED 감지 시 조기 종료, Ctrl+C 가능)"

# ── 10. 주행: ARRIVED(3) 감지 또는 시간 만료 ──────────────────────────────
while :; do
  el=$(( $(date +%s) - T_GO ))
  if [ "$el" -ge "$DUR" ]; then RESULT="TIMEOUT"; REASON="${DUR}s 만료"; break; fi
  st="$(timeout 8 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /api/routing/state autoware_adapi_v1_msgs/msg/RouteState --field state 2>/dev/null | tr -d '[:space:]')"
  if [ "$st" = "3" ]; then RESULT="ARRIVED"; REASON="t=${el}s"; echo "$P ARRIVED t=${el}s"; sleep 5; break; fi
  if ! kill -0 "$START_PID" 2>/dev/null; then RESULT="FAIL_STACK"; REASON="start.sh 종료됨 t=${el}s"; break; fi
  [ $((el % 30)) -lt 10 ] && echo "$P t=${el}s routing=$st"
  sleep 10
done
finish

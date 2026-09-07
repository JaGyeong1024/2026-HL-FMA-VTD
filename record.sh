#!/usr/bin/env bash
# 주행 기록: VTD 이더넷 원본(pcap) + 네트워크 상태 + 실행 로그를 한 폴더에 모은다. (bag 은 선택)
#
# 사용:  ./record.sh [태그]     → <repo>/test/<YYYYMMDD_HHMMSS>[_태그]/
#        Ctrl+C 로 종료. 시작 전에 ./start_autonomous.sh 가 떠 있어야 로그가 복사된다.
#        기본으로 ros2 bag 에 /vtd/raw_rx /vtd/raw_tx 도 기록한다.
#        BAG=0 ./record.sh      → bag 없이 pcap·네트워크·로그만
#        ALL=1 ./record.sh      → bag 에 전 토픽 기록 (용량 큼, 타이밍 문제 증거 보존용)
#
# 폴더 구성:
#   net/vtd.pcap    VTD 호스트와 주고받은 모든 패킷(TCP 9910 상태·제어, RTSP 8554, UDP 9912 …) 커널 타임스탬프 포함
#                   → 재생 서버로 VTD 대신 흘려 코드 재검증 (payload 추출: tshark -r vtd.pcap -q -z follow,tcp,raw,0)
#                   1회 설정 필요: sudo setcap cap_net_raw,cap_net_admin=eip /usr/bin/tcpdump
#   net/start.txt end.txt   ip addr/route/link 통계, ethtool, ss -tnpi, nstat, ping
#   net/ss_vtd.log          VTD TCP 소켓 rtt/retrans/cwnd 1초 샘플링
#   logs/           bridge.log autoware.log ros/(노드별 로그·launch.log) record.log
#   meta.txt        시각·호스트·git rev·ROS 환경·start_autonomous.sh 실행 정보(run_latest.env)
#   bag/            /vtd/raw_rx /vtd/raw_tx (ALL=1 이면 전 토픽, BAG=0 이면 없음)
set -o pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
BAG_MODE="raw"; [ "${BAG:-1}" = "0" ] && BAG_MODE="none"; [ "${ALL:-0}" = "1" ] && BAG_MODE="all"
if [ "$BAG_MODE" != "none" ]; then
  source /opt/ros/jazzy/setup.bash
  source "$ROOT/hlfma_ws/install/setup.bash"
fi

TS="$(date +%Y%m%d_%H%M%S)"
OUT="$ROOT/test/${TS}${1:+_$1}"
mkdir -p "$OUT/net" "$OUT/logs"
LOG="$OUT/logs/record.log"
log() { echo "[record $(date +%H:%M:%S)] $*" | tee -a "$LOG"; }
log "→ $OUT   (Ctrl+C 로 종료)"

# ── 실행 정보 (start_autonomous.sh 가 남긴 매니페스트) ─────────────
RUN_ENV="$HOME/hlfma/logs/run_latest.env"
VTD_HOST=192.168.50.11; BRIDGE_LOG=""; AW_LOG=""; ROS_LOG_DIR_RUN=""
if [ -f "$RUN_ENV" ]; then
  # shellcheck disable=SC1090
  source "$RUN_ENV"; ROS_LOG_DIR_RUN="$ROS_LOG_DIR"
else
  log "경고: $RUN_ENV 없음 — start_autonomous.sh 가 안 떠 있거나 구버전. 로그 복사 생략"
fi
{
  echo "record_start=$(date -Is)"; echo "host=$(hostname)"; echo "user=$USER"
  echo "uname=$(uname -srm)"; echo "git_rev=$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null)"
  echo "git_dirty=$(git -C "$ROOT" status --short 2>/dev/null | wc -l)"
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"; echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-default}"
  echo "bag_mode=$BAG_MODE"
  echo "--- run_latest.env ---"; cat "$RUN_ENV" 2>/dev/null
} > "$OUT/meta.txt"

# ── 네트워크 스냅샷 ────────────────────────────────────────────────
IFACE="$(ip route get "$VTD_HOST" 2>/dev/null | sed -n 's/.* dev \([^ ]*\).*/\1/p' | head -1)"
net_snapshot() {  # $1 = 파일
  {
    echo "### $(date -Is)  vtd_host=$VTD_HOST iface=${IFACE:-?}"
    echo "### ip -br addr";      ip -br addr
    echo "### ip route";         ip route
    echo "### ip -s link";       ip -s link
    [ -n "$IFACE" ] && { echo "### ethtool $IFACE"; ethtool "$IFACE" 2>&1; ethtool -S "$IFACE" 2>&1 | head -60; ethtool -k "$IFACE" 2>&1 | grep -E "offload|scatter" ; }
    echo "### ss -tnpi (VTD 소켓)"; ss -tnpi "dst $VTD_HOST or src $VTD_HOST" 2>&1
    echo "### ss -s";            ss -s
    echo "### nstat -az (TCP/IP 누적 카운터)"; nstat -az 2>/dev/null | grep -E "Tcp|Ip|Udp" 
    echo "### ping $VTD_HOST";   ping -c 5 -i 0.2 -W 1 "$VTD_HOST" 2>&1 | tail -3
  } > "$1" 2>&1
}
net_snapshot "$OUT/net/start.txt"
log "net: iface=${IFACE:-?}  $(sed -n 's/.*\(rtt.*\)/\1/p' "$OUT/net/start.txt" | head -1)"

# 소켓 상태 1초 샘플링 (rtt, retrans, cwnd, 버퍼) — 통신 끊김·지연 분석용
( while true; do
    echo "### $(date +%H:%M:%S.%N | cut -c1-12)"
    ss -tnpi "dst $VTD_HOST or src $VTD_HOST" 2>/dev/null | tail -n +2
    sleep 1
  done ) > "$OUT/net/ss_vtd.log" 2>&1 &
SS_PID=$!

# pcap — 기본 기록물. tcpdump 는 스크립트 자식이라 SIGINT 를 무시하므로 서브셸에서 되살린다
( trap - INT; exec tcpdump -i "${IFACE:-any}" -n -s 0 -w "$OUT/net/vtd.pcap" "host $VTD_HOST" ) </dev/null >"$OUT/net/tcpdump.err" 2>&1 &
TCPDUMP_PID=$!
sleep 1
if ! kill -0 "$TCPDUMP_PID" 2>/dev/null; then
  log "오류: tcpdump 실행 실패 → $(tail -1 "$OUT/net/tcpdump.err")"
  log "      권한이면 1회:  sudo setcap cap_net_raw,cap_net_admin=eip /usr/bin/tcpdump"
  kill "$SS_PID" 2>/dev/null; exit 1
fi
log "pcap: net/vtd.pcap (iface=${IFACE:-any}, host $VTD_HOST)"

# ── 종료 처리: bag·샘플러·pcap 정지 후 로그 복사 ───────────────────
BAG_PID=""
finish() {
  trap - EXIT INT TERM
  log "정리 중..."
  [ -n "$BAG_PID" ] && { kill -INT "$BAG_PID" 2>/dev/null; wait "$BAG_PID" 2>/dev/null; }
  kill "$SS_PID" 2>/dev/null
  kill -INT "$TCPDUMP_PID" 2>/dev/null; wait "$TCPDUMP_PID" 2>/dev/null
  log "pcap: $(tail -3 "$OUT/net/tcpdump.err" | head -1)"
  net_snapshot "$OUT/net/end.txt"
  [ -n "$BRIDGE_LOG" ] && [ -f "$BRIDGE_LOG" ] && cp "$BRIDGE_LOG" "$OUT/logs/bridge.log"
  [ -n "$AW_LOG" ]     && [ -f "$AW_LOG" ]     && cp "$AW_LOG"     "$OUT/logs/autoware.log"
  [ -n "$ROS_LOG_DIR_RUN" ] && [ -d "$ROS_LOG_DIR_RUN" ] && cp -r "$ROS_LOG_DIR_RUN" "$OUT/logs/ros"
  echo "record_end=$(date -Is)" >> "$OUT/meta.txt"
  pkill -P $$ sleep 2>/dev/null
  log "완료: $OUT  ($(du -sh "$OUT" 2>/dev/null | cut -f1))"
  ls "$OUT/logs" | sed 's/^/  logs\//' | tee -a "$LOG"
  exit 0
}
trap finish EXIT INT TERM

# ── bag (선택) ────────────────────────────────────────────────────
if [ "$BAG_MODE" != "none" ]; then
  # 스크립트의 백그라운드 자식은 SIGINT 를 무시하도록 시작되므로 서브셸에서 되살린다
  # (set -m 은 쓰지 않는다: 터미널에서 bag 이 키 입력을 읽다 SIGTTIN 으로 정지 → 즉시 종료 판정)
  if [ "$BAG_MODE" = "all" ]; then
    ( trap - INT; exec ros2 bag record --node-name "rec_$(basename "$OUT" | tr -c "A-Za-z0-9_" "_")" -a --include-hidden-topics -o "$OUT/bag" ) </dev/null >>"$LOG" 2>&1 &
  else
    ( trap - INT; exec ros2 bag record --node-name "rec_$(basename "$OUT" | tr -c "A-Za-z0-9_" "_")" --topics /vtd/raw_rx /vtd/raw_tx -o "$OUT/bag" ) </dev/null >>"$LOG" 2>&1 &
  fi
  BAG_PID=$!
  log "bag: $([ "$BAG_MODE" = all ] && echo 전토픽 || echo '/vtd/raw_rx /vtd/raw_tx') pid=$BAG_PID"
fi
log "기록 중... (Ctrl+C 로 종료)"
while true; do sleep 3600 & wait $!; done

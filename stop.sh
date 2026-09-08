#!/usr/bin/env bash
# 전체 정지: **이 디렉터리의** 브리지·route_node·판단노드·Autoware 를 확실히 종료.
# (start.sh 의 종료가 미진할 때 쓴다)
#
# 9/7: 예전에는 pkill 패턴이 경로로 한정되지 않아 다른 클론(-NG/-MY)과 사용자가 띄운 rviz 까지
#      죽였다. 대회 당일 다른 창을 끄는 사고가 되므로 전부 이 디렉터리 기준으로 좁혔다.
#      rviz 는 주행에 영향이 없으므로 여기서 끄지 않는다 (필요하면 창을 직접 닫는다).
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATTERNS=("$ROOT/hlfma_ws/install/" "map_path:=$ROOT/map")

kill_all() {   # $1 = 신호
  for pat in "${PATTERNS[@]}"; do
    for p in $(pgrep -f "$pat" 2>/dev/null); do
      [ "$p" = "$$" ] && continue
      kill "-$1" "$p" 2>/dev/null
    done
  done
}
alive() { for pat in "${PATTERNS[@]}"; do pgrep -f "$pat" >/dev/null 2>&1 && return 0; done; return 1; }

kill_all INT
for i in $(seq 1 16); do alive || break; sleep 0.5; done
kill_all KILL
sleep 1

n=0
for pat in "${PATTERNS[@]}"; do n=$((n + $(pgrep -f "$pat" 2>/dev/null | wc -l))); done
echo "정지 완료 ($ROOT). 잔존 $n개"
if [ "$n" -gt 0 ]; then
  for pat in "${PATTERNS[@]}"; do pgrep -af "$pat" 2>/dev/null | cut -c1-90; done
fi
# 참고: 다른 클론 프로세스는 건드리지 않는다.
other=$(pgrep -af "2026-HL-FMA-VTD-(NG|MY)/hlfma_ws/install" 2>/dev/null | wc -l)
[ "$other" -gt 0 ] && echo "(다른 클론 프로세스 ${other}개는 유지)"
exit 0

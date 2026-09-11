#!/usr/bin/env bash
# 시나리오 2 를 N회 반복 실행하고 회차별 요약을 한 파일에 모은다.
#   usage: repeat_sc2.sh [N] [duration_s] [label]
# 실패한 회차는 최대 1회 재시도한다(VTD/스택 기동 플레이크 대비).
ROOT=/home/a/2026-HL-FMA-VTD-NG
N="${1:-4}"
DUR="${2:-150}"
LABEL="${3:-rep}"
SUM=/tmp/sc2_summary.txt
: > "$SUM"
for i in $(seq 1 "$N"); do
  echo "########## 회차 $i/$N ##########" >> "$SUM"
  for try in 1 2; do
    "$ROOT/tools/run_sc2.sh" "$DUR" "${LABEL}$i" --no-rviz >> "$SUM" 2>&1
    rc=$?
    [ $rc -eq 0 ] && break
    echo "  (회차 $i 시도 $try 실패 - 재시도)" >> "$SUM"
  done
  echo "" >> "$SUM"
done
echo "ALLDONE" >> "$SUM"

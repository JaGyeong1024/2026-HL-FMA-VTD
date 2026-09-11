#!/usr/bin/env bash
# 묶음 1 하네스: 정지·감속·재출발. usage: bash tools/harness/h1_stop_restart.sh [케이스…]  (기본: h1_* 전부)
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"; source "$(dirname "${BASH_SOURCE[0]}")/run_case.sh"
cases=("$@"); [ ${#cases[@]} -eq 0 ] && cases=($(cd "$ROOT/tools/harness/cases" && ls h1_*.conf | sed 's/.conf$//'))
for c in "${cases[@]}"; do
  run_case "$c" || continue
  case "$c" in
    h1_red_green)
      sb=$(mget "['stopped_before_mark']"); ra=$(mget "['restart_after_mark_s']")
      note "녹색 전 정지=$sb, 녹색 후 재출발 ${ra:-없음} s, 최대감속 실측 $(mget "['max_decel_measured']")"
      [ "$sb" = "True" ] && pass "적신호 정지" || fail "적신호 정지 안 함"
      [ -n "$ra" ] && awk -v r="$ra" 'BEGIN{exit !(r<=10)}' && pass "녹색 후 ${ra}s 재출발" || fail "재출발 ${ra:-없음} (10 s 초과 또는 교착 D1)";;
    h1_car_ahead)
      g=$(mget "['obj_gap_at_stop']"); p=$(mget "['obj_passed']"); d=$(mget "['max_decel_measured']")
      note "정지 시 간격 ${g:-없음} m, 통과=$p, 최대감속 실측 $d, 명령 $(mget "['max_decel_cmd']")"
      [ "$p" = "False" ] && pass "정지차 통과 없음" || fail "정지차 옆을 지나감(회피?)"
      [ -n "$g" ] && awk -v g="$g" 'BEGIN{exit !(g>=2 && g<=8)}' && pass "정지 간격 ${g} m" || fail "정지 간격 ${g:-없음} m (2~8 m 밖 또는 미정지)";;
    h1_ped_d*)
      oc=$(mget "['mover9_outcome']"); md=$(mget "['mover9_min_dist']"); v=$(mget "['mover9_v_at_min']"); td=$(mget "['mover9_trigger_dist']"); v0=$(mget "['mover9_ego_v_at_start']")
      note "보행자 출발 시 ego 거리 ${td} m·속도 ${v0} m/s → 결과 **${oc}**, 최소거리 ${md} m(그때 ${v} m/s), 최대감속 실측 $(mget "['max_decel_measured']") 명령 $(mget "['max_decel_cmd_moving']")"
      [ "$oc" != "충돌" ] && pass "충돌 없음 (${oc})" || fail "충돌 (최소거리 ${md} m)";;
    h1_pedestrian)
      col=$(mget "['mover9_collision']"); md=$(mget "['mover9_min_dist']"); v=$(mget "['mover9_v_at_min']")
      note "보행자 최소거리 ${md} m (그때 속도 ${v} m/s), 최대감속 실측 $(mget "['max_decel_measured']")"
      [ "$col" = "False" ] && pass "충돌 없음" || fail "충돌 판정 (최소거리 ${md} m)";;
  esac
  echo "== [$c] 결과: $OUT/result.txt"; cat "$OUT/result.txt" | grep -c PASS | sed 's/^/  PASS /'
done

#!/usr/bin/env bash
# 묶음 2 하네스: 차선 단위 회피 판단. usage: bash tools/harness/h2_detour.sh [케이스…]  (기본: h2_* 전부)
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"; source "$(dirname "${BASH_SOURCE[0]}")/run_case.sh"
cases=("$@"); [ ${#cases[@]} -eq 0 ] && cases=($(cd "$ROOT/tools/harness/cases" && ls h2_*.conf | sed 's/.conf$//'))
for c in "${cases[@]}"; do
  run_case "$c" || continue
  p=$(mget "['obj_passed']"); lc=$(mget "['lane_changes']"); g=$(mget "['obj_min_gap']")
  detour=no; [ "$p" = "True" ] && detour=yes
  note "정지차 통과=$p, 최소간격 ${g} m, 차선전환=${lc}, 종료 $(cat "$OUT/final_state.txt")"
  [ "$detour" = "$EXPECT_DETOUR" ] && pass "우회=$detour (설계 기대 $EXPECT_DETOUR)" || fail "우회=$detour (설계 기대 $EXPECT_DETOUR)"
  grep -h "external\|EXTERNAL\|lane_change" "$OUT/bridge.log" 2>/dev/null | tail -3
  echo "== [$c] 결과: $OUT/result.txt"
done

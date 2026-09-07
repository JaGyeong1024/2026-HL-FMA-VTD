#!/usr/bin/env bash
# A13 하네스: 리스폰 후 복귀. usage: bash tools/harness/hA13_respawn.sh [케이스…]
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"; source "$(dirname "${BASH_SOURCE[0]}")/run_case.sh"
cases=("$@"); [ ${#cases[@]} -eq 0 ] && cases=($(cd "$ROOT/tools/harness/cases" && ls hA13_*.conf | sed 's/.conf$//'))
for c in "${cases[@]}"; do
  run_case "$c" || continue
  ra=$(mget "['restart_after_mark_s']"); rt=$(mget "['respawn_t']"); fs=$(cat "$OUT/final_state.txt")
  note "리스폰 t=$rt, 재출발 ${ra:-없음} s, 종료 $fs"
  grep -E "리스폰|점프|경로 설정|경로 거부|clear_route|set_route" "$OUT/bridge.log" | tail -6 | sed 's/^/  /'
  [ -n "$ra" ] && awk -v r="$ra" 'BEGIN{exit !(r<=30)}' && pass "리스폰 후 ${ra}s 재출발" || fail "리스폰 후 재출발 ${ra:-없음}"
  echo "$fs" | grep -q "route=2 mode=2" && pass "종료 상태 route SET·AUTONOMOUS" || fail "종료 상태 $fs"
  echo "== [$c] 결과: $OUT/result.txt"
done

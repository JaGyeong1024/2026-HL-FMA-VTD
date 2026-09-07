#!/usr/bin/env bash
# 묶음 3 하네스: 경로상 필수 차선변경. usage: bash tools/harness/h3_lane_change.sh [케이스…]  (기본: h3_* 전부)
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"; source "$(dirname "${BASH_SOURCE[0]}")/run_case.sh"
cases=("$@"); [ ${#cases[@]} -eq 0 ] && cases=($(cd "$ROOT/tools/harness/cases" && ls h3_*.conf | sed 's/.conf$//'))
for c in "${cases[@]}"; do
  run_case "$c" || continue
  # 의도 = 방향지시등 에피소드(bag), 실행 = 이웃 lanelet 전환(trace). route_node 의 "차선변경:" 로그는 loopless 전개 성공 시에만 나옴
  python3 "$ROOT/tools/harness/lcinfo.py" "$OUT" > "$OUT/lc.txt" 2>&1; cat "$OUT/lc.txt" | sed 's/^/  /'
  ni=$(python3 -c "import json; print(json.load(open('$OUT/lc.json'))['n_intent'])" 2>/dev/null)
  ne=$(python3 -c "import json; print(json.load(open('$OUT/lc.json'))['n_executed'])" 2>/dev/null)
  ns=$(python3 -c "import json; print(json.load(open('$OUT/lc.json'))['n_unexecuted_with_stop'])" 2>/dev/null)
  note "최고속 $(mget "['max_speed_kmh']") km/h, route_node 계획 로그 $(grep -c "차선변경:" "$OUT/bridge.log" 2>/dev/null)건(loopless 성공 시만 기록)"
  if [ "$EGO_MOVED" = "0" ]; then :
  elif [ "${ni:-0}" = "0" ]; then note "차선변경 의도 없음 (이 경로·구간에서는 N/A)"
  elif [ "$ne" = "$ni" ]; then pass "차선변경 의도 ${ni} 전부 실행"
  else fail "차선변경 의도 ${ni} 중 실행 ${ne}, 미실행 $((ni-ne)) (정지 동반 ${ns})"; fi
  echo "== [$c] 결과: $OUT/result.txt"
done

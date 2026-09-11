#!/usr/bin/env bash
# 묶음 4 하네스: 좌회전 포켓 진입. usage: bash tools/harness/h4_pocket.sh [케이스…] (기본: h4_* 전부)
# 판정: ego lanelet 열이 포켓(15194/14855/14611) 을 지나 교차로 18858 에 닿았는가.
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"; source "$(dirname "${BASH_SOURCE[0]}")/run_case.sh"
cases=("$@"); [ ${#cases[@]} -eq 0 ] && cases=($(cd "$ROOT/tools/harness/cases" && ls h4_*.conf | sed 's/.conf$//'))
POCKET="15194 14855 14611"
for c in "${cases[@]}"; do
  run_case "$c" || continue
  # lanelet_seq 는 [시각, lanelet_id] 쌍의 배열이다. id 만 뽑는다(0908: 쌍 전체를 문자열화해 판정이 항상 실패했다).
  seq=$(python3 -c "import json;print(' '.join(str(p[1]) for p in json.load(open('$OUT/metrics.json')).get('lanelet_seq',[])))" 2>/dev/null)
  lc=$(mget "['lane_changes']")
  note "lanelet 열: $seq"
  note "차선전환 ${lc}회, 최고속 $(mget "['max_speed_kmh']") km/h, 종료 $(cat "$OUT/final_state.txt")"
  hit=""; for p in $POCKET; do case " $seq " in *" $p "*) hit="$hit $p";; esac; done
  # 판정은 반드시 '포켓 lanelet 을 실제로 지났는가'가 먼저다.
  #   18858 만 보면 오판한다 — 교차로에서 lanelet 이 겹쳐 nearest 매칭이 튀어,
  #   직진 차선(18965, turn_direction=straight)으로 통과해도 18858 이 열에 찍힌다
  #   (2026-09-08 실측: 자차는 끝까지 A 중심선 0.03~0.56 m, 포켓과는 3.2 m 떨어져 있었다).
  if [ "$EGO_MOVED" = "0" ]; then :
  elif [ -z "$hit" ]; then fail "포켓 미진입 — lanelet 열에 15194/14855/14611 없음"
  elif case " $seq " in *" 18858 "*) true;; *) false;; esac; then pass "포켓 경유 좌회전 완료 (포켓:$hit)"
  else fail "포켓 진입($hit) 했으나 교차로 18858 미도달"; fi
  echo "== [$c] 결과: $OUT/result.txt"
done

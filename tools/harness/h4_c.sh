#!/usr/bin/env bash
# C 도달 여부만 판정
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"; source "$(dirname "${BASH_SOURCE[0]}")/run_case.sh"
C_LANES="15449 15233 14960 14677"
B_LANES="15414 15220 14925 14655"
for c in "$@"; do
  run_case "$c" || continue
  seq=$(python3 -c "import json;print(' '.join(str(p[1]) for p in json.load(open('$OUT/metrics.json')).get('lanelet_seq',[])))" 2>/dev/null)
  note "lanelet 열: $seq"
  hitC=""; for l in $C_LANES; do case " $seq " in *" $l "*) hitC="$hitC $l";; esac; done
  hitB=""; for l in $B_LANES; do case " $seq " in *" $l "*) hitB="$hitB $l";; esac; done
  note "B 도달:${hitB:- 없음}   C 도달:${hitC:- 없음}"
  if [ "$EGO_MOVED" = "0" ]; then :
  elif [ -n "$hitC" ]; then pass "C 도달 ($hitC)"
  elif [ -n "$hitB" ]; then fail "B 까지만 감 — C 미도달"
  else fail "A 에서 못 벗어남"; fi
done

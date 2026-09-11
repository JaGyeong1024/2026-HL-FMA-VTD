#!/usr/bin/env bash
# 기동 스크립트 하네스: 결함 주입 시 중단 여부. 전체 스택을 오래 띄우지 않는다.
# usage: bash tools/harness/h0_startup.sh
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
harness_init h0_startup
cd "$ROOT"
echo "-- 1) CSV 없음 → 즉시 중단(exit 1) 기대"
ROUTE_CSV=/nonexistent.csv timeout 20 ./start.sh mock > "$OUT/csv_missing.log" 2>&1; rc=$?
[ "$rc" = "1" ] && pass "CSV 없음: exit $rc" || fail "CSV 없음: exit $rc (기대 1)"
echo "-- 2) mock(VTD) 미기동 → 연결 확인 없이 Autoware 기동을 강행하는가"
pkill -f "mock_vtd.py --port 991[0]" 2>/dev/null
ENGAGE=false ROUTE_CSV="$HOME/hlfma/route/route_pretest_2.csv" ./start.sh mock > "$OUT/no_mock.log" 2>&1 < /dev/null & echo $! > "$OUT/p2.pid"
sleep 25
if pgrep -f "map_path:=$ROOT/map" > /dev/null; then fail "VTD 미연결인데 Autoware 기동 강행 (연결 확인 없음)"; else pass "VTD 미연결 시 중단"; fi
grep -m2 -E "VTD|연결|refused|ECONN" "$HOME/hlfma/logs/bridge_latest.log" 2>/dev/null | sed 's/^/  bridge: /'
kill -TERM "$(cat "$OUT/p2.pid")" 2>/dev/null; for i in $(seq 1 60); do kill -0 "$(cat "$OUT/p2.pid")" 2>/dev/null || break; sleep 0.5; done
harness_kill_leftovers
echo "-- 3) 중복 기동 → 두 번째는 중단 기대 (자기 디렉터리만 검사)"
python3 mock_vtd.py --port 9910 --quiet > "$OUT/mock3.log" 2>&1 < /dev/null & M3=$!
sleep 1
ENGAGE=false ROUTE_CSV="$HOME/hlfma/route/route_pretest_2.csv" ./start.sh mock > "$OUT/first.log" 2>&1 < /dev/null & echo $! > "$OUT/p3.pid"
sleep 12
ENGAGE=false ROUTE_CSV="$HOME/hlfma/route/route_pretest_2.csv" timeout 20 ./start.sh mock > "$OUT/second.log" 2>&1; rc=$?
[ "$rc" = "1" ] && pass "중복 기동 차단: exit $rc" || fail "중복 기동 통과: exit $rc"
head -2 "$OUT/second.log" | sed 's/^/  /'
kill -TERM "$(cat "$OUT/p3.pid")" 2>/dev/null; for i in $(seq 1 60); do kill -0 "$(cat "$OUT/p3.pid")" 2>/dev/null || break; sleep 0.5; done
sleep 3; l=$(pgrep -fc "$ROOT/hlfma_ws/instal[l]/|map_path:=$ROOT/ma[p]"); echo "$l" > "$OUT/first_leftover.txt"
[ "$l" = "0" ] && pass "start 스크립트 정리 훅(TERM)만으로 잔존 0" || fail "start 스크립트 정리 훅(TERM) 후 잔존 $l (강제 정리 필요)"
harness_kill_leftovers
kill $M3 2>/dev/null
echo "-- 4) 정리 후 잔존 (이 디렉터리 install 경로) — 정리 훅(INT) 만으로 남은 수는 first_leftover.txt"
n=$(pgrep -fc "$ROOT/hlfma_ws/instal[l]/|map_path:=$ROOT/ma[p]"); [ "$n" = "0" ] && pass "잔존 노드 0" || fail "잔존 노드 $n"
echo "-- 5) 다른 클론 프로세스 생존 (죽이면 안 됨)"
o=$(pgrep -fc "2026-HL-FMA-VTD-NG/hlfma_ws/install|map_path:=$HOME/2026-HL-FMA-VTD-NG"); note "NG 클론 프로세스 $o 개 (실행 전과 비교)"
cat "$OUT/result.txt"

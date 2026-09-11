#!/usr/bin/env bash
# 케이스 하나 실행 (source 해서 씀). 입력: CASE 이름 (cases/<CASE>.conf). 출력: OUT, metrics.json
run_case() {
  local name="$1"; local conf="$ROOT/tools/harness/cases/$name.conf"
  [ -f "$conf" ] || { echo "conf 없음: $conf"; return 1; }
  unset ROUTE MOCK_ARGS RUN_SEC METRIC_ARGS EXPECT EXPECT_DETOUR STOPLINE_OFFSET VEL_LIMIT ROUTE_CHAIN OBJ_LAT REROUTE
  source "$conf"
  harness_init "$name"
  cp "$conf" "$OUT/case.conf"
  local start; start=$(python3 "$ROOT/tools/harness/place.py" start "$ROUTE")
  local extra=""
  if [ -n "$STOPLINE_OFFSET" ]; then       # 정지선 기준 배치: 자차 경로 차선열(ROUTE_CHAIN) 의 첫 정지선 S → D = S - offset
    export ROUTE_CHAIN
    local sl; sl=$(python3 "$ROOT/tools/harness/place.py" chain-stopline "$ROUTE" | awk '{print $1}')
    local D; D=$(python3 -c "print(round($sl - $STOPLINE_OFFSET, 1))")
    extra="--obj-abs $(python3 "$ROOT/tools/harness/place.py" obj "$ROUTE" 7 "$D" "${OBJ_LAT:-0}")"
    note "정지선 ${sl} m (경로 차선열 기준), 정지차 D=${D} m 횡 ${OBJ_LAT:-0} m ($extra)"
    unset ROUTE_CHAIN
  fi
  mock_start $start $MOCK_ARGS $extra || return 1
  stack_start "$ROUTE"
  sleep 20; capture_start        # 기동 초기부터 기록 (MRM·diag·trajectory 의 시작 상태 증거)
  if ! stack_wait_ready 170; then fail "준비 실패 (route SET / 자율주행 가능)"; stack_stop; return 1; fi
  # REROUTE="2=15449 3=15233" — SET 된 루트의 preferred 만 바꿔 우회를 경로로 표현 (engage 전)
  if [ -n "$REROUTE" ]; then
    note "preferred 재지정: $REROUTE"
    python3 "$ROOT/tools/harness/reroute_preferred.py" $REROUTE 2>&1 | tee "$OUT/reroute.txt" | sed "s/^/  /"
    sleep 3
  fi
  if [ -n "$VEL_LIMIT" ]; then set_velocity_limit "$VEL_LIMIT"; fi
  sleep 2; engage
  run_for "$RUN_SEC"
  local snap; snap=$(snapshot_state); note "종료 시 $snap"; echo "$snap" > "$OUT/final_state.txt"
  # 방어: mock 시뮬 시간이 RUN_SEC 를 크게 넘으면 어떤 단계가 멈춰 있었던 것 (9/7 15:31 velocity pub 무한대기 사례)
  local simt; simt=$(grep -o "t=[0-9]*s" "$OUT/mock.log" | tail -1 | tr -dc 0-9)
  if [ -n "$simt" ] && [ "$simt" -gt $((RUN_SEC + 300)) ]; then fail "케이스 지연: mock 시뮬 ${simt}s > RUN_SEC ${RUN_SEC}+300 (단계 멈춤 의심 — 로그 확인)"; fi
  event stop            # 이 시각 이후 trace 는 스택 종료 중(브리지 페일세이프 -3.0 등) → metrics 가 잘라냄
  stack_stop
  metrics $METRIC_ARGS > /dev/null
  note "기대: ${EXPECT:-EXPECT_DETOUR=$EXPECT_DETOUR}"
  # 전제: ego 가 움직였어야 판정이 의미 있음 (미출발이면 스택 문제 → 별도 FAIL 로 표시)
  if [ -z "$(mget "['t_move']")" ]; then
    fail "ego 미출발 (engage 후 v 항상 0) — 판정 무효, 스택 로그 확인: $OUT/autoware.log"
    EGO_MOVED=0
  else EGO_MOVED=1; fi
}

#!/usr/bin/env bash
# 실기 모드 회귀 (시뮬 PC 없이): mock VTD + 브리지 + Autoware 전체 기동 → 경로 주입 → engage → 주행 확인
# 통과 조건 (개발계획_0902 §5 A0-9):
#   1 /vehicle/status/steering_status 발행     2 /planning/trajectory 발행
#   3 operation_mode: is_autonomous_mode_available true
#   4 경로 주입 후 /perception/traffic_light_recognition/traffic_signals 발행
#   5 engage 후 /control/command/control_cmd 발행     6 mock 차량 이동 (최고속 > 1 m/s)
#   7 발행자 없는 구독 토픽 0 (tools/check_topic_contract.sh)
# 사용: bash tools/regress_mock.sh [route.csv]   (기본 docs/대회정보/route_example.csv)
#   환경: MOCK_ARGS="--x .. --y .. --hdg .. --z .." (mock 시작 pose, 기본 route_example 시작점), OBS_SEC=주행 관찰 시간(기본 90)
#   예) 4.9km 실경로: MOCK_ARGS="--x 447.23 --y -234.05 --hdg <h> --z 44.16" OBS_SEC=600 bash tools/regress_mock.sh tools/real_route_path1.csv
# 로그: ~/hlfma/logs/regress_mock/
set -o pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROUTE="${1:-$ROOT/docs/대회정보/route_example.csv}"
OUT="$HOME/hlfma/logs/regress_mock"; mkdir -p "$OUT"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
source /opt/ros/jazzy/setup.bash
source "$ROOT/hlfma_ws/install/setup.bash"
export LD_LIBRARY_PATH="$HOME/acados/lib:${LD_LIBRARY_PATH:-}"

pass=0; fail=0
check() { if [ "$2" = "1" ]; then echo "  [PASS] $1"; pass=$((pass+1)); else echo "  [FAIL] $1"; fail=$((fail+1)); fi; }
cleanup() {
  # start.sh 의 종료 훅이 자기 프로세스 그룹만 정리한다 (다른 클론 스택은 안 건드림)
  kill -INT $AW_PID 2>/dev/null; for i in $(seq 1 30); do kill -0 $AW_PID 2>/dev/null || break; sleep 0.5; done
  kill -9 $AW_PID 2>/dev/null; kill $MOCK_PID 2>/dev/null
}
trap cleanup EXIT
pkill -f "mock_vtd.py --tl" 2>/dev/null; sleep 1

echo "== mock VTD (신호 적색 → ${TL_GREEN_AT:-170}s에 녹색) =="
GREEN_AT="${TL_GREEN_AT:-170}"   # engage(~110s) 후 적색을 보며 정지선에 서고, 이 시각에 녹색
python3 "$ROOT/mock_vtd.py" --tl 1 --tl-at ${GREEN_AT}:3 ${MOCK_ARGS:-} > "$OUT/mock.log" 2>&1 &
MOCK_PID=$!
sleep 1

echo "== Autoware + 브리지 기동 (rviz 없음, route=$ROUTE) =="
cd "$ROOT"
RVIZ=false ROUTE_CSV="$ROUTE" AUTO_ENGAGE=false ./start.sh mock > "$OUT/autoware.log" 2>&1 &
AW_PID=$!
echo "  노드 안정화 대기 90s"; sleep 90

echo "== 1 차량상태 토픽 =="
hz=$(timeout 8 ros2 topic hz /vehicle/status/steering_status 2>&1 | grep -m1 -o "average rate: [0-9.]*" | grep -o "[0-9.]*$")
check "steering_status ${hz:-0}Hz" "$( [ -n "$hz" ] && echo 1 || echo 0 )"

echo "== 2 trajectory =="
tr=$(timeout 15 ros2 topic hz /planning/trajectory 2>&1 | grep -m1 -o "average rate: [0-9.]*" | grep -o "[0-9.]*$")
check "trajectory ${tr:-0}Hz" "$( [ -n "$tr" ] && echo 1 || echo 0 )"

echo "== 3 자율주행 가능 =="
avail=$(timeout 10 ros2 topic echo /api/operation_mode/state --once --qos-durability transient_local --qos-reliability reliable --field is_autonomous_mode_available 2>/dev/null | head -1)
check "is_autonomous_mode_available=$avail" "$( [ "${avail,,}" = "true" ] && echo 1 || echo 0 )"

echo "== 4 경로·신호등 =="
rs=$(timeout 10 ros2 topic echo /api/routing/state --once --qos-durability transient_local --qos-reliability reliable --field state 2>/dev/null | head -1)
check "routing state=$rs (2=SET)" "$( [ "$rs" = "2" ] && echo 1 || echo 0 )"
tl=$(timeout 8 ros2 topic echo /perception/traffic_light_recognition/traffic_signals --once 2>/dev/null | grep -c "traffic_light_group_id")
check "traffic_signals 규제요소 $tl개" "$( [ "${tl:-0}" -gt 0 ] && echo 1 || echo 0 )"

echo "== 7 토픽 계약 =="
bash "$ROOT/tools/check_topic_contract.sh" > "$OUT/contract.txt" 2>&1
miss=$(grep -c "^  /" "$OUT/contract.txt")
check "발행자 없는 구독 토픽 $miss개" "$( [ "$miss" = "0" ] && echo 1 || echo 0 )"
grep "^  /" "$OUT/contract.txt" | head -15

echo "== 5 engage =="
timeout 15 ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode '{}' > "$OUT/engage.txt" 2>&1
grep -o "success=[a-z]*\|code=[0-9]*\|message='[^']*'" "$OUT/engage.txt" | tr '\n' ' '; echo
sleep 8
cc=$(timeout 8 ros2 topic hz /control/command/control_cmd 2>&1 | grep -m1 -o "average rate: [0-9.]*" | grep -o "[0-9.]*$")
check "control_cmd ${cc:-0}Hz" "$( [ -n "$cc" ] && echo 1 || echo 0 )"
om=$(timeout 8 ros2 topic echo /api/operation_mode/state --once --qos-durability transient_local --qos-reliability reliable --field mode 2>/dev/null | head -1)
check "operation mode=$om (2=AUTONOMOUS)" "$( [ "$om" = "2" ] && echo 1 || echo 0 )"

OBS="${OBS_SEC:-90}"
echo "== 6 주행 (적색 정지 → 60s 녹색 → 출발) ${OBS}s 관찰 =="
sleep "$OBS"
v=$(timeout 5 ros2 topic echo /vehicle/status/velocity_status --once --field longitudinal_velocity 2>/dev/null | head -1)
echo "  현재 속도 $v m/s"
kill $MOCK_PID 2>/dev/null; sleep 2
maxv=$(grep -o "v=[0-9.]*km/h" "$OUT/mock.log" | grep -o "[0-9.]*" | sort -n | tail -1)
check "mock 최고속 ${maxv:-?} km/h (>3.6)" "$( awk -v m="${maxv:-0}" 'BEGIN{exit !(m>3.6)}' && echo 1 || echo 0 )"
echo "  -- 적색 구간 정지 확인 (녹색 전환 ${GREEN_AT}s 직전 20s) --"
awk -v g="$GREEN_AT" 'match($0,/t=([0-9]+)s pos=\(([-0-9.]+),([-0-9.]+)\) v=([0-9.]+)/,a){t=a[1]+0; if(t>=g-20 && t<=g+30 && t%4==0) print "  " $0}' "$OUT/mock.log" | head -14
grep -E "신호등:" "$HOME/hlfma/logs/bridge_latest.log" | head -6
grep "pos=" "$OUT/mock.log" | awk 'NR%10==0' | tail -8

echo "== 브리지 로그 요약 =="
grep -E "맵 로드|VTD 연결|TlRouter|신호등:|점프|두절|ERROR|경로 설정|경로 거부|goal:|점 [0-9]+:" "$HOME/hlfma/logs/bridge_latest.log" | head -30
echo "== Autoware 오류 =="
grep -c "process has died" "$OUT/autoware.log"; grep -iE "error|exception" "$OUT/autoware.log" | grep -v "no error\|ERROR_NONE" | sort | uniq -c | sort -rn | head -10
echo
echo "== 결과: PASS $pass / FAIL $fail =="

#!/usr/bin/env bash
# hlfma_ws 전체 빌드. 사용: cd hlfma_ws && ./build.sh  (수 시간; 로그 build.log, 완료 마커 build.result)

# 증분: ./build.sh --packages-select vtd_autoware_bridge autoware_launch
set -o pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/env.sh"   # ROS + acados + colcon 기본옵션
cd "$WS"
rm -f build.result
if [ $# -gt 0 ]; then
  SEL=("$@")
else
  # 실기·psim에 필요한 최상위 타깃. 의존성 폐쇄는 colcon이 계산.
  # 타깃 목록 파일 build_targets.txt (주석 제외, 공백 구분)
  mapfile -t T < <(grep -v "^#" "$WS/build_targets.txt" | tr " " "\n" | grep -v "^$")
  SEL=(--packages-up-to "${T[@]}")
fi
colcon build "${SEL[@]}" 2>&1 | tee -a build.log
RC=${PIPESTATUS[0]}
if [ $RC -eq 0 ]; then echo OK > build.result; else echo "FAILED($RC)" > build.result; fi
echo "build.result: $(cat build.result)"

#!/usr/bin/env bash
# 별도 터미널에서만 실행: blocked_route_detour의 RTC 후보·승인 상태를 표시한다.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ROOT/hlfma_ws/install/setup.bash"
SIDE="${1:-left}"
case "$SIDE" in left|right) ;; *) echo "usage: $0 [left|right]" >&2; exit 2 ;; esac
ros2 topic echo "/planning/cooperate_status/external_request_lane_change_${SIDE}"

#!/usr/bin/env bash
# 주행 기록: ros2 bag record -a (제어기로 들어오고 나가는 VTD 원본 패킷은 브리지가 /vtd/raw_rx, /vtd/raw_tx 로 발행하므로 bag 에 함께 담김)
#
# 사용:  ./record.sh [이름]        → ~/hlfma/logs/bags/<이름>_<MMDD_HHMMSS>/
#        Ctrl+C 로 종료. 시작 전에 ./start_autonomous.sh 가 떠 있어야 토픽이 잡힌다.
# 재생:  ros2 bag play <dir>      요약: ros2 bag info <dir>
# 패킷 추출: (추후) python3 tools/bag_packets.py <dir>   → DataPacket/CtrlPacket 를 CSV 로 (별도 도구)
set -o pipefail
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
source /opt/ros/jazzy/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/hlfma_ws/install/setup.bash"
NAME="${1:-run}"
OUT="$HOME/hlfma/logs/bags/${NAME}_$(date +%m%d_%H%M%S)"
mkdir -p "$HOME/hlfma/logs/bags"
echo "[record] → $OUT   (Ctrl+C 로 종료)"
# 전 토픽. 점유격자(400KB×10Hz)·rviz 마커류가 크지만 10분에 수 GB 이내라 그대로 담는다.
exec ros2 bag record -a -o "$OUT" --include-hidden-topics

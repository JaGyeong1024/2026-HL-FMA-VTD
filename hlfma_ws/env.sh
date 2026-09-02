# hlfma_ws 환경 — 빌드·실행 공통. ~/.bashrc 와 start_autoware.sh 가 이 파일을 source 한다.
# 사용: source ~/2026-HL-FMA-VTD/hlfma_ws/env.sh   (이후 colcon build / ros2 launch 바로 가능)
_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
# acados (path_optimizer MPC 솔버, ~/acados 로컬 설치): 빌드는 CMAKE_PREFIX_PATH/ACADOS_SOURCE_DIR, 실행은 LD_LIBRARY_PATH
export ACADOS_SOURCE_DIR="$HOME/acados"
export CMAKE_PREFIX_PATH="$HOME/acados${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$HOME/acados/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
# colcon 기본 옵션 (symlink-install, Release, 4 워커) — colcon_defaults.yaml
export COLCON_DEFAULTS_FILE="$_WS/colcon_defaults.yaml"
export MAKEFLAGS="${MAKEFLAGS:--j4}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
# 빌드 산출물이 있으면 오버레이 source
[ -f "$_WS/install/setup.bash" ] && source "$_WS/install/setup.bash"
unset _WS

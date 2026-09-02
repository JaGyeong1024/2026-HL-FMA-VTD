#!/usr/bin/env bash
# 브리지 단독 기능 시험 (Autoware 없이): mock VTD ↔ bridge_node
# 확인: 연결, 차량상태 토픽 이름·주기, 초기화 상태, 가짜 경로 주입 후 신호등 발행, 리스폰 이벤트, 워치독
set -o pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/autoware/install/setup.bash"      # 메시지 패키지용 (hlfma_ws 빌드 전 임시)
export ROS_DOMAIN_ID=43
cd "$HOME/2026-HL-FMA-VTD"
OUT="$HOME/hlfma/logs/bridge_alone"; mkdir -p "$OUT"
pkill -f mock_vtd.py 2>/dev/null; pkill -f "vtd_autoware_bridge.bridge_node" 2>/dev/null; sleep 1

python3 mock_vtd.py --tl 1 --tl-at 20:3 --respawn-at 30 --drop-at 40 --cruise 3 --quiet > "$OUT/mock.log" 2>&1 &
MOCK=$!
sleep 1
cd hlfma_ws/src/hlfma/vtd_autoware_bridge
python3 -m vtd_autoware_bridge.bridge_node --ros-args -p vtd_host:=127.0.0.1 \
  -p map_osm:="$HOME/2026-HL-FMA-VTD/map/lanelet2_map.osm" > "$OUT/bridge.log" 2>&1 &
BR=$!
cd "$HOME/2026-HL-FMA-VTD"
sleep 10
echo "== 발행 토픽 =="; ros2 topic list | grep -E "vehicle/status|localization|perception|vtd"
echo "== steering_status hz =="; timeout 6 ros2 topic hz /vehicle/status/steering_status 2>&1 | grep -m1 "average rate"
echo "== velocity_status =="; timeout 5 ros2 topic echo /vehicle/status/velocity_status --once --field longitudinal_velocity 2>&1 | head -1
echo "== initialization_state =="; timeout 5 ros2 topic echo /localization/initialization_state --once --field state --qos-durability transient_local --qos-reliability reliable 2>&1 | head -1
echo "== pointcloud/ogm =="; timeout 5 ros2 topic hz /perception/occupancy_grid_map/map 2>&1 | grep -m1 "average rate"
echo "== 가짜 경로 주입 (route_example lanelet 체인) =="
python3 - <<'EOF'
import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from autoware_planning_msgs.msg import LaneletRoute, LaneletSegment, LaneletPrimitive
ids=[214, 17306, 666, 2110, 17890, 16354, 15781, 15449, 15233, 14960, 14677, 19073, 13900, 13686, 13168, 147153, 14699, 18769, 12759, 20783, 11786, 11404]
rclpy.init(); n=Node('fake_route')
q=QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
p=n.create_publisher(LaneletRoute,'/planning/mission_planning/route',q)
m=LaneletRoute(); m.header.frame_id='map'
for i in ids:
    s=LaneletSegment(); s.preferred_primitive.id=i; s.primitives.append(LaneletPrimitive(id=i, primitive_type='lane')); m.segments.append(s)
time.sleep(1); p.publish(m); time.sleep(1); print('route published', len(ids)); n.destroy_node(); rclpy.shutdown()
EOF
sleep 4
echo "== traffic_signals (state=1 → RED 기대) =="; timeout 5 ros2 topic echo /perception/traffic_light_recognition/traffic_signals --once 2>&1 | grep -E "traffic_light_group_id|color" | head -8
echo "== 20s: 녹색 전환 대기 =="; sleep 10
timeout 5 ros2 topic echo /perception/traffic_light_recognition/traffic_signals --once 2>&1 | grep -E "color" | head -2
echo "== 30s: 리스폰 이벤트 대기 =="; timeout 15 ros2 topic echo /vtd/respawn --once 2>&1 | head -1 && echo "respawn 수신"
echo "== 워치독: 제어 명령 3초 발행 후 중단 =="
python3 - <<'EOF2'
import rclpy, time
from rclpy.node import Node
from autoware_control_msgs.msg import Control
rclpy.init(); n=Node('fake_ctrl'); p=n.create_publisher(Control,'/control/command/control_cmd',1)
m=Control(); m.lateral.steering_tire_angle=0.1; m.longitudinal.acceleration=1.0
t0=time.time()
while time.time()-t0<3.0:
    p.publish(m); time.sleep(0.05)
print('ctrl 3s 발행 후 중단'); n.destroy_node(); rclpy.shutdown()
EOF2
sleep 3
echo "== 40s: 연결 끊김/재접속 대기 =="; sleep 8
echo "== 브리지 로그 요약 =="; grep -E "맵 로드|OsmMap|VTD 연결|INITIALIZED|TlRouter|신호등|점프|워치독|두절|ERROR|Traceback" "$OUT/bridge.log" | head -30
echo "== mock 로그 =="; tail -5 "$OUT/mock.log"
kill $BR $MOCK 2>/dev/null; sleep 1; pkill -f mock_vtd.py 2>/dev/null; pkill -f "vtd_autoware_bridge.bridge_node" 2>/dev/null
echo "== 종료 =="

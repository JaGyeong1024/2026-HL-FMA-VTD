#!/usr/bin/env bash
# 시작: 경로 SET 확인 후 자율주행(AUTONOMOUS) 전환. 운영측 Start 후 실행.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
source /opt/ros/jazzy/setup.bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/hlfma_ws/install/setup.bash"
python3 - "$@" <<'PY'
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from autoware_adapi_v1_msgs.msg import RouteState, OperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
rclpy.init(); n=Node('start_hlfma')
latched=QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
st={'route':None,'avail':None,'mode':None}
n.create_subscription(RouteState,'/api/routing/state',lambda m: st.update(route=m.state),latched)
n.create_subscription(OperationModeState,'/api/operation_mode/state',lambda m: st.update(avail=m.is_autonomous_mode_available,mode=m.mode),latched)
def spin(sec):
    t=time.time()
    while rclpy.ok() and time.time()-t<sec: rclpy.spin_once(n,timeout_sec=0.1)
# 경로 SET 대기 (최대 60s). 2=SET, 3=ARRIVED
print('[start_hlfma] 경로 SET 대기...')
t0=time.time()
while rclpy.ok() and time.time()-t0<60:
    spin(0.5)
    if st['route']==2: break
    if int(time.time()-t0)%5==0: print(f"  routing state={st['route']} avail={st['avail']} (2=SET)")
if st['route']!=2:
    print(f"[start_hlfma] 경로가 SET(2) 이 아님: state={st['route']}. bridge 로그 확인.", file=sys.stderr); sys.exit(1)
# 자율주행 가능 대기 (최대 30s)
t0=time.time()
while rclpy.ok() and not st['avail'] and time.time()-t0<30: spin(0.5)
print(f"[start_hlfma] routing=SET, is_autonomous_mode_available={st['avail']} → engage")
cli=n.create_client(ChangeOperationMode,'/api/operation_mode/change_to_autonomous')
cli.wait_for_service(timeout_sec=10.0)
fut=cli.call_async(ChangeOperationMode.Request())
t0=time.time()
while rclpy.ok() and not fut.done() and time.time()-t0<15: rclpy.spin_once(n,timeout_sec=0.1)
r=fut.result()
if r: print(f"[start_hlfma] engage: success={r.status.success} code={r.status.code} '{r.status.message}'")
spin(3)
print(f"[start_hlfma] operation mode={st['mode']} (2=AUTONOMOUS)")
print("[start_hlfma] 정지: ros2 service call /api/operation_mode/change_to_stop autoware_adapi_v1_msgs/srv/ChangeOperationMode {}")
n.destroy_node(); rclpy.shutdown()
PY

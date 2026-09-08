#!/usr/bin/env python3
"""주행 계측 — 판단·제어·상태가 바뀌는 곳을 한 파일에 모은다.

record.sh(전 토픽 bag + pcap, 8GB/판, 자식 프로세스가 안 죽음)를 대체한다.
바뀔 때만 한 줄씩 JSONL 로 쓰므로 한 판에 수 MB 다.

usage: python3 tools/trace.py <출력.jsonl> [초]
"""
import sys, json, time, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from autoware_adapi_v1_msgs.msg import RouteState, OperationModeState, MrmState
from autoware_planning_msgs.msg import LaneletRoute, Trajectory
from autoware_perception_msgs.msg import PredictedObjects
from autoware_internal_planning_msgs.msg import PlanningFactorArray
from autoware_internal_debug_msgs.msg import StringStamped
from tier4_rtc_msgs.msg import CooperateStatusArray
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import VelocityReport, TurnIndicatorsCommand
from nav_msgs.msg import Odometry

LATCH = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Trace(Node):
    def __init__(self, path):
        super().__init__('trace')
        self.f = open(path, 'w', buffering=1)
        self.t0 = time.time()
        self.last = {}
        self.ego = None

        s = self.create_subscription
        s(Odometry, '/localization/kinematic_state', self.on_odom, 1)
        s(VelocityReport, '/vehicle/status/velocity_status', self.on_vel, 1)
        s(RouteState, '/api/routing/state', lambda m: self.ev('route_state', {'state': m.state}), LATCH)
        s(OperationModeState, '/api/operation_mode/state',
          lambda m: self.ev('op_mode', {'mode': m.mode, 'avail': m.is_autonomous_mode_available}), LATCH)
        s(MrmState, '/api/fail_safe/mrm_state',
          lambda m: self.ev('mrm', {'state': m.state, 'behavior': m.behavior}), 1)
        s(LaneletRoute, '/planning/mission_planning/route', self.on_route, LATCH)
        s(StringStamped,
          '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/'
          'debug/internal_state', self.on_internal, 1)
        s(Trajectory, '/planning/trajectory', self.on_traj, 1)
        s(PredictedObjects, '/perception/object_recognition/objects', self.on_objs, 1)
        s(Control, '/control/command/control_cmd', self.on_ctrl, 1)
        s(TurnIndicatorsCommand, '/control/command/turn_indicators_cmd',
          lambda m: self.ev('turn', {'cmd': m.command}), 1)

        # planning_factors/* 는 노드마다 따로다 — 있는 것 전부 붙인다
        self.create_timer(2.0, self.attach_factors)
        self.factors_done = set()
        self.get_logger().info(f'trace 시작 → {path}')

    # ---------------- 콜백
    def on_odom(self, m):
        p = m.pose.pose.position
        self.ego = (p.x, p.y, abs(m.twist.twist.linear.x))

    def on_vel(self, m):
        v = round(abs(m.longitudinal_velocity), 1)
        self.ev_if('v', {'v': v})

    def on_route(self, m):
        self.ev('route', {'n_seg': len(m.segments),
                          'preferred': [sg.preferred_primitive.id for sg in m.segments][:8],
                          'uuid': bytes(m.uuid.uuid).hex()[:8]})

    def on_internal(self, m):
        ap = cd = ''
        for line in m.data.splitlines():
            if 'approved modules' in line: ap = line.split('==>')[-1].strip()
            elif 'candidate modules' in line: cd = line.split('==>')[-1].strip()
        self.ev_if('modules', {'approved': ap, 'candidate': cd})

    def on_traj(self, m):
        if not m.points: return
        n = len(m.points)
        vmin = min(p.longitudinal_velocity_mps for p in m.points)
        L = sum(math.dist((m.points[i].pose.position.x, m.points[i].pose.position.y),
                          (m.points[i+1].pose.position.x, m.points[i+1].pose.position.y))
                for i in range(min(n, 400) - 1))
        self.ev_if('traj', {'n': n, 'len': round(L), 'vmin': round(vmin, 1)})

    def on_objs(self, m):
        if self.ego is None: return
        near = []
        for o in m.objects:
            q = o.kinematics.initial_pose_with_covariance.pose.position
            d = math.dist((q.x, q.y), self.ego[:2])
            if d < 90:
                near.append((round(d), round(o.kinematics.initial_twist_with_covariance.twist.linear.x, 1)))
        near.sort()
        self.ev_if('objs', {'n': len(m.objects), 'near': near[:6]})

    def on_ctrl(self, m):
        self.ev_if('ctrl', {'acc': round(m.longitudinal.acceleration, 1),
                            'steer': round(m.lateral.steering_tire_angle, 2)})

    def attach_factors(self):
        for name, types in self.get_topic_names_and_types():
            if '/planning/planning_factors/' in name and name not in self.factors_done:
                self.factors_done.add(name)
                mod = name.rsplit('/', 1)[-1]
                self.create_subscription(
                    PlanningFactorArray, name,
                    lambda m, mod=mod: self.on_factor(mod, m), 1)

    def on_factor(self, mod, m):
        if not m.factors: return
        out = []
        for f in m.factors:
            d = f.control_points[0].distance if f.control_points else None
            out.append({'d': None if d is None else round(d, 1),
                        'detail': getattr(f, 'detail', '')[:40]})
        self.ev_if(f'factor:{mod}', {'f': out})

    # ---------------- 출력
    def ev(self, kind, data):
        rec = {'t': round(time.time() - self.t0, 2), 'k': kind, **data}
        if self.ego: rec['xy'] = [round(self.ego[0], 1), round(self.ego[1], 1)]
        self.f.write(json.dumps(rec, ensure_ascii=False) + '\n')

    def ev_if(self, kind, data):
        key = json.dumps(data, sort_keys=True, ensure_ascii=False)
        if self.last.get(kind) == key: return
        self.last[kind] = key
        self.ev(kind, data)


def main():
    path = sys.argv[1]
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 1e9
    rclpy.init(); n = Trace(path)
    t0 = time.time()
    try:
        while rclpy.ok() and time.time() - t0 < dur:
            rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    n.f.close(); n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()

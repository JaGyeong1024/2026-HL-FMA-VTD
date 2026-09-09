#!/usr/bin/env python3
"""주행 계측 — 돌고 있는 모든 노드·토픽에 자동으로 붙어서 기록한다.

설계
  * 토픽을 손으로 고르지 않는다. 2초마다 전체 토픽 목록을 훑어 새로 생긴 것에 자동 구독.
    파이프라인 중간 단계(behavior path / behavior velocity / path_optimizer /
    motion_velocity / smoother)가 전부 자동으로 들어온다.
  * QoS 는 퍼블리셔 것을 그대로 따라간다 (latched 토픽 놓치지 않음).
  * /rosout 을 통째로 받는다 → 노드 코드를 안 고쳐도 전 노드의 판단 로그가 한 파일에 모인다.
  * 무거운 타입(라이다·영상·TF·맵)만 제외. 나머지는 타입별 요약기로 압축.
  * 값이 바뀔 때만 한 줄(JSONL).

usage: python3 tools/trace.py <출력.jsonl> [초]
"""
import sys, json, time, math, re
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict

# 안 받는 것: 대역폭만 먹고 판단 근거가 아닌 것들
HEAVY_TYPE = re.compile(
    r'PointCloud2|PointCloud$|msg/Image|CompressedImage|CameraInfo|LaserScan'
    r'|OccupancyGrid|TFMessage|LaneletMapBin|PolygonStamped')
SKIP_TOPIC = re.compile(r'^/tf|^/parameter_events|^/rosout_agg|^/clock$|^/trace')

# 타입별 최소 기록 간격(초). 안 적힌 건 DEFAULT_DT.
DEFAULT_DT = 0.1
TYPE_DT = {'MarkerArray': 0.5, 'PredictedObjects': 0.2, 'TrackedObjects': 0.5,
           'DetectedObjects': 0.5, 'DiagnosticArray': 1.0}


def _pt(p):
    """Trajectory / Path / PathWithLaneId / PoseStamped 점 하나를 (pose, v, lane_ids) 로."""
    if hasattr(p, 'point'):                       # PathPointWithLaneId
        return p.point.pose, getattr(p.point, 'longitudinal_velocity_mps', 0.0), list(getattr(p, 'lane_ids', []))
    return p.pose, getattr(p, 'longitudinal_velocity_mps', 0.0), []


def pts_summary(pts):
    """경로/궤적 한 개를 요약. stop_d = 앞쪽 정지점까지 거리 = 가상벽 위치."""
    n = len(pts)
    if n == 0:
        return {'n': 0}
    s = 0.0
    vmin = float('inf')
    stop_d = None
    lanes = []
    prev = None
    for p in pts:
        pose, v, lids = _pt(p)
        c = (pose.position.x, pose.position.y)
        if prev is not None:
            s += math.dist(prev, c)
        prev = c
        if v < vmin:
            vmin = v
        if stop_d is None and v < 0.1:
            stop_d = round(s, 1)
        for lid in lids:
            if not lanes or lanes[-1] != lid:
                lanes.append(lid)
    first_pose, _, _ = _pt(pts[0])
    out = {'n': n, 'len': round(s, 1), 'vmin': round(vmin, 1),
           'end': [round(prev[0], 1), round(prev[1], 1)]}
    if stop_d is not None:
        out['stop_d'] = stop_d
    if lanes:
        out['lanes'] = lanes[:10]
    return out


class Trace(Node):
    def __init__(self, path):
        super().__init__('trace')
        self.f = open(path, 'w', buffering=1)
        self.t0 = time.time()
        self.last = {}          # 변화 감지
        self.last_t = {}        # 레이트 제한
        self.log_seen = {}      # /rosout 중복 억제
        self.subs = {}
        self.ego = None
        self.nbytes = 0
        self.create_timer(2.0, self.discover)
        self.create_timer(30.0, self.heartbeat)
        self.discover()
        self.get_logger().info(f'trace 시작 → {path}')

    # ---------------- 자동 구독
    def discover(self):
        new = []
        for name, types in self.get_topic_names_and_types():
            if name in self.subs or not types:
                continue
            ttype = types[0]
            if SKIP_TOPIC.search(name) or HEAVY_TYPE.search(ttype):
                self.subs[name] = None
                continue
            try:
                cls = get_message(ttype)
            except Exception:
                self.subs[name] = None
                continue
            qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST)
            infos = self.get_publishers_info_by_topic(name)
            if infos:
                p = infos[0].qos_profile
                if p.reliability in (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT):
                    qos.reliability = p.reliability
                if p.durability in (DurabilityPolicy.VOLATILE, DurabilityPolicy.TRANSIENT_LOCAL):
                    qos.durability = p.durability
            short = ttype.rsplit('/', 1)[-1]
            dt = TYPE_DT.get(short, DEFAULT_DT)
            try:
                self.subs[name] = self.create_subscription(
                    cls, name,
                    lambda m, n=name, s=short, d=dt: self.on_any(n, s, d, m), qos)
                new.append((name, short))
            except Exception:
                self.subs[name] = None
        if new:
            self.ev('subscribe', {'n': len(new), 'topics': [f'{n}|{s}' for n, s in new]})

    def heartbeat(self):
        self.ev('hb', {'subs': sum(1 for v in self.subs.values() if v), 'mb': round(self.nbytes / 1e6, 1)})

    # ---------------- 공통 콜백
    def on_any(self, name, short, dt, m):
        # ego 위치는 항상 최신으로
        if name == '/localization/kinematic_state':
            p = m.pose.pose.position
            self.ego = (p.x, p.y, m.twist.twist.linear.x)

        if short == 'Log':
            return self.on_log(m)

        now = time.time()
        if now - self.last_t.get(name, 0.0) < dt:
            return
        self.last_t[name] = now
        try:
            data = self.summarize(name, short, m)
        except Exception as e:
            data = {'err': f'{type(e).__name__}: {e}'[:120]}
        if data is None:
            return
        self.ev_if(name, data)

    def on_log(self, m):
        """전 노드의 로그. 같은 줄이 2초 안에 반복되면 접는다."""
        key = (m.name, m.msg[:200])
        now = time.time()
        if now - self.log_seen.get(key, 0.0) < 2.0:
            return
        self.log_seen[key] = now
        self.ev('log', {'node': m.name, 'lvl': m.level, 'msg': m.msg[:400],
                        'src': f'{m.function}:{m.line}'})

    # ---------------- 타입별 요약
    def summarize(self, name, short, m):
        if short in ('Trajectory', 'Path', 'PathWithLaneId'):
            return pts_summary(m.points)
        if short == 'Path' and hasattr(m, 'poses'):
            return pts_summary(m.poses)

        if short == 'MarkerArray':
            ns = Counter(mk.ns for mk in m.markers)
            texts = []
            for mk in m.markers:
                t = getattr(mk, 'text', '')
                if t and t not in texts:
                    texts.append(t[:60])
            if not ns and not texts:
                return None
            return {'ns': dict(ns.most_common(8)), 'text': texts[:8]}

        if short in ('PredictedObjects', 'TrackedObjects', 'DetectedObjects'):
            near = []
            if self.ego:
                for o in m.objects:
                    k = o.kinematics
                    pose = (getattr(k, 'initial_pose_with_covariance', None)
                            or getattr(k, 'pose_with_covariance', None))
                    tw = (getattr(k, 'initial_twist_with_covariance', None)
                          or getattr(k, 'twist_with_covariance', None))
                    if pose is None:
                        continue
                    q = pose.pose.position
                    d = math.dist((q.x, q.y), self.ego[:2])
                    if d < 90:
                        v = round(tw.twist.linear.x, 1) if tw else None
                        near.append((round(d), v))
                near.sort()
            return {'n': len(m.objects), 'near': near[:6]}

        if short == 'DiagnosticArray':
            bad = [(s.name[:60], s.level) for s in m.status if s.level != 0]
            return {'bad': bad[:8]} if bad else None

        if short == 'PlanningFactorArray':
            if not m.factors:
                return {'f': []}
            out = []
            for fct in m.factors:
                d = fct.control_points[0].distance if fct.control_points else None
                out.append({'m': fct.module[:24] if hasattr(fct, 'module') else '',
                            'b': getattr(fct, 'behavior', None),
                            'd': None if d is None else round(d, 1),
                            'detail': getattr(fct, 'detail', '')[:60]})
            return {'f': out}

        if short == 'CooperateStatusArray':
            return {'s': [{'m': s.module.type if hasattr(s.module, 'type') else '',
                           'safe': s.safe, 'cmd': s.command_status.type,
                           'auto': getattr(s, 'auto_mode', None),
                           'd': round(s.start_distance, 1)} for s in m.statuses][:8]}

        # ---- HL FMA 9/10 계측 추가 ----
        #   추종 오차·조향·실제 가속도를 재려면 이 세 종류가 필요한데 그동안
        #   기본 요약기가 잘라내서 분석 때마다 값이 없었다.
        if short == 'Control':                       # /control/command/control_cmd 등
            lo = getattr(m, 'longitudinal', None)
            la = getattr(m, 'lateral', None)
            out = {}
            if lo is not None:
                out['acc'] = round(getattr(lo, 'acceleration', 0.0), 3)
                out['vel'] = round(getattr(lo, 'velocity', 0.0), 3)
                out['jerk'] = round(getattr(lo, 'jerk', 0.0), 3)
            if la is not None:
                out['steer'] = round(getattr(la, 'steering_tire_angle', 0.0), 4)
                out['steer_rate'] = round(getattr(la, 'steering_tire_rotation_rate', 0.0), 4)
            return out or None

        if short == 'AccelWithCovarianceStamped':    # /localization/acceleration
            a = m.accel.accel.linear
            return {'ax': round(a.x, 3), 'ay': round(a.y, 3)}

        if short == 'SteeringReport':                # /vehicle/status/steering_status
            return {'steer_actual': round(m.steering_tire_angle, 4)}

        if short == 'Float32MultiArrayStamped' and 'diagnostic' in name:
            # trajectory_follower lateral/longitudinal diagnostic (횡편차·요오차 등)
            return {'d': [round(float(x), 4) for x in list(m.data)[:12]]}

        if short == 'Float32Stamped' or short == 'Float64Stamped':
            return {'d': round(float(m.data), 4)}

        if short == 'VelocityLimit':
            return {'max_v': round(m.max_velocity, 2), 'sender': getattr(m, 'sender', '')}

        if short == 'StringStamped' and 'internal_state' in name:
            ap = cd = ''
            for line in m.data.splitlines():
                if 'approved modules' in line:
                    ap = line.split('==>')[-1].strip()
                elif 'candidate modules' in line:
                    cd = line.split('==>')[-1].strip()
            return {'approved': ap, 'candidate': cd}

        if short == 'LaneletRoute':
            return {'n_seg': len(m.segments),
                    'preferred': [sg.preferred_primitive.id for sg in m.segments][:12],
                    'uuid': bytes(m.uuid.uuid).hex()[:8]}

        if short == 'Odometry':
            p = m.pose.pose.position
            return {'v': round(m.twist.twist.linear.x, 1),
                    'xy': [round(p.x, 1), round(p.y, 1)]}

        # 나머지는 통째로 (헤더 빼고, 잘라서)
        d = message_to_ordereddict(m)
        d.pop('header', None)
        d.pop('stamp', None)
        s = json.dumps(d, ensure_ascii=False, default=str)
        return {'v': s[:400]} if len(s) > 400 else json.loads(s) if s.startswith('{') else {'v': s}

    # ---------------- 출력
    def ev(self, kind, data):
        rec = {'t': round(time.time() - self.t0, 2), 'k': kind, **data}
        if self.ego:
            rec['xy'] = [round(self.ego[0], 1), round(self.ego[1], 1)]
            rec['v'] = round(self.ego[2], 1)
        line = json.dumps(rec, ensure_ascii=False, default=str) + '\n'
        self.nbytes += len(line)
        self.f.write(line)

    def ev_if(self, kind, data):
        key = json.dumps(data, sort_keys=True, ensure_ascii=False, default=str)
        if self.last.get(kind) == key:
            return
        self.last[kind] = key
        self.ev(kind, data)


def main():
    path = sys.argv[1]
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 1e9
    rclpy.init()
    n = Trace(path)
    t0 = time.time()
    try:
        while rclpy.ok() and time.time() - t0 < dur:
            rclpy.spin_once(n, timeout_sec=0.02)
    except KeyboardInterrupt:
        pass
    n.ev('end', {'mb': round(n.nbytes / 1e6, 2),
                 'subs': sum(1 for v in n.subs.values() if v)})
    n.f.close()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

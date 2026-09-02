"""경로 CSV → Autoware 경로 주입 노드.

대회 당일 받는 CSV(seq,x,y — 대회정보.md §3): 첫 점 = 시작(스폰, VTD가 정함), 마지막 = 종료,
중간 점은 2개씩 짝 = 교차로 진입·진출. 이 노드는
  1. CSV를 읽어 각 점을 맵 lanelet 중심선에 투영해 heading을 구하고 (mission_planner는 goal heading이
     차선 방향과 goal_angle_threshold_deg(45°) 이상 다르면 거부),
  2. 종료 판정이 "후륜축이 종료 좌표 통과"이므로 goal은 종료 좌표에서 차선을 따라 goal_extend_m 앞에 두고,
  3. 중간 점들은 waypoints로 넣어 ADAPI /api/routing/set_route_points 를 호출한다.
     (allow_goal_modification=false: goal_planner의 갓길 정차 후보 생성 방지)
  4. 응답 status를 반드시 로그로 남긴다 (adaptor류는 실패를 로깅하지 않음).

런치 인자 route_csv:=<파일> 로 지정. auto_engage:=true 면 경로 SET 후 자율주행 전환까지 요청한다
(기본 false — rviz에서 경로를 눈으로 확인한 뒤 사람이 engage).
수동 engage:
  ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode {}
"""
import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from autoware_adapi_v1_msgs.msg import RouteState
from autoware_adapi_v1_msgs.srv import SetRoute, SetRoutePoints, ClearRoute, ChangeOperationMode
from autoware_adapi_v1_msgs.msg import RouteSegment, RoutePrimitive

from .osm_map import OsmMap

try:
    import lanelet2
    from lanelet2.projection import LocalCartesianProjector
    from lanelet2.io import Origin
    HAVE_LANELET2 = True
except Exception:  # noqa
    HAVE_LANELET2 = False


def load_csv(path):
    pts = []
    with open(path, newline='') as f:
        for row in csv.reader(f):
            if not row or not row[0].strip().lstrip('-').isdigit():
                continue          # 헤더·빈 줄
            pts.append((int(row[0]), float(row[1]), float(row[2])))
    pts.sort(key=lambda r: r[0])
    return [(x, y) for _, x, y in pts]


class RouteNode(Node):
    def __init__(self):
        super().__init__('vtd_route_node')
        dp = self.declare_parameter
        dp('route_csv', '')
        dp('map_osm', '')
        dp('goal_extend_m', 25.0)      # 종료 좌표에서 차선을 따라 앞으로 (후륜축 통과 보장)
        dp('lane_search_m', 8.0)       # CSV 점 ↔ lanelet 매칭 허용 거리
        dp('auto_engage', False)
        dp('reinject_on_respawn', False)  # 리스폰 이벤트 시 경로 재주입 (실측 후 결정)
        dp('use_waypoints', True)      # 중간 짝점을 waypoints로 (false면 goal만)
        g = lambda k: self.get_parameter(k).value
        self.csv_path = g('route_csv')
        self.goal_extend = float(g('goal_extend_m'))
        self.lane_search = float(g('lane_search_m'))
        self.auto_engage = bool(g('auto_engage'))
        self.use_waypoints = bool(g('use_waypoints'))
        if not self.csv_path:
            raise RuntimeError('route_csv 파라미터가 비어 있음')
        self.map = OsmMap(g('map_osm'), self.get_logger())
        # lanelet2 라우팅 그래프 (후보 조합 중 총길이 최소 선택용). 좌표계는 상대 비교용이라 원점은 임의
        self.rg = None
        if HAVE_LANELET2:
            try:
                lm = lanelet2.io.load(g('map_osm'), LocalCartesianProjector(Origin(37.2, 126.8)))
                tr = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                   lanelet2.traffic_rules.Participants.Vehicle)
                self.rg = lanelet2.routing.RoutingGraph(lm, tr)
                self.lm = lm
                self.get_logger().info('lanelet2 라우팅 그래프 준비 (경로점 매칭 최적화 사용)')
            except Exception as e:
                self.get_logger().warning(f'lanelet2 로드 실패 → 탐욕 매칭 사용: {e}')
        else:
            self.get_logger().warning('lanelet2 python 없음 → 탐욕 매칭 사용')
        self.points = load_csv(self.csv_path)
        if len(self.points) < 2:
            raise RuntimeError(f'경로 점이 2개 미만: {self.csv_path}')
        self.get_logger().info(f'경로 CSV {self.csv_path}: {len(self.points)}점')

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.route_state = None
        self.ego = None
        self.create_subscription(RouteState, '/api/routing/state', self.on_route_state, latched)
        self.create_subscription(Odometry, '/localization/kinematic_state', self.on_odom, 1)
        if bool(g('reinject_on_respawn')):
            self.create_subscription(Empty, '/vtd/respawn', self.on_respawn, 1)
        self.cli_clear = self.create_client(ClearRoute, '/api/routing/clear_route')
        self.cli_set = self.create_client(SetRoutePoints, '/api/routing/set_route_points')
        self.cli_set_seg = self.create_client(SetRoute, '/api/routing/set_route')
        self.cli_engage = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_autonomous')
        self.done = False
        self.attempts = 0
        self.full_lanelets = None


    def on_route_state(self, msg):
        self.route_state = msg.state

    def on_odom(self, msg):
        self.ego = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.ego_yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                                  1.0 - 2.0 * (o.y * o.y + o.z * o.z))

    def on_respawn(self, _):
        self.get_logger().warning('리스폰 이벤트 → 경로 재주입')
        self.done = False

    # ---------------- 경로 계산
    def build_request(self):
        pts = self.points
        req = SetRoutePoints.Request()
        req.header.frame_id = 'map'
        req.option.allow_goal_modification = False
        poses = []
        n = len(pts)
        # 1) 점별 후보 (진입·진출 방향 어느 쪽과든 ±90° 안)
        cand = []
        for i, (x, y) in enumerate(pts):
            hints = []
            if i > 0:
                hints.append(math.atan2(y - pts[i - 1][1], x - pts[i - 1][0]))
            if i < n - 1:
                hints.append(math.atan2(pts[i + 1][1] - y, pts[i + 1][0] - x))
            c = self.map.match_candidates(x, y, hints, self.lane_search, need_pred=(i > 0), need_succ=(i < n - 1))
            if not c:
                raise RuntimeError(f'점 {i} ({x:.1f},{y:.1f})에서 {self.lane_search}m 안에 진행방향 lanelet 없음')
            cand.append(c)
        # 시작 앵커: 시작점은 자유변수가 아니라 ego 가 실제로 있는 lanelet 이어야 한다.
        # DP 가 전체 길이를 줄이려 시작을 옆 차선으로 갈아타면 route 가 ego 를 안 지나
        # behavior_path_planner 가 "Ego is out of route" 로 판정해 궤적을 안 낸다.
        start_lid = None
        if self.ego is not None:
            eh = getattr(self, 'ego_yaw', None)
            h0 = [eh] if eh is not None else [
                (math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0]) if n > 1 else 0.0)]
            sc = self.map.match_candidates(self.ego.x, self.ego.y, h0, self.lane_search,
                                           need_pred=False, need_succ=True)
            if sc:
                start_lid = min(sc, key=lambda t: t[2]['d'])[0]  # ego 를 품은(오프셋 최소) lanelet
                if all(l != start_lid for l, _, _ in cand[0]):
                    cand[0] = [x for x in sc if x[0] == start_lid] + cand[0]
                self.get_logger().info(f'시작 앵커: ego lanelet {start_lid} (ego pose 기준, 시작 고정)')
        # 2) 조합 선택: lanelet2 라우팅 총길이 + 벌점 최소 (DP). 라우팅 불가면 탐욕(벌점 최소)
        choice = [c[0][0] for c in cand]
        if start_lid is not None:
            choice[0] = start_lid
        if self.rg is not None:
            INF = float('inf')
            cost = [{start_lid: 0.0}] if start_lid is not None else [{lid: pen for lid, pen, _ in cand[0]}]
            back = [{}]
            for i in range(1, n):
                cost.append({}); back.append({})
                for lid, pen, _ in cand[i]:
                    best, barg = INF, None
                    for plid, pc in cost[i - 1].items():
                        if pc == INF:
                            continue
                        L = self._route_len(plid, lid)
                        if L is None:
                            continue
                        if pc + L + pen < best:
                            best, barg = pc + L + pen, plid
                    cost[i][lid] = best
                    back[i][lid] = barg
            end = min(cost[-1], key=cost[-1].get) if cost[-1] else None
            if end is not None and cost[-1][end] < INF:
                choice = [end]
                for i in range(n - 1, 0, -1):
                    choice.append(back[i][choice[-1]])
                choice.reverse()
                self.get_logger().info(f'경로점 매칭: 라우팅 총길이+벌점 {cost[-1][end]:.0f}m (후보 조합 최적)')
                fl = self._expand_sequence(choice)
                if fl and len(set(fl)) != len(fl):
                    import collections as _c
                    dups = [x for x, c in _c.Counter(fl).items() if c > 1]
                    self.get_logger().warning(
                        f'명시 세그먼트 경로에 순환 lanelet {dups} 발견 → set_route_points 로 폴백 '
                        f'(mission_planner 가 loopless 경로를 계획하도록)')
                    fl = None
                self.full_lanelets = fl
            else:
                self.get_logger().error('어느 후보 조합으로도 lanelet2 경로가 이어지지 않음 → 탐욕 매칭으로 시도')
        for i, (x, y) in enumerate(pts):
            lid = choice[i]
            info = next(inf for l, p, inf in cand[i] if l == lid)
            d, s, h = self.map.project(lid, x, y)
            # mission_planner는 waypoint를 자기 기준 최근접 lanelet에 붙이므로, 선택한 lanelet 중심선 위 점을 넘긴다
            px, py, _ = self.map._interp(self.map.lanelets[lid], s)
            poses.append((px, py, h, lid, s, d))
            warn = (' ⚠ ' + ','.join(info['flags'])) if info['flags'] else ''
            self.get_logger().info(f'  점 {i}: ({x:.1f},{y:.1f}) → lanelet {lid} 오프셋 {d:.2f}m 방향차 {info["dh_deg"]:.0f}° 폭 {info["w"]:.1f}m heading {math.degrees(h):.0f}° → 중심선 ({px:.1f},{py:.1f}){warn}')
        # goal: 마지막 점에서 차선을 따라 goal_extend 앞
        x, y, h, lid, s, _ = poses[-1]
        route_set = {p[3] for p in poses}
        gx, gy, gh, glid = self.map.point_along(lid, s, self.goal_extend, prefer=route_set)
        self.get_logger().info(f'  goal: 종료점 {self.goal_extend:.0f}m 앞 ({gx:.1f},{gy:.1f}) lanelet {glid} heading {math.degrees(gh):.0f}°')
        req.goal = self._pose(gx, gy, gh, glid)
        if self.use_waypoints:
            for (x, y, h, lid, s, d) in poses[1:-1]:
                req.waypoints.append(self._pose(x, y, h, lid))
            # 종료 좌표 자체도 waypoint로 (goal이 그 앞이므로 반드시 통과)
            x, y, h, lid = poses[-1][:4]
            req.waypoints.append(self._pose(x, y, h, lid))
        return req

    def _route_len(self, a, b):
        """lanelet a → b 최단 경로 길이 [m] (lanelet2 routing). 경로 없으면 None."""
        sp = self._shortest_ids(a, b)
        if sp is None:
            return None
        return sum(lanelet2.geometry.length2d(self.lm.laneletLayer[i]) for i in sp)

    def _shortest_ids(self, a, b):
        """a→b 최단 경로 lanelet id 열 (a 포함, b 포함). 없으면 None."""
        try:
            r = self.rg.getRoute(self.lm.laneletLayer[a], self.lm.laneletLayer[b])
        except Exception:
            return None
        if r is None:
            return None
        return [l.id for l in r.shortestPath()]

    def _expand_sequence(self, choice):
        """선택 lanelet 열(웨이포인트별) → 사이를 최단경로로 채운 전체 lanelet id 열 (중복 제거).
        순환 경로도 그대로 펼쳐지므로 set_route(명시 세그먼트)로 넘기면 mission_planner 붕괴 없음."""
        seq = [choice[0]]
        for a, b in zip(choice, choice[1:]):
            sp = self._shortest_ids(a, b)
            if sp is None:
                self.get_logger().warning(f'lanelet {a}→{b} 경로 없음 → set_route 불가, set_route_points 폴백')
                return None
            for lid in sp[1:]:
                if lid != seq[-1]:
                    seq.append(lid)
        return seq

    def build_seg_request(self):
        """전체 lanelet 순서를 SetRoute 세그먼트로. goal = 마지막 점을 그 lanelet 중심선 goal_extend 앞
        (단 마지막 세그먼트 lanelet 안). 순환 경로도 붕괴하지 않음."""
        req = SetRoute.Request()
        req.header.frame_id = 'map'
        req.option.allow_goal_modification = False
        for lid in self.full_lanelets:
            seg = RouteSegment()
            seg.preferred = RoutePrimitive(id=int(lid), type='lane')
            req.segments.append(seg)
        # goal: 마지막 CSV 점을 마지막 lanelet 중심선에 투영해 goal_extend 앞 (그 lanelet 안으로 클램프)
        gx, gy = self.points[-1]
        last = self.full_lanelets[-1]
        d, s0, h = self.map.project(last, gx, gy)
        L = self.map.lanelets[last].length
        s_goal = min(s0 + self.goal_extend, max(0.0, L - 1.0))
        px, py, ph = self.map._interp(self.map.lanelets[last], s_goal)
        req.goal = self._pose(px, py, ph, last)
        self.get_logger().info(f'  goal(세그먼트): lanelet {last} s={s_goal:.1f}/{L:.1f} ({px:.1f},{py:.1f})')
        return req

    def _pose(self, x, y, yaw, lid=None):
        p = Pose()
        p.position.x, p.position.y = x, y
        p.position.z = self.map.elevation(lid, x, y) if lid is not None else 0.0
        p.orientation.z, p.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        return p

    # ---------------- 실행 (콜백 안에서 spin_once를 중첩하지 않도록 절차형)
    def wait(self, cond, what, timeout=None):
        t0 = time.time()
        while rclpy.ok() and not cond():
            rclpy.spin_once(self, timeout_sec=0.2)
            if int(time.time() - t0) % 5 == 0:
                self.get_logger().info(f'{what} 대기', throttle_duration_sec=5.0)
            if timeout is not None and time.time() - t0 > timeout:
                return False
        return True

    def run(self):
        self.wait(lambda: self.ego is not None, 'ego 위치(/localization/kinematic_state)')
        self.wait(lambda: (self.cli_set.service_is_ready() or self.cli_set_seg.service_is_ready()) and self.cli_clear.service_is_ready(),
                  '라우팅 서비스(/api/routing/set_route)')
        self.wait(lambda: self.route_state is not None, '라우팅 상태(/api/routing/state)')
        while rclpy.ok():
            if not self.done:
                self.attempts += 1
                try:
                    self.run_once()
                except Exception as e:
                    self.get_logger().error(f'경로 주입 실패: {e!r}')
                if not self.done:
                    if self.attempts >= 3:
                        self.get_logger().error('경로 주입 3회 실패 — 중단. CSV·맵·ego 위치를 확인할 것')
                        self.done = True
                    else:
                        self.wait(lambda: False, '재시도', timeout=3.0)
            rclpy.spin_once(self, timeout_sec=0.5)

    def call(self, client, req, timeout=15.0):
        fut = client.call_async(req)
        t0 = time.time()
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > timeout:
                raise TimeoutError(f'{client.srv_name} 응답 없음 ({timeout}s)')
        return fut.result()

    def run_once(self):
        if self.route_state in (RouteState.SET, RouteState.CHANGING, RouteState.ARRIVED):
            self.get_logger().info('기존 경로 제거 (clear_route)')
            r = self.call(self.cli_clear, ClearRoute.Request())
            self.get_logger().info(f'  clear_route: success={r.status.success} code={r.status.code} {r.status.message}')
            time.sleep(1.0)
        # 경로를 먼저 계산(build_request 가 self.full_lanelets 를 설정) 후 방식 선택.
        # 명시 세그먼트(set_route)를 우선한다 → route 가 ego 차선에서 시작하는 결정론적 경로.
        self.full_lanelets = None
        req = self.build_request()
        if self.full_lanelets and self.cli_set_seg.service_is_ready():
            r = self.call(self.cli_set_seg, self.build_seg_request(), timeout=30.0)
            self.get_logger().info(f'set_route(명시 세그먼트) 호출: lanelet {len(self.full_lanelets)}개, 시작 lanelet {self.full_lanelets[0]}')
        else:
            self.get_logger().info(f'set_route_points 호출: waypoints {len(req.waypoints)}개')
            r = self.call(self.cli_set, req, timeout=30.0)
        st = r.status
        if not st.success:
            self.get_logger().error(f'경로 거부: code={st.code} message="{st.message}"')
            return
        self.get_logger().info(f'경로 설정 성공: code={st.code} {st.message}')
        # SET 확인
        t0 = time.time()
        while rclpy.ok() and self.route_state != RouteState.SET and time.time() - t0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'라우팅 상태: {self.route_state} (2=SET)')
        self.done = True
        if self.auto_engage:
            if not self.cli_engage.wait_for_service(timeout_sec=10.0):
                self.get_logger().error('change_to_autonomous 서비스 없음')
                return
            r = self.call(self.cli_engage, ChangeOperationMode.Request())
            self.get_logger().info(f'engage: success={r.status.success} code={r.status.code} {r.status.message}')
        else:
            self.get_logger().info('auto_engage=false: rviz에서 경로 확인 후 수동 engage — '
                                   'ros2 service call /api/operation_mode/change_to_autonomous '
                                   'autoware_adapi_v1_msgs/srv/ChangeOperationMode {}')


def main():
    rclpy.init()
    node = RouteNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

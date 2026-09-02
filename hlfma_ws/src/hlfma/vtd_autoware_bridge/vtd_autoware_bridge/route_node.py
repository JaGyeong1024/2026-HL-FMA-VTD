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
from autoware_adapi_v1_msgs.srv import SetRoutePoints, ClearRoute, ChangeOperationMode

from .osm_map import OsmMap


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
        self.cli_engage = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_autonomous')
        self.done = False
        self.attempts = 0


    def on_route_state(self, msg):
        self.route_state = msg.state

    def on_odom(self, msg):
        self.ego = msg.pose.pose.position

    def on_respawn(self, _):
        self.get_logger().warning('리스폰 이벤트 → 경로 재주입')
        self.done = False

    # ---------------- 경로 계산
    def build_request(self):
        pts = self.points
        req = SetRoutePoints.Request()
        req.header.frame_id = 'map'
        req.option.allow_goal_modification = False
        prev_lid = None
        prev_pt = pts[0]
        poses = []
        for i, (x, y) in enumerate(pts):
            # heading 힌트 = CSV 인접점 방향 (첫 점은 다음 점 방향). 최근접만 쓰면 반대 차로(1.4~1.6m)가
            # 정방향(4.4m)보다 가까운 경우가 있어 경로가 2배로 부풀 수 있음 (검토보고 D_맵v2 D-1)
            if i > 0:
                hint = math.atan2(y - prev_pt[1], x - prev_pt[0])
            else:
                nx, ny = pts[1]
                hint = math.atan2(ny - y, nx - x)
            lid = self.map.nearest_lanelet(x, y, hint, self.lane_search)
            if lid is None:
                lid = self.map.nearest_lanelet(x, y, None, self.lane_search)
            if lid is None:
                raise RuntimeError(f'점 {i} ({x:.1f},{y:.1f})에서 {self.lane_search}m 안에 lanelet 없음')
            d, s, h = self.map.project(lid, x, y)
            poses.append((x, y, h, lid, s, d))
            dead = ' ⚠ 후속 lanelet 없음(막다른 차선)' if self.map.is_dead_end(lid) else ''
            self.get_logger().info(f'  점 {i}: ({x:.1f},{y:.1f}) → lanelet {lid} 오프셋 {d:.2f}m heading {math.degrees(h):.0f}°{dead}')
            prev_pt, prev_lid = (x, y), lid
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
        self.wait(lambda: self.cli_set.service_is_ready() and self.cli_clear.service_is_ready(),
                  '라우팅 서비스(/api/routing/set_route_points)')
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
        req = self.build_request()
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

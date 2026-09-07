#!/usr/bin/env python3
"""SET 된 루트의 preferred_primitive 만 바꿔 다시 넣는다 (우회를 '경로'로 표현).

세그먼트의 primitives(이웃 묶음)는 그대로 두고 preferred 만 교체하므로
route_handler 의 route_lanelets_ 는 유지되고(=차선변경 가능) preferred_lanelets_ 만 바뀐다.
  route_handler.cpp:558-570 참조.

usage: reroute_preferred.py 2=15449 3=15233 4=14855          # 세그먼트 인덱스=새 preferred id
       reroute_preferred.py 0=16354:16144,16249                # preferred:추가할 이웃들
mission_planner 는 차선변경이 불필요한 구간을 primitive 1개짜리 세그먼트로 만든다. 거기서는
route_lanelets_ 에 이웃이 없어 차선변경이 원천 불가하므로, 우회 시에는 이웃을 직접 넣어줘야 한다.
"""
import sys, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from autoware_planning_msgs.msg import LaneletRoute
from autoware_adapi_v1_msgs.srv import SetRoute
from autoware_adapi_v1_msgs.msg import RouteSegment, RoutePrimitive

def main():
    want, add = {}, {}
    for a in sys.argv[1:]:
        k, v = a.split('=')
        if ':' in v:
            v, alts = v.split(':', 1)
            add[int(k)] = [int(x) for x in alts.split(',') if x]
        want[int(k)] = int(v)
    rclpy.init()
    n = Node('reroute_preferred')
    q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
    box = {}
    n.create_subscription(LaneletRoute, '/planning/mission_planning/route',
                          lambda m: box.setdefault('r', m), q)
    for _ in range(200):
        rclpy.spin_once(n, timeout_sec=0.05)
        if 'r' in box: break
    if 'r' not in box:
        print('루트 없음 — 중단'); return 1
    r = box['r']
    cli = n.create_client(SetRoute, '/api/routing/change_route')
    if not cli.wait_for_service(timeout_sec=10.0):
        print('change_route 서비스 없음 — 중단'); return 1
    req = SetRoute.Request()
    req.header = r.header
    req.goal = r.goal_pose
    req.option.allow_goal_modification = False
    for i, s in enumerate(r.segments):
        seg = RouteSegment()
        new = want.get(i, s.preferred_primitive.id)
        ids = [p.id for p in s.primitives]
        if i in add:
            # preferred 자신도 세그먼트에 없을 수 있다(상류 이웃을 새로 넣는 경우) — 반드시 포함시킨다.
            for extra in [new] + add[i]:
                if extra not in ids:
                    ids.append(extra)
        if i in add:
            print(f'  seg{i}: primitive 확장 → {ids}')
        if new not in ids:
            print(f'  seg{i}: {new} 는 이 세그먼트에 없음 {ids} — 원래 값 유지')
            new = s.preferred_primitive.id
        seg.preferred = RoutePrimitive(id=new, type='lane')
        seg.alternatives = [RoutePrimitive(id=p, type='lane') for p in ids if p != new]
        if new != s.preferred_primitive.id:
            print(f'  seg{i}: preferred {s.preferred_primitive.id} → {new}  (대안 {len(seg.alternatives)}개)')
        req.segments.append(seg)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=20.0)
    if not fut.done():
        print('change_route 응답 없음'); return 1
    res = fut.result()
    print(f'change_route: success={res.status.success} code={res.status.code} "{res.status.message}"')
    return 0 if res.status.success else 1

if __name__ == '__main__':
    sys.exit(main())

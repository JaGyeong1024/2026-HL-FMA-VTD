#!/usr/bin/env python3
"""백을 재생하되 헤더 스탬프를 현재 시각으로 다시 찍어 발행한다.

VTD 없이 planning 스택만 올려두고 브리지가 주던 입력(자차 상태·인지·경로)을
백에서 흘려보내기 위한 도구. Autoware 노드들은 데이터 신선도를 now() 기준으로
보기 때문에 원본 스탬프 그대로는 전부 stale 로 버려진다.

  python3 bag_replay.py <bag> [--rate 1.0] [--start 0] [--duration 0] [--loop-route]
"""
import argparse
import bisect
import threading
import time

import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# 브리지가 발행하던 것들. 이 목록 밖은 스택이 스스로 만든다.
LIVE = [
    "/localization/kinematic_state",
    "/localization/acceleration",
    "/vehicle/status/velocity_status",
    "/vehicle/status/steering_status",
    "/vehicle/status/gear_status",
    "/vehicle/status/control_mode",
    "/vehicle/status/turn_indicators_status",
    "/vehicle/status/hazard_lights_status",
    "/sensing/imu/imu_data",
    "/perception/object_recognition/objects",
    "/perception/traffic_light_recognition/traffic_signals",
    # behavior_path_planner 의 isDataReady 가 이 둘을 기다린다(brige 의 publish_dummy_perception).
    "/perception/occupancy_grid_map/map",
    "/perception/obstacle_segmentation/pointcloud",
    "/tf",
]
LATCHED = [
    "/planning/mission_planning/route",
    "/localization/initialization_state",
]

QOS_LIVE = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST)
QOS_LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)


def restamp(msg, stamp):
    """header.stamp 또는 stamp 를 재기록. TFMessage 는 각 transform 마다."""
    if hasattr(msg, "transforms"):
        for tf in msg.transforms:
            tf.header.stamp = stamp
        return
    if hasattr(msg, "header"):
        msg.header.stamp = stamp
    elif hasattr(msg, "stamp"):
        msg.stamp = stamp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--start", type=float, default=0.0, help="백 시작 후 몇 초부터")
    ap.add_argument("--duration", type=float, default=0.0, help="0 이면 끝까지")
    ap.add_argument("--topics", default="", help="쉼표 구분 추가 토픽")
    ap.add_argument("--force-autonomous", action="store_true",
                    help="/system/operation_mode/state 를 AUTONOMOUS 로 20Hz 발행")
    args = ap.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    wanted = [t for t in LIVE + LATCHED if t in types]
    wanted += [t for t in args.topics.split(",") if t and t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))

    rclpy.init()
    node = Node("bag_replay")
    pubs = {}
    cls = {}
    for t in wanted:
        cls[t] = get_message(types[t])
        qos = QOS_LATCHED if t in LATCHED else QOS_LIVE
        pubs[t] = node.create_publisher(cls[t], t, qos)

    print(f"[replay] {len(wanted)} 토픽: " + ", ".join(sorted(wanted)))

    # 차량 인터페이스가 없으면 operation_mode 가 AUTONOMOUS 로 못 올라간다. 그런데
    # behavior_path_planner 는 이 값으로 동작이 갈린다(check_transit_failure 의
    # ManualModeNearTerminal, planner_manager 의 route snap 스킵). 실주행과 같은
    # 조건을 만들려면 강제로 넣어줘야 한다.
    if args.force_autonomous:
        from autoware_adapi_v1_msgs.msg import OperationModeState
        om_pub = node.create_publisher(OperationModeState, "/system/operation_mode/state",
                                       QOS_LATCHED)

        def om_loop():
            m = OperationModeState()
            m.mode = OperationModeState.AUTONOMOUS
            m.is_autoware_control_enabled = True
            m.is_in_transition = False
            m.is_stop_mode_available = True
            m.is_autonomous_mode_available = True
            m.is_local_mode_available = True
            m.is_remote_mode_available = True
            while rclpy.ok():
                try:
                    m.stamp = node.get_clock().now().to_msg()
                    om_pub.publish(m)
                except Exception:  # 종료 중 퍼블리셔 파기
                    return
                time.sleep(0.05)

        threading.Thread(target=om_loop, daemon=True).start()
        print("[replay] operation_mode = AUTONOMOUS 강제 발행")

    # 백 전체를 메모리로. 프레임 수가 커도 이 목록은 몇 만 건 수준이다.
    frames = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        frames.append((t, topic, data))
    if not frames:
        print("[replay] 메시지 없음")
        return
    base = frames[0][0]
    rel = [(t - base) / 1e9 for t, _, _ in frames]
    i0 = bisect.bisect_left(rel, args.start)
    end = args.start + args.duration if args.duration > 0 else float("inf")

    # 래치 토픽은 재생 구간 이전 값이라도 먼저 깔아준다(경로가 대표적).
    #   TRANSIENT_LOCAL 이라도 한 번만 쏘면 디스커버리 경합에 따라 늦게 뜬 구독자가
    #   놓치는 일이 있다(실측: ManualLaneChangeHandler 는 받고 behavior_path_planner 는
    #   "waiting for route" 로 남아 경로가 아예 안 나옴). 주기적으로 다시 쏜다.
    latched_msgs = []
    for t, topic, data in frames[:i0]:
        if topic in LATCHED:
            latched_msgs.append((topic, data))

    def latched_loop():
        while rclpy.ok():
            for topic, data in latched_msgs:
                try:
                    m = deserialize_message(data, cls[topic])
                    restamp(m, node.get_clock().now().to_msg())
                    pubs[topic].publish(m)
                except Exception:  # 종료 중 퍼블리셔 파기
                    return
            time.sleep(2.0)

    if latched_msgs:
        threading.Thread(target=latched_loop, daemon=True).start()
        print(f"[replay] 래치 토픽 {len(latched_msgs)}건 2초마다 재발행")

    print(f"[replay] {args.start:.1f}s 부터 재생 (rate={args.rate}), 총 {rel[-1]:.1f}s")
    t_wall0 = time.time()
    n = 0
    for k in range(i0, len(frames)):
        if rel[k] > end:
            break
        target = t_wall0 + (rel[k] - args.start) / args.rate
        dt = target - time.time()
        if dt > 0:
            time.sleep(dt)
        _, topic, data = frames[k]
        m = deserialize_message(data, cls[topic])
        restamp(m, node.get_clock().now().to_msg())
        pubs[topic].publish(m)
        n += 1
        if n % 2000 == 0:
            print(f"[replay] t={rel[k]:6.1f}s  {n} 건")
    print(f"[replay] 완료 {n} 건")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

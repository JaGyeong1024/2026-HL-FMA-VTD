"""VTD 주행 1회를 짧은 리포트로 요약한다. 목표는 '좌회전 차선까지 흔들림 없이'."""
import math
import statistics as st
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1]
BP = "/planning/scenario_planning/lane_driving/behavior_planning"
PATH = BP + "/path_with_lane_id"
ODOM = "/localization/kinematic_state"
STEER = "/vehicle/status/steering_status"
CMD = "/control/command/control_cmd"
# 좌회전 진입 이후 경로에만 나타나는 차로들 (실측)
TURN_IDS = {14568, 14611, 14135, 14318, 10302, 14855}

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
tt = {t.name: t.type for t in r.get_all_topics_and_types()}
rtc = sorted(n for n in tt if n.startswith("/planning/cooperate_status/"))
want = [x for x in (PATH, ODOM, STEER, CMD) if x in tt] + rtc
r.set_filter(rosbag2_py.StorageFilter(topics=want))
cache = {}
t0 = None
ego = []
steer = []
cmd = []
paths = []
uuids = {}
STATE = {0: "W", 1: "RUN", 2: "ABT", 3: "OK", 4: "FAIL"}
while r.has_next():
    topic, data, t = r.read_next()
    if t0 is None:
        t0 = t
    rel = (t - t0) / 1e9
    if topic not in cache:
        cache[topic] = get_message(tt[topic])
    m = deserialize_message(data, cache[topic])
    if topic == ODOM:
        ego.append((rel, m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.x))
    elif topic == STEER:
        steer.append((rel, m.steering_tire_angle))
    elif topic == CMD:
        cmd.append((rel, m.lateral.steering_tire_angle))
    elif topic == PATH:
        pts = [(p.point.pose.position.x, p.point.pose.position.y) for p in m.points]
        ids = {i for p in m.points for i in p.lane_ids}
        paths.append((rel, pts, ids))
    elif topic in rtc:
        for s in m.statuses:
            u = bytes(s.uuid.uuid).hex()[:8]
            uuids.setdefault((topic, u), STATE.get(s.state.type, "?"))

print("=== %s ===" % BAG.split("/")[-2])
if not ego:
    print("주행 데이터 없음")
    sys.exit()
dur = ego[-1][0]
dist = sum(math.dist(ego[i][1:3], ego[i - 1][1:3]) for i in range(1, len(ego)))
vmax = max(e[3] for e in ego)
print("주행 %.0fs, 이동 %.0fm, 최고속 %.1f m/s, 최종 (%.1f, %.1f)"
      % (dur, dist, vmax, ego[-1][1], ego[-1][2]))

# 정지 구간 (v<0.2 가 3초 이상)
stops = []
i = 0
while i < len(ego):
    if ego[i][3] < 0.2:
        j = i
        while j < len(ego) and ego[j][3] < 0.2:
            j += 1
        if ego[j - 1][0] - ego[i][0] >= 3.0:
            stops.append((ego[i][0], ego[j - 1][0], ego[i][1], ego[i][2]))
        i = j
    else:
        i += 1
print("3초 이상 정지: %d회%s" % (len(stops), "".join(
    "  [t=%.0f~%.0fs (%.1f,%.1f)]" % s for s in stops[:4])))

# 좌회전 차로 진입 여부
first_turn = next((p[0] for p in paths if p[2] & TURN_IDS), None)
ego_turn = None
for p in paths:
    if p[2] & TURN_IDS:
        pass
print("좌회전 경로 최초 등장: %s" % ("t=%.1fs" % first_turn if first_turn else "없음"))
have = [p for p in paths if p[2] & TURN_IDS]
print("좌회전 경로 유지: %d / %d 사이클 (%.0f%%)"
      % (len(have), len(paths), 100.0 * len(have) / max(1, len(paths))))

# 경로 길이 붕괴
short = [p for p in paths if len(p[1]) <= 5]
print("경로 5점 이하: %d 사이클" % len(short))

# 조향 흔들림: 명령 각속도
def wobble(series, name):
    if len(series) < 3:
        return
    rates = []
    for i in range(1, len(series)):
        dt = series[i][0] - series[i - 1][0]
        if 0 < dt < 0.5:
            rates.append(abs(series[i][1] - series[i - 1][1]) / dt)
    if not rates:
        return
    rs = sorted(rates)
    # 부호 반전 횟수 = 좌우 떨림
    flips = sum(1 for i in range(2, len(series))
                if (series[i][1] - series[i - 1][1]) * (series[i - 1][1] - series[i - 2][1]) < 0)
    print("%s 각속도 평균 %.3f p95 %.3f 최대 %.3f rad/s, 방향반전 %d회 (%.1f/s)"
          % (name, st.mean(rates), rs[int(len(rs) * .95)], max(rates), flips, flips / max(1e-9, dur)))

wobble(cmd, "조향명령")
wobble(steer, "실조향  ")

# RTC 승인 단위
from collections import Counter
c = Counter(k[0].split("/")[-1] for k in uuids)
print("RTC 항목: " + ", ".join("%s %d" % (k, v) for k, v in sorted(c.items())) or "없음")

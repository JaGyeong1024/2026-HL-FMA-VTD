"""조향 부호 실측 — VTD에 +steer를 보내고 heading 변화를 관측.

usage: python3 test_steer_sign.py [host] [steer_rad] [accel] [sec]

원리: 정지 상태에서 accel로 가속하며 steer(+)를 유지.
heading(rad)이 증가(CCW, 좌회전)하면 "+steer = 좌" → Autoware 규약(좌+)과 동일, steer_sign=+1
heading이 감소(CW, 우회전)하면 "+steer = 우" → steer_sign=-1 (예제 코드의 -steer 반전과 일치)
"""
import math
import sys
import time

from hlvtd_io import VTDClient

host = sys.argv[1] if len(sys.argv) > 1 else "192.168.50.11"
steer = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2
accel = float(sys.argv[3]) if len(sys.argv) > 3 else 1.5
dur = float(sys.argv[4]) if len(sys.argv) > 4 else 5.0

c = VTDClient(host)
s0 = c.recv_state()
print(f"시작: pos=({s0.x:.1f},{s0.y:.1f}) heading={s0.heading:.4f} rad")

t0 = time.time()
last = s0
n = 0
while time.time() - t0 < dur:
    c.send_ctrl(steer, accel, 0)
    last = c.recv_state()
    n += 1
    if n % 20 == 0:
        dh = (last.heading - s0.heading + math.pi) % (2 * math.pi) - math.pi
        print(f"t={time.time()-t0:4.1f}s pos=({last.x:.1f},{last.y:.1f}) "
              f"heading={last.heading:.4f} Δheading={dh:+.4f}")
c.send_ctrl(0.0, -3.0, 0)  # 감속 정지

dh = (last.heading - s0.heading + math.pi) % (2 * math.pi) - math.pi
dist = math.hypot(last.x - s0.x, last.y - s0.y)
print(f"\n결과: 이동 {dist:.1f}m, Δheading = {dh:+.4f} rad ({math.degrees(dh):+.1f}°)")
if abs(dh) < 0.02:
    print("판정 불가 — 거의 안 움직임 (시뮬 Start 상태·리스폰 확인)")
elif dh > 0:
    print(f"판정: 패킷 +steer = 좌회전(CCW) → Autoware(좌+)와 동일 → bridge steer_sign = +1.0")
else:
    print(f"판정: 패킷 +steer = 우회전(CW) → Autoware(좌+)와 반대 → bridge steer_sign = -1.0")
c.close()

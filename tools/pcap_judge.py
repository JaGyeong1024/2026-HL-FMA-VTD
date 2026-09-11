#!/usr/bin/env python3
"""pcap 만으로 규정 판정: 접촉(footprint 겹침), 보행자/자전거 최근접, 최고속도, 신호 전이.
사용: python3 pcap_judge.py <vtd.pcap> <bridge 패키지 경로(src/hlfma/vtd_autoware_bridge)>"""
import sys, struct, math, collections
sys.path.insert(0, sys.argv[2])
from vtd_autoware_bridge import protocol
EGO_L, EGO_W, FRONT, REAR = 4.9, 1.9, 3.808, 1.1   # 아이오닉6, base_link 기준
def packets(path):
    f = open(path, 'rb'); gh = f.read(24); magic = struct.unpack('<I', gh[:4])[0]
    end = '<' if magic in (0xa1b2c3d4, 0xa1b23c4d) else '>'; nano = magic in (0xa1b23c4d, 0x4d3cb2a1)
    buf = b''
    while True:
        h = f.read(16)
        if len(h) < 16: return
        ts, tu, il, _ = struct.unpack(end + 'IIII', h); data = f.read(il); t = ts + tu / (1e9 if nano else 1e6)
        if len(data) < 54: continue
        eth = 14; ihl = (data[eth] & 0xF) * 4
        if data[eth + 9] != 6: continue
        if struct.unpack('!H', data[eth + ihl:eth + ihl + 2])[0] != 9910: continue
        doff = (data[eth + ihl + 12] >> 4) * 4; buf += data[eth + ihl + doff:]
        while len(buf) >= protocol.DATA_SIZE:
            pkt, buf = buf[:protocol.DATA_SIZE], buf[protocol.DATA_SIZE:]
            try: yield t, protocol.unpack_data(pkt)
            except Exception: pass
t0 = None; prev = None; vmax = (0, 0); contacts = []; near = {}; tl = []; last_tl = None; vru_ep = {}
for t, st in packets(sys.argv[1]):
    if t0 is None: t0 = t
    rt = t - t0
    v = 0.0
    if prev is not None and t - prev[0] > 1e-3:
        v = math.hypot(st.x - prev[1], st.y - prev[2]) / (t - prev[0])
        if v > 40: v = prev[3]  # 위치 튐
    prev = (t, st.x, st.y, v)
    if v > vmax[0]: vmax = (v, rt)
    cur = (st.tl_id, st.tl_state)
    if cur != last_tl: tl.append((rt, st.tl_id, st.tl_state)); last_tl = cur
    c, s = math.cos(st.heading), math.sin(st.heading)
    for (oid, x, y, z, h, sp, L, W, H) in st.objects:
        dx, dy = x - st.x, y - st.y; lon = dx * c + dy * s; lat = -dx * s + dy * c
        if abs(lon) > 60 or abs(lat) > 15: continue
        # footprint 간격(축 정렬 근사: 객체를 자차 좌표축에 정렬된 박스로 봄 → 보수적)
        oL = abs(L * math.cos(h - st.heading)) + abs(W * math.sin(h - st.heading)); oW = abs(L * math.sin(h - st.heading)) + abs(W * math.cos(h - st.heading))
        lon_gap = (lon - oL / 2 - FRONT) if lon >= 0 else (-lon - oL / 2 - REAR)
        lat_gap = abs(lat) - oW / 2 - EGO_W / 2
        gap = max(lon_gap, lat_gap)
        vru = H >= 1.2 and L < 2.8
        kind = 'VRU' if vru else ('정지물' if H < 0.6 else '차량')
        e = near.setdefault(oid, [1e9, 0, 0, kind, 0, 0, 0])
        if gap < e[0]: e[:] = [gap, rt, v * 3.6, kind, lon, lat, sp]
        if gap < 0 and (not contacts or contacts[-1][1] != oid or rt - contacts[-1][0] > 2): contacts.append((rt, oid, kind, v * 3.6, lon, lat, lon_gap, lat_gap))
        if vru and 0 < lon < 50 and abs(lat) < 12:
            ep = vru_ep.setdefault(oid, [rt, rt, 0.0]); ep[1] = rt; ep[2] = max(ep[2], v * 3.6)
print(f"최고속도 {vmax[0]*3.6:.1f} km/h @t={vmax[1]:.0f}s  (기준 50)")
print(f"접촉(footprint 겹침) {len(contacts)}건:")
for cnt in contacts: print(f"  t={cnt[0]:.1f}s 객체{cnt[1]} {cnt[2]} 자차 {cnt[3]:.0f}km/h 전방{cnt[4]:+.1f} 횡{cnt[5]:+.1f} (종간격 {cnt[6]:+.2f} 횡간격 {cnt[7]:+.2f})")
print("객체별 최근접 간격 (1.0 m 미만만):")
for oid, e in sorted(near.items(), key=lambda x: x[1][0]):
    if e[0] < 1.0: print(f"  객체{oid} {e[3]} 간격 {e[0]:+.2f}m @t={e[1]:.0f}s 자차 {e[2]:.0f}km/h 전방{e[4]:+.1f} 횡{e[5]:+.1f} 객체속도 {e[6]:.1f}")
print("보행자/자전거 접근(전방 50 m·횡 12 m 안) 중 자차 최고속도:")
for oid, ep in sorted(vru_ep.items(), key=lambda x: x[1][0]): print(f"  객체{oid} t={ep[0]:.0f}~{ep[1]:.0f}s 자차최고 {ep[2]:.0f}km/h 최근접 {near[oid][0]:+.2f}m")
print(f"신호 전이 {len(tl)}회 (적색 전이만):")
for r in tl:
    if r[2] == 1: print(f"  t={r[0]:.1f}s 신호{r[1]} 적색")

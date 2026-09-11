#!/usr/bin/env python3
"""주행 하나를 로그+토픽 한 번에 분석. usage: analyze.py <로그디렉터리>"""
import sys, os, glob, math, collections, re
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

D=sys.argv[1].rstrip('/')
bag=(glob.glob(f"{D}/bag/*.mcap") or [f"{D}/bag"])[0]
log=f"{D}/autoware.log"
print(f"=== {os.path.basename(D)} ===\n")

# ---------- 로그 ----------
if os.path.exists(log):
    txt=open(log,errors="ignore").read()
    pat=[("자율 해제/MRM", r"MRM State changed: [A-Z_]+ -> [A-Z_]+"),
         ("차선변경 진단", r"\[HLFMA[^\]]*\][^|\n]{0,60}"),
         ("회피", r"static_obstacle_avoidance[^\n]{0,40}"),
         ("진단 ERROR", r"- /autoware/[a-z_/]+ (?:ERROR|STALE)")]
    for name,p in pat:
        c=collections.Counter(re.findall(p,txt))
        if c:
            print(f"[로그] {name}")
            for k,v in c.most_common(6): print(f"   {v:5d}  {k.strip()[:78]}")
            print()

# ---------- 토픽 ----------
r=SequentialReader(); r.open(StorageOptions(uri=bag,storage_id='mcap'),ConverterOptions('',''))
tt={t.name:t.type for t in r.get_all_topics_and_types()}
def get(n): return get_message(tt[n]) if n in tt else None
W={n:get(n) for n in tt if any(k in n for k in
   ("kinematic_state","velocity_status","control_cmd","objects","planning_factors",
    "cooperate_status","avoidance_debug","max_velocity","operation_mode/state"))}
t0=None; ego=[]; fac=collections.defaultdict(int); stop=collections.defaultdict(list)
rtc=collections.Counter(); avoid=collections.Counter(); vlim=[]; mode=[]
while r.has_next():
    topic,d,ts=r.read_next()
    if topic not in W or W[topic] is None: continue
    t=ts/1e9
    if t0 is None: t0=t
    rel=t-t0
    m=deserialize_message(d,W[topic])
    if "kinematic_state" in topic:
        p=m.pose.pose.position; ego.append((rel,p.x,p.y,m.twist.twist.linear.x))
    elif "avoidance_debug" in topic:
        for a in getattr(m,"avoidance_info",[]) or []:
            avoid[getattr(a,"failed_reason","?")]+=1
    elif "cooperate_status" in topic:
        for st in getattr(m,"statuses",[]):
            rtc[(topic.split('/')[-1], bool(st.safe), int(st.command_status.type))]+=1
    elif "planning_factors" in topic:
        n=topic.split('/')[-1]
        for f in getattr(m,"factors",[]):
            fac[n]+=1
            for cp in getattr(f,"control_points",[]):
                if getattr(cp,"velocity",1)<0.1: stop[n].append(getattr(cp,"distance",0))
    elif "max_velocity" in topic:
        vlim.append((round(rel,1), round(getattr(m,"max_velocity",-1),2)))
    elif "operation_mode/state" in topic:
        mode.append((round(rel,1), m.mode))

if ego:
    mv=[e for e in ego if e[3]>0.3]
    print(f"[자차] 관측 {ego[-1][0]:.0f}s  최대속도 {max(e[3] for e in ego)*3.6:.1f} km/h  "
          f"이동거리 {math.dist(ego[0][1:3],ego[-1][1:3]):.1f} m")
    st=[e for e in ego if e[3]<0.2 and e[0]>10]
    if st: print(f"       최초 정지 t={st[0][0]:.1f}s")
if mode:
    ch=[m for i,m in enumerate(mode) if i==0 or m[1]!=mode[i-1][1]]
    print(f"[모드] " + " → ".join(f"t{a}s:{b}" for a,b in ch[:8]))
if vlim:
    u=[]; 
    for a in vlim:
        if not u or u[-1][1]!=a[1]: u.append(a)
    print(f"[속도상한] " + ", ".join(f"t{a}s:{b}" for a,b in u[:8]))
if avoid:
    print("[회피 거절 사유]")
    for k,v in avoid.most_common(8): print(f"   {v:5d}  {k}")
if stop:
    print("[정지 요인]")
    for k,v in sorted(stop.items(), key=lambda x:-len(x[1])):
        print(f"   {k:32s} {len(v):5d}회  최소거리 {min(v):7.2f} m")
if rtc:
    print("[RTC]")
    for k,v in rtc.most_common(6): print(f"   {k[0]:34s} safe={k[1]} cmd={k[2]}  {v}")

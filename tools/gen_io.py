import os, re, collections, io, datetime
D="/tmp/rosgraph"
SKIP_T=re.compile(r"(^/rosout$|^/parameter_events$|^/tf|^/clock$|/debug/|/virtual_wall|marker|published_time|processing_time|/config_logger|describe_parameters|get_parameter|list_parameters|set_parameters|get_type_description)")
SKIP_N=re.compile(r"(_rviz|^/rviz|transform_listener|launch_ros|_ros2cli)")
nodes={}
for f in sorted(os.listdir(D)):
    if not f.startswith("info"): continue
    lines=open(os.path.join(D,f),errors="ignore").read().splitlines()
    node=None; mode=None; sub=[]; pub=[]; srv_s=[]; srv_c=[]
    for ln in lines:
        s=ln.strip()
        if not node and s.startswith("/"): node=s; continue
        if s.startswith("Subscribers:"): mode="s"; continue
        if s.startswith("Publishers:"): mode="p"; continue
        if s.startswith("Service Servers:"): mode="ss"; continue
        if s.startswith("Service Clients:"): mode="sc"; continue
        if s.startswith("Action"): mode=None; continue
        m=re.match(r"(/\S+): (\S+)", s)
        if m and mode:
            t,ty=m.group(1),m.group(2)
            if SKIP_T.search(t): continue
            if mode=="s": sub.append((t,ty))
            elif mode=="p": pub.append((t,ty))
            elif mode=="ss": srv_s.append((t,ty))
            elif mode=="sc": srv_c.append((t,ty))
    if node and not SKIP_N.search(node):
        nodes[node]=dict(sub=sorted(set(sub)),pub=sorted(set(pub)),
                         srv_s=sorted(set(srv_s)),srv_c=sorted(set(srv_c)))

pubs=collections.defaultdict(list); subs=collections.defaultdict(list)
for n,d in nodes.items():
    for t,_ in d["pub"]: pubs[t].append(n)
    for t,_ in d["sub"]: subs[t].append(n)

o=io.StringIO()
o.write("<!-- AUTOGEN:NODE_IO START -->\n")
o.write(f"## 부록 · 노드별 입출력 (자동 생성, {datetime.date.today()} 실측)\n\n")
o.write("> 생성: 스택 기동 후 `ros2 node list` + 노드별 `ros2 node info` 캡처 → 스크립트 변환.\n")
o.write("> 잡음 제외: rosout, parameter_events, tf, debug/*, marker, published_time, 파라미터 서비스.\n")
o.write(f"> 노드 {len(nodes)}개. 갱신하려면 이 구역(AUTOGEN 마커 사이)만 교체.\n\n")

o.write("### 주행 사슬 (요약)\n\n```\n")
chain=[("vtd_autoware_bridge","/localization/kinematic_state · /perception/object_recognition/objects"),
       ("mission_planner","/planning/mission_planning/route"),
       ("behavior_path_planner","…/behavior_planning/path_with_lane_id"),
       ("behavior_velocity_planner","…/behavior_planning/path"),
       ("elastic_band_smoother","…/motion_planning/path_smoother/path"),
       ("path_optimizer","…/motion_planning/path_optimizer/trajectory"),
       ("motion_velocity_planner","…/motion_planning/motion_velocity_planner/trajectory"),
       ("velocity_smoother","/planning/scenario_planning/trajectory"),
       ("scenario_selector","/planning/trajectory"),
       ("trajectory_follower","/control/trajectory_follower/control_cmd"),
       ("vehicle_cmd_gate","/control/command/control_cmd"),
       ("vtd_autoware_bridge","→ VTD CtrlPacket")]
for i,(n,t) in enumerate(chain):
    o.write(f"{'  '*0}{n}\n    ↓ {t}\n" if i<len(chain)-1 else f"{n}\n")
o.write("```\n\n")

o.write("### 노드별 상세\n\n")
for n in sorted(nodes):
    d=nodes[n]
    if not d["sub"] and not d["pub"]: continue
    o.write(f"#### `{n}`\n\n")
    if d["sub"]:
        o.write("- **구독**\n")
        for t,ty in d["sub"]:
            src=[x for x in pubs.get(t,[]) if x!=n]
            o.write(f"  - `{t}` · {ty}" + (f"  ← {', '.join('`'+s+'`' for s in src)}" if src else "  ← (외부/없음)") + "\n")
    if d["pub"]:
        o.write("- **발행**\n")
        for t,ty in d["pub"]:
            dst=[x for x in subs.get(t,[]) if x!=n]
            o.write(f"  - `{t}` · {ty}" + (f"  → {', '.join('`'+s+'`' for s in dst)}" if dst else "  → (구독자 없음)") + "\n")
    if d["srv_s"]:
        o.write("- **서비스 제공**: " + ", ".join(f"`{t}`" for t,_ in d["srv_s"]) + "\n")
    if d["srv_c"]:
        o.write("- **서비스 호출**: " + ", ".join(f"`{t}`" for t,_ in d["srv_c"]) + "\n")
    o.write("\n")

o.write("### 구독자 없는 발행 토픽 (배선 점검용)\n\n")
orphan=[t for t in pubs if t not in subs]
for t in sorted(orphan)[:60]:
    o.write(f"- `{t}` ← {', '.join('`'+p+'`' for p in pubs[t])}\n")
o.write("\n<!-- AUTOGEN:NODE_IO END -->\n")
open("/tmp/node_io.md","w").write(o.getvalue())
print(f"노드 {len(nodes)}개, 구독자없는토픽 {len(orphan)}개 → /tmp/node_io.md ({len(o.getvalue())} bytes)")

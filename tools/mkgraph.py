import os, re, sys, collections
D="/tmp/rosgraph"
SKIP_T = re.compile(r"(^/rosout$|^/parameter_events$|^/tf|^/clock$|^/diagnostics|/debug|/virtual_wall|marker|^/system/processing_time|_time_checker|/processing_time)")
SKIP_N = re.compile(r"(_rviz|rviz|transform_listener|launch_ros|_ros2cli|^/system/topic_state_monitor|^/system/dummy_diag|duplicated_node|component_state_monitor|^/system/processing_time)")

pubs=collections.defaultdict(set); subs=collections.defaultdict(set); types={}
nodes=set()
for f in os.listdir(D):
    if not f.startswith("info"): continue
    txt=open(os.path.join(D,f),errors="ignore").read().splitlines()
    node=None; mode=None
    for ln in txt:
        s=ln.strip()
        if not node and s.startswith("/"): node=s; continue
        if s.startswith("Subscribers:"): mode="s"; continue
        if s.startswith("Publishers:"): mode="p"; continue
        if s.endswith(":") and not s.startswith("/"): mode=None; continue
        m=re.match(r"(/\S+): (\S+)", s)
        if m and mode and node:
            t,ty=m.group(1),m.group(2)
            if SKIP_T.search(t): continue
            types[t]=ty
            (pubs if mode=="p" else subs)[t].add(node)
            nodes.add(node)
nodes={n for n in nodes if not SKIP_N.search(n)}

def ns(n):
    p=n.strip("/").split("/")
    return p[0] if p else "root"

def short(t):
    return t.replace("/planning/scenario_planning/lane_driving/","…/").replace("/planning/scenario_planning/","/planning/…/") \
            .replace("/perception/object_recognition/","/perception/…/")

edges=collections.Counter()
for t,ps in pubs.items():
    for p in ps:
        if p not in nodes: continue
        for s in subs.get(t,()):
            if s not in nodes or s==p: continue
            edges[(p,s,short(t))]+=1

COLOR={"planning":"#e8f0fb","control":"#fdf0e3","system":"#eef1f2","map":"#eaf4ec",
       "perception":"#f6ecf6","localization":"#f6ecf6","vehicle":"#fdf0e3","adapi":"#eef1f2",
       "sensing":"#f6ecf6"}

def dot(fname, keep_nodes=None, title=""):
    E=[(a,b,t) for (a,b,t) in edges if (keep_nodes is None or (a in keep_nodes and b in keep_nodes))]
    N=set()
    for a,b,_ in E: N.add(a); N.add(b)
    by=collections.defaultdict(list)
    for n in N: by[ns(n)].append(n)
    out=['digraph G {','  rankdir=LR;','  splines=spline; overlap=false;',
         '  graph [fontname="Helvetica", fontsize=11, labelloc=t, label="%s"];'%title,
         '  node [shape=box, style="rounded,filled", fontname="Helvetica", fontsize=10, margin="0.12,0.06"];',
         '  edge [fontname="Helvetica", fontsize=7.5, color="#7b8a93", arrowsize=0.6];']
    for k,ns_ in sorted(by.items()):
        out.append('  subgraph cluster_%s {'%re.sub(r"\W","_",k))
        out.append('    label="%s"; fontsize=12; color="#b9c6cc"; style=rounded;'%k)
        for n in sorted(ns_):
            lbl=n.split("/")[-1]
            out.append('    "%s" [label="%s", fillcolor="%s"];'%(n,lbl,COLOR.get(k,"#f2f4f5")))
        out.append('  }')
    for a,b,t in sorted(E):
        out.append('  "%s" -> "%s" [label="%s"];'%(a,b,t))
    out.append("}")
    open(fname,"w").write("\n".join(out))
    return len(N), len(E)

# 1) 주행 사슬 (핵심)
CORE=[n for n in nodes if re.search(r"(vtd_autoware_bridge|vtd_route_node|lane_planner|mission_planner|route_selector|behavior_path_planner|behavior_velocity_planner|path_optimizer|elastic_band|motion_velocity_planner|velocity_smoother|scenario_selector|external_velocity_limit_selector|trajectory_follower|vehicle_cmd_gate|shift_decider|operation_mode_transition|planning_validator|control_validator|autonomous_emergency_braking|lane_departure_checker)", n)]
n1,e1=dot("/tmp/core.dot", set(CORE), "HL FMA 주행 사슬 (실측 ROS 그래프)")
n2,e2=dot("/tmp/full.dot", None, "HL FMA 전체 노드 그래프 (실측)")
print(f"core: 노드 {n1} 엣지 {e1}")
print(f"full: 노드 {n2} 엣지 {e2}")

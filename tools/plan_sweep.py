#!/usr/bin/env python3
"""lane_planner.plan() 가설공간 전수 탐색 + 요인별 기여도."""
import sys, itertools, collections
sys.path.insert(0, '/home/a/2026-HL-FMA-VTD-JG/hlfma_ws/src/hlfma/vtd_autoware_bridge')
import rclpy
from rclpy.logging import LoggingSeverity
from autoware_planning_msgs.msg import LaneletRoute, LaneletSegment, LaneletPrimitive
from autoware_perception_msgs.msg import PredictedObjects, PredictedObject
from vtd_autoware_bridge.lane_planner import LanePlanner
from vtd_autoware_bridge import lane_planner as LP

SEGS=[[16249],[15750],[15379,15414,15449,15484],[15194,15207,15220,15233,15246],
      [14855,14890,14925,14960,14995],[14611,14633,14655,14677,14699],[18858],[14135],
      [14318],[14568],[20722],[10302],[150949],[17149,17182],[17058,17067],
      [16943,16976],[21435],[13392,13467,13542,13617],[13727,13741,13755,13769],
      [13998,14031,14064,14097],[18337]]
PREF=[16249,15750,15379,15194,14855,14611,18858,14135,14318,14568,20722,10302,150949,
      17182,17067,16976,21435,13392,13741,14031,18337]
CARS=[(302.926,-20.021),(296.979,-15.850),(299.745,-21.288),
      (295.027,-1.515),(297.467,-8.447),(300.438,-14.665)]

def route():
    r=LaneletRoute()
    for lanes,p in zip(SEGS,PREF):
        sg=LaneletSegment()
        sg.primitives=[LaneletPrimitive(id=i,primitive_type='lane') for i in lanes]
        sg.preferred_primitive=LaneletPrimitive(id=p,primitive_type='lane')
        r.segments.append(sg)
    return r
def objs():
    m=PredictedObjects()
    for x,y in CARS:
        o=PredictedObject(); q=o.kinematics.initial_pose_with_covariance.pose
        q.position.x,q.position.y=x,y; m.objects.append(o)
    return m

rclpy.init(args=[])
n=LanePlanner()
n.get_logger().set_level(LoggingSeverity.FATAL)
n.omap=LP.OsmMap('/home/a/2026-HL-FMA-VTD-JG/map/lanelet2_map.osm')
n.route=route(); n.orig_preferred=list(PREF); n.objs=objs()

# 축
AX = collections.OrderedDict(
    start   =[(15750,0.0),(15414,0.0),(15414,20.0),(15379,10.0)],
    v       =[13.7,8.0,4.0,2.0],
    prep    =[1.0,0.6,0.3,0.0],
    horizon =[250.0,150.0,100.0],
    objm    =[6.0,4.0,3.0],          # OBJ_MARGIN
    lcpen   =[8.0,2.0],              # LC_PENALTY
    offr    =[0.6,0.0],              # OFF_ROUTE
    goal    =['end','target'],       # 목표 조건
)
base_objm, base_lcpen, base_offr = LP.OBJ_MARGIN, LP.LC_PENALTY, LP.OFF_ROUTE
rows=[]
names=list(AX); combos=list(itertools.product(*AX.values()))
for combo in combos:
    d=dict(zip(names,combo))
    LP.OBJ_MARGIN,LP.LC_PENALTY,LP.OFF_ROUTE = d['objm'],d['lcpen'],d['offr']
    n.horizon=d['horizon']; n.lc_prepare_s=d['prep']
    sl,ss=d['start']
    seg=n.seg_index_of(sl)
    if seg is None: continue
    corr=n.corridor(seg)
    if len(corr)<2: continue
    if d['goal']=='target':
        reach=n.must_reach(corr); ti=None
        for k,(i,lanes,ln) in enumerate(corr):
            if reach[k] and set(reach[k])!=set(lanes): ti=k; break
        if ti is not None and ti>=1: corr=corr[:ti+1]
    try: path,lc=n.plan(corr,sl,ss,d['v'])
    except Exception: path,lc=None,-1
    lanes=[]
    if path:
        for lid,c in path:
            if not lanes or lanes[-1]!=lid: lanes.append(lid)
    rows.append((d, path is not None, lc, lanes))
LP.OBJ_MARGIN,LP.LC_PENALTY,LP.OFF_ROUTE=base_objm,base_lcpen,base_offr

ok=[r for r in rows if r[1]]
print('총 조합 %d, 성공 %d (%.0f%%)'%(len(rows),len(ok),100*len(ok)/max(1,len(rows))))
print()
print('=== 요인별 성공률 (한 축의 값을 고정했을 때) ===')
for ax in names:
    line=[]
    for val in AX[ax]:
        sub=[r for r in rows if r[0][ax]==val]
        s=sum(1 for r in sub if r[1])
        line.append('%s=%s:%3.0f%%'%(ax,val,100*s/max(1,len(sub))))
    print('  '+'  '.join(line))
print()
print('=== 우측 우회 경로가 나온 조합 (열+1/+2 = 15449/15484/15233/15246/14960/14995 경유) ===')
RIGHT={15449,15484,15233,15246,14960,14995}
rt=[r for r in ok if RIGHT & set(r[3])]
print('  %d개 / 성공 %d개'%(len(rt),len(ok)))
seen=set()
for d,_,lc,lanes in rt[:6]:
    k=tuple(lanes)
    if k in seen: continue
    seen.add(k)
    print('   %s lc_len=%.0f'%({a:d[a] for a in ('start','v','prep','goal','objm')},lc))
    print('     '+'→'.join(map(str,lanes[:14])))
print()
print('=== 실패가 확정적인 조합 (성공 0%) ===')
for ax in names:
    for val in AX[ax]:
        sub=[r for r in rows if r[0][ax]==val]
        if sub and not any(r[1] for r in sub):
            print('  %s=%s 이면 항상 실패 (%d조합)'%(ax,val,len(sub)))
rclpy.shutdown()

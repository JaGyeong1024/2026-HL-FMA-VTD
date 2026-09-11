import sys, itertools
exec(open('/tmp/sweep.py').read().split("rows=[]")[0])
RIGHT={15449,15484,15233,15246,14960,14995}
real_must_reach = n.must_reach

def run(d):
    LP.OBJ_MARGIN,LP.LC_PENALTY,LP.OFF_ROUTE=d['objm'],d['lcpen'],d['offr']
    n.horizon=d['horizon']; n.lc_prepare_s=d['prep']
    sl,ss=d['start']; seg=n.seg_index_of(sl)
    if seg is None: return None
    n.must_reach = real_must_reach
    corr=n.corridor(seg)
    if len(corr)<2: return None
    if d['goal']=='target':
        full=real_must_reach(corr); ti=None
        for k,(i,lanes,ln) in enumerate(corr):
            if full[k] and set(full[k])!=set(lanes): ti=k; break
        if ti is None or ti<1: return None
        corr=corr[:ti+1]
        sliced=full[:ti+1]
        n.must_reach = lambda c, _s=sliced: _s      # 전체 회랑 기준 reach 유지
    try: path,lc=n.plan(corr,sl,ss,d['v'])
    except Exception: path,lc=None,-1
    n.must_reach = real_must_reach
    lanes=[]
    if path:
        for lid,c in path:
            if not lanes or lanes[-1]!=lid: lanes.append(lid)
    return (d, path is not None, lc, lanes)

names=list(AX); rows=[]
for combo in itertools.product(*AX.values()):
    r=run(dict(zip(names,combo)))
    if r: rows.append(r)

ok=[r for r in rows if r[1]]
rt=[r for r in ok if RIGHT & set(r[3])]
print('총 %d, 성공 %d (%.0f%%), 그중 우측경유 %d (%.0f%%)'
      %(len(rows),len(ok),100*len(ok)/len(rows),len(rt),100*len(rt)/max(1,len(ok))))
print()
def tab(a,b,pred,label):
    print('=== %s ==='%label)
    print('  %-10s'%(a+'\\'+b), *['%8s'%str(y) for y in AX[b]])
    for x in AX[a]:
        cells=[]
        for y in AX[b]:
            sub=[r for r in rows if r[0][a]==x and r[0][b]==y]
            s=sum(1 for r in sub if pred(r))
            cells.append('%7.0f%%'%(100*s/max(1,len(sub))))
        print('  %-10s'%str(x), *cells)
    print()
tab('goal','prep',lambda r:r[1],'성공률: goal × prep')
tab('goal','prep',lambda r:r[1] and RIGHT&set(r[3]),'★우측 우회가 나온 비율: goal × prep')
tab('start','prep',lambda r:r[1] and RIGHT&set(r[3]),'★우측 우회: start × prep (goal 양쪽 합산)')

print('=== 현재 파라미터(objm6/lcpen8/offr0.6/h250) 에서 goal×prep 별 실제 경로 ===')
for g in ('end','target'):
    for pr in AX['prep']:
        m=[r for r in rows if r[0]['goal']==g and r[0]['prep']==pr
           and r[0]['start']==(15750,0.0) and r[0]['v']==13.7
           and r[0]['objm']==6.0 and r[0]['lcpen']==8.0 and r[0]['offr']==0.6
           and r[0]['horizon']==250.0]
        if not m: continue
        d,okk,lc,ln=m[0]
        tag='★우측' if RIGHT&set(ln) else ('좌측' if okk else '')
        print('  goal=%-6s prep=%.1f lc=%2.0f %s %-6s %s'
              %(g,pr,lc,'성공' if okk else '실패',tag,'→'.join(map(str,ln[:12]))))
rclpy.shutdown()

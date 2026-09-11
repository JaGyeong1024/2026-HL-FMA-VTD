#!/usr/bin/env python3
"""하네스 실행 결과 요약표 생성: 케이스별 최신 실행 디렉터리의 result.txt·metrics.json → markdown.
usage: summarize.py [케이스 이름…]   (기본: ~/hlfma/logs/harness 의 모든 케이스, 각 최신 실행)
"""
import os, sys, json, glob, re
L = os.path.expanduser('~/hlfma/logs/harness')
names = sys.argv[1:]
runs = {}
for d in sorted(glob.glob(os.path.join(L, '*_????_??????'))):
    m = re.match(r'(.+)_\d{4}_\d{6}$', os.path.basename(d))
    if m and (not names or m.group(1) in names):
        runs[m.group(1)] = d      # 정렬상 마지막 = 최신
print('| 케이스 | 기대 | 실측 | 판정 | 로그 |')
print('|---|---|---|---|---|')
for name, d in runs.items():
    res = open(os.path.join(d, 'result.txt'), errors='replace').read() if os.path.exists(os.path.join(d, 'result.txt')) else ''
    exp = ''
    conf = os.path.join(d, 'case.conf')
    if os.path.exists(conf):
        for line in open(conf):
            if line.startswith('EXPECT'):
                exp = line.split('=', 1)[1].strip().strip('"')
    facts = [l.strip()[7:] for l in res.splitlines() if '[INFO]' in l and any(k in l for k in ('간격', '재출발', '최소거리', '통과=', '계획', '실측', '리스폰 t'))]
    verdict = ' / '.join(l.strip() for l in res.splitlines() if '[PASS]' in l or '[FAIL]' in l)
    print(f"| {name} | {exp} | {'; '.join(facts)} | {verdict} | {os.path.basename(d)} |")

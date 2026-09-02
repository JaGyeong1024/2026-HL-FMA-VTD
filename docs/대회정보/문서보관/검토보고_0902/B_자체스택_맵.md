# 담당 B — 자체 파이썬 스택 + xodr 파서 + Lanelet2 맵 파이프라인 적대적 검토

작성 2026-09-02. 대상: `/home/a/HL_FMA/controller/*.py`, `controller/out/livinglab_lanelet2_native.osm`,
`실습파일/HL_FMA_VTD_LivingLab.{xodr,osgb,xml}`, `route_example.csv`, `controller/real_route_path1.csv`.
검증 스크립트는 전부 실제 실행함 (`/tmp/.../scratchpad/chk1.py ~ chk15.py`).

---

## ① 구조 요약

| 파일 | 역할 | 상태 |
|---|---|---|
| `xodr_map.py` (306줄) | OpenDRIVE 파서. planView(line/arc/spiral/poly3), laneOffset, laneSection, width 다항식, roadMark, signal, road link | 기하 정확도 **실측 우수**, 의미정보(신호 validity·object) **미파싱** |
| `lane_graph.py` (203줄) | 방향성 차선 그래프 + Dijkstra 경로 생성 | 연결성 **실측 우수**, 방향·실선 제약 **없음** |
| `controller.py` (94줄) | Pure Pursuit + 곡률 속도 프로파일 + NaN 가드 | **MVP 그대로**. 15개 채점항목 중 1개만 구현 |
| `hlvtd_io.py` | 9910 패킷 팩/언팩 + 1109B 재조립 클라이언트 | 정상. 단 backlog 폐기가 속도 추정을 깨뜨림 |
| `run_real.py` (203줄) | 실기 러너 (SCP 재시작 → 경로 → 20Hz 루프 → 통계·플롯) | **기본 동작이 SCP로 시뮬을 정지·재로드** ← 대회장 치명 |
| `export_lanelet2.py` (311줄) | xodr → Lanelet2 .osm 직접 익스포터 | 기하 우수, **신호등 부착 방향이 50% 틀림** |
| `check_route.py` | 당일 경로 CSV 점검 원커맨드 | 좋은 도구인데 **제어기 PC에 없음** |
| `mock_vtd.py` / `run_closed_loop.py` | 자전거 모델 모의 서버 + 폐루프 회귀 | 동작 확인(822m 완주, 횡오차 평균 0.30m) |
| `scp_ctrl.py` / `set_view_camera.py` / `test_steer_sign.py` / `extract_scenario_route.py` | 연구실 전용 보조 도구 | 대회장에서는 쓸 일 없음 |
| `convert_lanelet2.py` / `validate_*.py` | crdesigner 경로(백업) + 검증 | 참고용 |

**xodr 실측 프로필**: rev 1.8, road 651, junction 94, geometry 4000 (line 1527 / spiral 1246 / arc 656 / poly3 571,
**paramPoly3 0건** → 파서의 미지원은 실해 없음), laneSection 901, roadMark 3000(solid 1752 / broken 1111 /
none 125 / solid solid 12; color yellow 1730 / standard 1270; laneChange none 1764 / both 1236),
signal 646(전부 dynamic, type 1000008/1000012/1000020), controller 214, object 6344, **lane `<speed>` 0건**,
**road `<type>` 0건**, **userData 0건**, `<surface>` 651개 전부 빈 태그.

---

## ② 확인된 결함 (심각도 순)

### 🚨 B-1 [치명] `run_real.py` 기본 실행이 대회장 VTD를 정지·재로드시킨다
`run_real.py:57-76`. `ATTACH = "attach" in sys.argv` 이므로 **인자 없이 실행하면 attach가 아니라 재시작 모드**다.
```
scp_ctrl.send(bus, '<SimCtrl><Stop /></SimCtrl>')
scp_ctrl.send(bus, f'<SimCtrl><LoadScenario filename="HL_FMA_VTD_LivingLab_real.xml" /></SimCtrl>')
scp_ctrl.send(bus, '<SimCtrl><Init mode="operation" /></SimCtrl>')  … <Start/>
```
대회장/사전테스트에서는 **운영측이 채점 시나리오를 돌리고 있다.** 여기에 SCP 48179로 Stop + 우리 시나리오
LoadScenario를 던지면 채점 세션을 파괴한다. 게다가 `HL_FMA_VTD_LivingLab_real.xml`은 대회장 VTD에 존재하지도
않으므로 Init이 실패해 `SystemExit`(:69)로 끝난다 → 20분 중 최소 2분(Stop 2s + Load 2s + InitDone 120s 타임아웃) 소모.
todo0901 §3에 "사전테스트는 `run_real.py <csv>`로 나감"이라고 적혀 있는데, **그 사용법이 곧 이 경로다.**
→ 기본값을 attach로 뒤집고, SCP 재시작은 `--restart` 명시 옵션으로만.

부수: `run_real.py:78-87` 관전 카메라 SCP는 **attach 모드에서도 무조건 전송**된다. 대회장 화면을 우리가 바꾸는 셈.

### 🚨 B-2 [치명] 익스포트된 Lanelet2 맵에서 신호등 646개 중 **323개(50%)가 반대 차로에 붙어 있다**
`export_lanelet2.py:187` — `lanes = sec.right if sig.orientation == "+" else sec.left`.
**실측: xodr의 646개 signal이 전부 `orientation="+"`** (chk11). 즉 이 분기는 항상 `sec.right`만 고른다.

그런데 진짜 적용 방향은 `orientation`이 아니라 `<validity fromLane toLane>`과 t 부호에 들어 있다.
직접 확인한 예 — road 72 signal id=1: `s="0.0" t="2.0" orientation="+"` + `<validity fromLane="2" toLane="2"/>`
→ **lane +2 = 좌측 차선 = −s 주행 방향**인데 익스포터는 우측(+s) 차선에 붙였다.

정량화(chk13/chk14): 같은 road에서 3m 이내에 있는 정지선 오브젝트(`Rm_StopLine_300cm_JPN_01.flt`)의 t 부호로
적용 방향을 판정하면 **좌(−s) 담당 323개 / 우(+s) 담당 278개 / 판정불가 45개**. 그리고 646개 signal이 얹힌
laneSection은 **전부 좌·우 driving 차선을 모두 가진 양방향 구간**이다 → 323개는 **확정 오배정**.

결과: (a) −s 방향 접근로에는 신호등 regulatory element가 아예 없어 Autoware가 적색을 무시한다(항목 7, 중대 −6),
(b) +s 차로에는 교차로 **출구쪽**에 유령 정지선이 생겨 초록불에도 멈출 수 있다(항목 8).
`xodr_signal_id` 태그도 폐기하기로 했으므로(9/1 Q&A) 재작성 부담은 적다.

올바른 규칙: `<validity>`가 있으면 그 lane id 부호로, 없으면 **가장 가까운 StopLine 오브젝트의 t 부호**
(t<0 → 우측/+s, t>0 → 좌측/−s), 그것도 없으면 `sign(sig.t)`.

### 🚨 B-3 [치명] 경로 폴리라인의 중복점 → 곡률 NaN/∞ → **경로 중간에서 속도 0으로 감속 + 조향 0**
`lane_graph.build_route`가 노드 폴리라인을 이어붙일 때 접합부에 **완전 동일한 점**이 생긴다
(route_example 4곳, real_route_path1 18곳; chk9). `controller.PathTracker._curvature`(controller.py:32-41)는
`np.gradient(p, self.s)`를 쓰는데 `ds == 0`이면 0으로 나눠 `k = nan` 또는 |k| = 2.9e8 가 된다.

실측 결과 (chk9/chk10):
- `v_profile`에 **NaN 12개(822m 경로) / 36개(4.9km 경로)**
- `v_profile` 내부 최소값 **0.5 km/h (822m) / 0.0 km/h (4.9km)**, **<10 km/h 지점 76개(4.9km)**
  — 가짜 곡률(k=−116.7 @ (745,−610) 등)에서 시작해 역방향 감속 전파(controller.py:26-29)가
  **선행 수십 m를 통째로 감속 램프로 만든다.**
- NaN이 `v_target`에 들어오면 `accel = nan` → 안전가드(controller.py:89-90)가 **`steer=0.0, accel=-2.0`**으로 덮어씀.
  즉 곡선 중간에 **조향을 0으로 놓고 급제동**한다 (항목 3 차로유지, 항목 8 녹색신호 정차 위험).

822m 폐루프에서는 완주했지만(평균 35 km/h) 4.9km 실경로에서는 76개 지점이므로 실주행에서 반드시 터진다.
수정: `build_route` 반환 직전 `ds < 1e-3` 점 제거 + `_curvature`에서 `np.nan_to_num` 및 `|k| <= 0.5` 클램프.

### 🔴 B-4 [높음] 채점 15개 항목 중 **구현 1개**. 폴백 스택은 "완주 시도"만 가능하다
| No | 항목 | 구현 | 근거 |
|---|---|---|---|
| 1 | 제한속도 | △ 부분 | `V_DEFAULT = 50/3.6` 고정(controller.py:11). **마진 0** — 폐루프 실측 최고속 정확히 50.0 km/h. 평가는 RDB 실속도 기준이므로 초과 확실. 도로별 제한속도 소스 없음 |
| 2 | 보호구역 | ✗ | 없음 |
| 3 | 차로유지 | △ | Pure Pursuit만. 모의 실측 횡오차 평균 0.30 / 최대 1.35 m (822m). B-3의 조향 0 이벤트가 위협 |
| 4 | 중앙선 침범 | △ | 경로가 차선 중심이라 우발적으로만 회피. 감시 로직 없음 |
| 5 | 보도 침범 | ✗ | 없음 |
| 6 | 실선 차로변경 | ✗ | **`lane_graph._build`(:63-69)가 roadMark를 보지 않고 lc 엣지를 무조건 생성**. `LaneNode.marks` 필드는 선언만 되고 채워지지 않음(:23). 다행히 real_route_path1의 lc 9회는 모두 broken 선(chk8)이었지만 보장 아님 |
| 7 | 적색신호 정지 | ✗ | `tl_state`를 `hlvtd_io`가 파싱만 하고 `controller.py`가 **전혀 쓰지 않음**. 정지선 좌표도 미사용 |
| 8 | 녹색신호 통과 | ✗ | 오히려 B-3의 유령 감속이 위반 유발 |
| 9 | 적색점멸 일시정지 | ✗ | 없음 |
| 10 | 보행자 | ✗ | `objects[]` 파싱만, 미사용 |
| 11 | 장애물 | ✗ | 미사용 |
| 12 | 횡단보도 정차금지 | ✗ | 없음 |
| 13 | 방향지시등 | ✗ | `Controller.update`가 **항상 `0` 반환**(controller.py:94). real_route_path1은 차로변경 9회 → 경미 −3 확정 |
| 14 | 충돌 | ✗ | 없음 |
| 15 | 리스폰 | ✓ | 점프>3m 감지 + 경로 인덱스 재정렬(run_real.py:141-151) |

정리: **완전 구현 1/15(리스폰), 부분 3/15.** 폴백으로 나가면 구간마다 신호 위반(−6)·지시등(−3)이 누적된다.
사전테스트(9/3)는 채점이 없으므로 문제 없지만, **본선 폴백으로는 성립하지 않는다.**

### 🔴 B-5 [높음] 경로 계획에 **주행 방향 제약이 없다** — 역주행/중앙선 침범 잠재
`lane_graph.nearest_nodes`(:113-122)는 반경 6m 내 모든 차선을 후보로 내놓고, `build_route`는 그 거리만을
Dijkstra 진입비용으로 쓴다. 실측(chk4) 결과 **모든 waypoint에서 반대 방향 차선이 3~4m 차이로 함께 후보에 든다**
(예: route_example wp1 → `(30,0,-1)` heading 98° @0.14m, `(30,0,1)` heading −82° @3.13m).
`LANE_CHANGE_COST=30m`에 비해 방향 오선택 비용은 3m뿐이므로, 당일 CSV가 중앙선 반대편으로 1~2m만
치우쳐도 **한 leg 전체가 대향 차로로 계획될 수 있다**(항목 4 중대 −6 + 항목 14).
현재 두 경로에서는 우연히 정상. ego heading(시작점)·직전 leg 진행방향으로 후보를 필터링해야 함.

### 🟠 B-6 [중] Lanelet2 맵의 신호등이 실제의 3배로 중복 생성됨
xodr `<controller>` 214개가 각각 **type 1000008 / 1000012 / 1000020 signal 하나씩(정확히 3개)** 을 묶는다
(chk: controller type-set 히스토그램이 214개 전부 `('1000008','1000012','1000020')`, |t| 평균 3.03/3.26/2.68 →
0.35m 간격의 같은 등주 램프 모듈). **실제 신호등 지점은 214곳인데 익스포터는 646개 regulatory_element를 만든다**
(`export_lanelet2.py:178-224`). 같은 정지선에 TrafficLightGroup이 3개 겹치므로 브리지가 3개 전부에 state를
넣어주지 않으면 일부가 UNKNOWN으로 남는다. `<controller>`로 묶어 **1개 reg elem + refers 3개**가 정답.

### 🟠 B-7 [중] Lanelet2 맵 전체가 z=0 평면인데 실제 지형은 0~79 m
`export_lanelet2.Osm.add_node`가 z 기본 0이고 `<elevationProfile>`(1261 레코드, a: 0~79.3 m, 평균 53.1 m)을
파서가 아예 읽지 않는다. 차선 노드는 `ele=0`, 신호등 노드만 `ele=4.60`(zOffset). 실기에서 VTD ego z는 40~79 m로
들어온다 → **맵과 50m 어긋난 pose**. psim은 z=0 initialpose였으므로 검증되지 않은 구간이다.
브리지에서 ego z를 0으로 눌러주든지, ele를 elevationProfile로 채우든지 둘 중 하나는 본선 전에 확정 필요.

### 🟠 B-8 [중] `check_route.py`가 제어기 PC에 없다
제어기 PC `~/2026-HL-FMA-VTD/tools/`에 있는 것: `xodr_map.py, lane_graph.py, controller.py, hlvtd_io.py,
run_real.py, scp_ctrl.py, set_view_camera.py, test_steer_sign.py, extract_scenario_route.py, xodr, csv 2개, psim_*.sh`.
**없는 것: `check_route.py`, `export_lanelet2.py`, `validate_*.py`, `HL_FMA_VTD_LivingLab_real.xml`.**
`check_route.py`는 docstring부터 "대회 당일 경로 CSV 즉시 점검 — 받자마자 실행하는 원커맨드"인데 정작 현장 장비에 없다.
(`HL_FMA_VTD_LivingLab_real.xml` 부재는 B-1의 SCP 재시작이 확실히 실패한다는 뜻이기도 하다.)

### 🟡 B-9 [중저] 완주 판정 직전에 정지하도록 설계돼 있다
`PathTracker.__init__`가 `v_profile[-1] = 0.0`(controller.py:30)으로 종점 정지를 강제하고,
`build_route`는 마지막 waypoint에서 경로를 잘라낸다(lane_graph.py:196-202). 완주 판정은 **후륜축이 종료 지점을
통과**해야 성립하는데(대회정보 §7A), 현재 구성은 종료 좌표에 "도착해서 선다". 종점 뒤로 15~20 m 연장 후
그 지점을 정지 목표로 삼아야 안전하다. `finished()`의 `tol=3.0`(:52)도 통과 전 종료 판정을 낼 수 있다.

### 🟡 B-10 [중저] 속도 추정이 패킷 폐기와 충돌한다
`VTDClient.recv_state`(hlvtd_io.py:154-157)는 밀린 패킷을 버리고 최신 1개만 돌려준다. 반면
`Controller.update`는 `dt=0.05` 고정으로 위치차분을 나눈다(controller.py:67). 프레임을 2개 건너뛰면
`v_est`가 2배로 뜨고 → 불필요한 제동. 또 `run_real.py:141`의 리스폰 판정(`jump > 3.0`)도
50 km/h에서 5프레임만 밀리면 **가짜 리스폰**으로 오인해 경로 인덱스를 되감는다.
패킷에 시간 정보가 없으므로 `time.time()` 기반 dt 또는 폐기한 패킷 수 카운트로 보정해야 한다.

### 🟡 B-11 [중저] 익스포터 잔여 3건의 실제 채점 영향
- **횡단보도 미출력**: 실측 확인 — .osm에 `v="crosswalk"` 0건. Autoware crosswalk 모듈이 동작하지 않음
  → 항목 10(보행자, **중대**)·12(횡단보도 정차, 경미)를 objects 기반 장애물 정지에만 의존.
  **다만 재료는 xodr에 있다** (아래 B-12) — "횡단보도 object 2개뿐"이라는 대회정보 §13 기록은 **오류**.
- **speed_limit 일괄 "50 km/h"**: .osm 2480개 전부 동일. xodr에 lane `<speed>`·road `<type>`이 0건이라
  다른 소스가 없는 것은 사실 → 항목 2(보호구역)를 맵으로 못 푼다. B-12 참조.
- **lane_change 태그가 섹션 중점 기준**(`export_lanelet2.py:121,147,150` — `mark_rec_at(mid_ds)`):
  한 섹션 안에서 실선↔점선이 바뀌면 절반이 틀린다. 실측으로 위험도는 미확인(섹션 내 roadMark 다중 레코드 빈도 미조사).

### 🟡 B-12 [정보/기회] xodr object에 **정지선·횡단보도가 통째로 들어 있는데 전혀 쓰지 않고 있다**
`<object>` 6344개의 name 속성을 열어보니 (chk6/chk7):
- **`Rm_StopLine_300cm_JPN_01.flt` 710개** — 정지선. 폭 2.4~3.0 m, 두께 0.24~0.30 m, road/s/t 정확.
  signal.s와 비교하면 **중앙값 −0.14 m, p5~p95 = [−0.15, +0.35] m** (chk6) → 익스포터가 정지선 위치로
  signal.s를 쓰는 것은 **95% 구간에서 오차 0.35 m 이내로 검증됨**(항목 7의 2m 허용치 대비 안전).
  단 **|Δ|>5 m 이상치 32개**(최악 171.5 m: road 2819 sig 676~680) → 경로에 걸리면 치명. object 기준으로 바꾸면 해소.
- **`Rm_Warning_Crosswalk_JPN_03.flt` 400개 = 실제 횡단보도**. 각 5.0 m(종) × 1.5 m(횡) 타일이 같은 s에
  t 방향 3 m 간격으로 늘어서 도로를 가로지른다(예: road 72 s=179.5에 t=−4.4/−1.4/+1.6/+4.7).
  **(road, s)로 묶으면 횡단보도 197곳**. 이것으로 Lanelet2 crosswalk lanelet과 항목 12 판정을 만들 수 있다.
- `RM_537_ST/LT/RT`, `RM_538_SLT/SRT`, `RM_539_LUT/UT` 노면 화살표 → 차로별 회전 방향(turn_direction 정확도 향상용).
- `<signal><validity fromLane toLane>` 148건 — 좌회전 전용등 식별에 쓸 수 있다(예: `(-1,-1)`, `(2,2)` 최내측 차로 전용 68건).

### 🟢 B-13 [양호 — 반증된 우려] 파서 기하와 그래프 연결성은 신뢰할 만하다
직접 수치 검증했고 결과가 좋다:
- **spiral**: `_sample_geometry`의 사다리꼴 적분 종점 vs `scipy.integrate.quad` 고정밀 적분 —
  60개 표본 평균 8.9e-5 m, **최대 2.2 mm** (chk15). Fresnel 특수함수 불필요가 맞다.
- **poly3**: 호길이 재매개화 종점 오차 평균 3.0e-6 m, **최대 25 µm**.
- **arc**: 곡률 최소 |k|=1.16e-4 (반경 8.6 km) → `x0 + (sin h − sin hdg)/k`의 0 나눗셈 위험 없음.
- **차선 그래프 연결성**(chk2): succ 엣지 2494개, 끝점-시작점 간격 **평균 4 mm, p99 4.7 cm, 2 m 초과는 단 1건**
  (알려진 road 1927 s=106.7, (1064,−881)의 3.30 m). **방향이 뒤집힌 엣지 0건** (junction laneLink의
  contactPoint 처리가 실제로 옳다는 뜻).
- **경로 생성**: route_example 8wp→822 m, real_route_path1 14wp→4927 m, 각 0.1 s. 순환 경로 트림 버그 회귀 통과.
- **제어기 PC(python3.12/numpy1.26)에서 동일 결과, 맵 로드+그래프+경로 생성 총 0.9 s** — 콜드부팅 후 지연 요인 아님.

---

## ③ 미검증 위험

1. **`<positionRoad>` 무시**: 183개 signal이 자기 road와 **다른 road**를 positionRoad로 가리키고,
   `|s − positionRoad.s|` 평균 113 m·최대 740 m (chk5). 익스포터의 신호등 몸체 좌표(`sig.x, sig.y`)는
   소유 road 기준이라 이 183개는 물리적 위치가 틀리다. TCP로 state를 받으므로 주행에는 영향이 없을 것으로
   보이나(카메라 인식 미사용), Autoware가 등 위치를 거리 판정에 쓰는지 **미확인**.
2. **끝점 용접이 정지선/신호등 way까지 포함**: `export_lanelet2.py:264-265`가 모든 way의 첫·끝 노드를
   0.05 m로 뭉친다. stop_line 끝점이 차선 경계 끝점과 5 cm 안에 들면 병합되고, 대표 노드의 z가
   min(node id) 것으로 덮이므로 신호등 z(4.6 m)가 0으로 떨어질 수 있다. 발생 빈도 미조사.
3. **lane_change 태그 섹션 중점 근사의 실제 오분류 건수** 미측정 (B-11).
4. **`localize` 윈도우** `[i−50, i+400]`(controller.py:43-50): 자기교차 경로에서 400점(≈400 m) 앞을 보므로
   근접 병렬 구간에서 오매칭 가능. 두 경로에서는 미발생.
5. **`build_route` 반경 6 m 하드코딩**(lane_graph.py:113): 당일 CSV가 다차로 도로의 기준선 위에 찍혀 오면
   최근접 차선 중심까지 6 m를 넘어 `ValueError`로 죽는다. real_route_path1의 최대가 **5.0 m**로 이미 아슬아슬.
6. **`np.interp`로 heading 보간**: `ref_hdg`는 unwrap돼 있으나 geometry 경계에서 s가 중복되어
   `np.interp`가 어느 쪽 값을 취하는지 경계 1점에서 불확정. 실해는 mm 수준으로 추정.
7. VTD `.xml`의 `<SignalController>`는 go 15 s / attention 3 s / stop 18 s (36 s 주기) — 본선 시나리오도
   같은지 **미확인**. 같다면 잔여 녹색시간 추정이 가능하다.

---

## ④ 붉은 노면(어린이보호구역) xodr 조사 — 결론

**xodr에는 없다.** 다음을 전수 확인했다:
- `<surface>` 651개 전부 **빈 태그**(CRG 없음), road `<type>` **0건**, `userData` **0건**
- `roadMark` color 값은 `standard`(1270)·`yellow`(1730) **둘뿐** — red 없음
- lane type은 `driving/border/sidewalk/none/parking` **5종뿐** — restricted·biking 없음
- object name 122종 전수 검사 — red/school/zone/child 계열은 `BldResSchool01.flt`(학교 건물 1채, road 190, (528,−86))뿐

**대신 osgb 시각 모델에 있다.** `HL_FMA_VTD_LivingLab.osgb`(284 MB, gzip 내장 → 905 MB)를 풀어 텍스처를
추출한 결과 노면 텍스처 110종 중 **`StyleSrfBikeway.rgb`의 첫 픽셀들이 (168,71,71),(102,43,43),(58,25,25)…
= 붉은 아스팔트**다. xodr에 biking 차선이 하나도 없으므로 이 텍스처는 자전거도로가 아니라
**보호구역 붉은 포장**에 재사용된 것으로 판단된다. (참고: `StyleSrfStreetRestrict01.rgba`는 회백색+알파 데칼이라 붉은 노면 아님.)

- 텍스처 문자열 주변의 정점 배열을 휴리스틱 스캔하면 **x 478~522, y −142~−68, z≈42**의 평면 폴리곤 뭉치가 나온다
  — 학교 건물(528,−86) 바로 앞이고, `real_route_path1`이 이 박스를 **106 m 통과**한다(route_example은 0 m).
  단 osgb 씬그래프를 정식 파싱한 것이 아니라 **후보이지 확정 아님**.
- 정식 추출 경로: 시뮬 PC(VTD의 OSG 동봉)에서 `osgconv LivingLab.osgb out.osgt` → ASCII에서
  `StyleSrfBikeway.rgb` StateSet을 쓰는 Geode의 정점 좌표를 뽑으면 붉은 노면 폴리곤 좌표를 **정확히** 얻는다.

**xodr 안의 차선책**: `roadmark_speed_30.flt` 노면 "30" 표시 **71개 / road 35개**가 있고,
`RM_517_50.flt`("50" 표시) 69개와 **road 집합이 완전히 분리**된다. 다만 30 표시 도로들은 맵 전역에 흩어져 있고
학교 앞 후보 구역(road 190 등)에는 30 표시가 **없어** 보호구역과 1:1이 아니다(9/1 Q&A "일반 노면의 30은 평가 대상 아님"과 일치).
따라서 **30 표시 = 보호구역의 상위집합도 부분집합도 아닌 별개**로 보는 게 안전하다.
감점 구조상 저속 주행 자체는 무벌점이므로, 실무적 절충은 **"30 표시 도로 + osgb에서 뽑은 붉은 구역"의 합집합에서 28 km/h 주행**이다.

---

## ⑤ 개선 제안 (우선순위)

**내일(9/3 사전테스트) 전에 반드시 — 30분 작업**
1. **B-1**: `run_real.py`의 기본값을 attach로 뒤집기. `RESTART = "--restart" in sys.argv`로 바꾸고
   카메라 SCP도 `--cam`일 때만. (현장에서 이 한 줄이 채점 세션을 지킨다)
2. **B-3**: `build_route` 반환 직전 `ds < 1e-3` 중복점 제거 + `_curvature`에 `nan_to_num` 및 `|k| ≤ 0.5` 클램프.
   회귀: `run_closed_loop.py 20` 완주 + `v_profile` 내부 최소값이 15 km/h 이상인지 확인.
3. **B-8**: `check_route.py`(+ `.venv` 없이 도는지 확인)를 제어기 PC `tools/`로 복사. 당일 첫 동작 =
   `python3 check_route.py <받은 csv>`.
4. **B-4-1**: `V_DEFAULT`를 `47/3.6`로. 평가가 RDB 실속도 기준이라는 9/2 답변 반영.
5. 사전테스트 실측 항목 추가: 신호등 state가 **어느 정지선 기준으로 언제 바뀌는지** 로깅
   (우리 xodr 정지선 추정치와의 오차 확인 — B-12의 32개 이상치 검증).

**본선(9/12) 전 — 맵 파이프라인**
6. **B-2**: `xodr_map.Signal`에 `validity(from,to)` 파싱 추가 → 익스포터 신호등 부착 규칙 교체
   (validity → StopLine t 부호 → sign(sig.t)). 재익스포트 후 `validate_native_osm.py` 회귀.
7. **B-6**: `<controller>` 214개 단위로 묶어 reg elem 1개 + refers 3개.
8. **B-12**: 정지선을 `Rm_StopLine_300cm_JPN_01.flt` object 기준으로 전환(이상치 32개 해소),
   `Rm_Warning_Crosswalk_JPN_03.flt`를 (road,s)로 묶어 **crosswalk lanelet 197개** 출력.
9. **B-7**: `<elevationProfile>` 파싱해 `ele` 채우기, 또는 브리지에서 ego z=0 고정 — 둘 중 하나로 확정.
10. **B-11/스쿨존**: 시뮬 PC에서 `osgconv`로 붉은 노면 폴리곤 좌표 추출 → 해당 lanelet에 `speed_limit 30 km/h`.

**본선 전 — 폴백 스택을 실제 폴백으로 만들려면 (B-4)**
11. 최소 3개만 넣어도 감점이 크게 준다: **① 신호등 정지**(state 1/2/6 + 경로상 다음 정지선, 범퍼 기준 2 m 이내 정지),
    **② 방향지시등**(node_seq의 lc 엣지 위치를 미리 알고 있으므로 3초 전 점등은 순수 계산),
    **③ 선행차/보행자 정지**(objects[]에서 경로 전방 ±2 m 내 최근접 → TTC 기반 감속).
12. **B-5**: `nearest_nodes`에 heading 인자를 추가해 진행방향 ±90° 밖 후보 제외, `build_route`는 ego heading(첫 leg)과
    직전 노드 방향(이후 leg)으로 필터. 반경도 6 → 10 m로 넓히고 실패 시 명확한 진단 출력.
13. **B-6(그래프)**: `lane_graph`의 lc 엣지 생성 시 넘는 경계의 roadMark를 확인해 `solid`면 생성 금지
    (`LaneNode.marks`를 실제로 채운다). 이미 `mark_rec_at`이 있어 10줄이면 된다.
14. **B-9**: 경로를 종점 뒤 15~20 m 연장하고 `v_profile[-1]=0`을 그 연장 끝으로 이동.
15. **B-10**: `dt`를 `time.time()` 차분으로, 리스폰 임계값을 `max(3.0, 3*v_est*dt)`로.

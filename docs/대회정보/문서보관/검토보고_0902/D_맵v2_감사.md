# 담당 D — Lanelet2 맵 v2 전수 감사 (읽기 전용)

작성 2026-09-02. 대상 `/home/a/HL_FMA/controller/out/livinglab_lanelet2_v2.osm` (33MB, 9/2 17:45 생성,
제어기 PC `map/lanelet2_map.osm` 배치판). 비교 대상 원본 xodr `/home/a/HL_FMA/실습파일/HL_FMA_VTD_LivingLab.xodr`,
익스포터 `/home/a/HL_FMA/controller/export_lanelet2.py`(529줄), 파서 `xodr_map.py`(363줄).

검증은 전부 실제 실행. lanelet2 파이썬은 **`/home/a/HL_FMA/controller/.venv/bin/python`(3.10, lanelet2 설치됨)** 에만 있음
(시스템 python3 에는 없음). 감사 스크립트는 `/tmp/claude-1000/.../scratchpad/aud/a1.py ~ a19.py`.
`export_lanelet2.py` 를 스크래치로 복사해 재실행한 결과가 **v2.osm 과 바이트 단위 동일**(`cmp` IDENTICAL) —
즉 배포된 맵은 현재 익스포터로 재현 가능하고, 아래 수치는 전부 그 재현판 기준.

---

## ① 요약 판정

**판정: 조건부 합격.** Lanelet2 구조 규약(공유 경계·연결·방향·태그)과 신호등/정지선 의미 계층은
**Autoware 소비 규칙 기준으로 결함 0건**이다. 9/1 검토(B_자체스택_맵.md)에서 치명으로 잡힌 세 건
— 신호등 50% 반대 차로(B-2), 규제요소 3배 중복(B-6), z=0(B-7) — 은 **전부 해소되었고 독립 증거로 재확인**했다.

- **B-2 해소 확인**: 방향 판정 214개 controller 중 `validity` 근거 50개가 road_end 휴리스틱과 **50/50 일치**,
  독립 증거인 정지선 object t 부호와 **197/197 일치**, 모순 0건, 증거 없음 15개(양쪽 다 정지선 object 없음/양쪽 다 있음).
  → 반대 차로 배정 의심 사례 0건.
- **B-6 해소 확인**: `lanelet당 traffic_light 규제요소 수 = {1: 353}`, 어느 lanelet 에도 안 붙은 규제요소 0개.
- **B-7 해소 확인**: `ele` min 35.55 / max 79.29 / z==0 노드 0개.
- **정지선**: 353개 (lanelet, 규제요소) 쌍 전부 진행방향과 교차, lanelet 끝까지 거리 median 0.15m / max 0.35m,
  위반 0건. 정지선 way 130개 전부가 부착 lanelet 의 좌우 경계를 완전히 가로지름 (130/130).
- **커버리지**: xodr signal 646개 → traffic_light way 646개(고유 id 646, refers 미참조 0개),
  controller 214개 → 130 접근로 규제요소, 미사용 controller 0개.

**불합격 사유가 되지 않는 이유**: 남은 결함은 (a) 미출력 요소(횡단보도·보호구역 속도·비신호 정지선),
(b) 근사(lane_change 섹션 중점), (c) xodr 원본 결함의 전파(퇴화 차선·강제 용접) 세 종류이고,
어느 것도 **맵 로딩·라우팅·신호등 정지를 깨뜨리지 않는다**(lanelet2 로드 경고/에러 0건, 두 경로 라우팅 성공).

**단, 맵 자체보다 위험한 것을 하나 찾았다 → D-1.** 경로 CSV 점을 방향 무시하고 최근접 lanelet 에 스냅하면
real_route_path1 경로가 **4772m → 11106m (2.3배)**, 차로변경 **9회 → 31회**로 부풀어 오른다. 맵 결함이 아니라
`route_node` 설계 결함이며, 이대로 두면 완주 자체가 무너진다.

---

## ② 항목별 수치

### 1. Lanelet2 구조 규약

| 검사 | 결과 | 판정 |
|---|---|---|
| (a) 인접 차선 경계 way **공유** | xodr 기준 인접 driving 쌍 **1205개**, 맵에서 `A.right == B.left` 로 공유된 쌍 **1205개 (100%)**. 같은 way 를 두 lanelet 이 같은 role 로 쓰는 사례 0건 | ✅ |
| (a′) 그 중 lanelet2 routing 관계 생성 | left/right(차로변경 가능) **997** + adjacentLeft/Right(불가) **206** = 1203 / 1205. 미생성 2쌍은 퇴화 lanelet(폭 0) | ✅ |
| (b) 연속 lanelet 끝/시작 노드 공유 | xodr link 기준 succ 쌍 **2494**, lanelet2 following **2500**. 누락 **1건**(`(2810,2,2)→(2810,1,2)`, 폭 0 퇴화), 추가 **7건** | ✅ |
| (b′) 추가 7건 성격 | 전부 간극 0.000m, heading 차 ≤1° — xodr `link` 레코드가 빠진 실제 연결의 용접 복구 (`(1776,0,1)→(4740,0,1)` @(558,−809) 등). **역주행 연결 0건** | ✅ |
| (c) 좌/우 way 방향 규약 | 좌·우 경계 way 진행방향 불일치 **0개**. 좌우 뒤집힘(centerline 진행방향 대비 left 가 오른쪽) **0개** (cross 평균이 정확히 0.0 인 6개는 폭 0 퇴화 lanelet, 부호 판정 불가) | ✅ |
| (d) 자기교차 | 경계 way 자기교차 **0개** | ✅ |
| (d′) 폭 | 평균폭 median 3.01m / p1 0.64 / p99 3.75 / **max 5.55 (>6m 0개)**. 평균폭 <2m **186개**, <1m **56개**, 최소폭 <0.10m **232개**, 전 구간 <0.10m 인 **완전 퇴화 4개** | ⚠ D-5 |
| (d″) 폭 급변 | 1m 샘플당 1m 초과 변화 **2개** (`47950` Δ1.64m, `48442` Δ1.18m — 둘 다 road 1927) | ⚠ D-6 |
| (e) id 유일성 | node+way+relation **149,071개, 중복 0** | ✅ |
| (e′) lanelet 태그 | 2480개 전부 `type=lanelet, subtype=road, location=urban, one_way=yes, speed_limit=50 km/h`. `turn_direction` 1060개 | ✅ / ⚠ D-3 |
| (e″) way 태그 | `line_thin` 2869(solid/yellow 1758, dashed/white 1078, solid/white 29, dashed/yellow 4), `virtual` 869, `line_thick(solid_solid)` 16, `road_border` 1, `stop_line` 130, `traffic_light` 646 | ✅ |

**Autoware 가 읽는 태그 중 누락된 것** (제어기 PC 소스 대조, `autoware_lanelet2_extension/.../src/validation.cpp`):

- `light_bulbs` role / `traffic_light_id` 태그 (`validation.cpp:89-99`) — **없음**. 경고만 발생, 로딩·정지 판정에는 무영향
  (등알 좌표는 `traffic_light_map_based_detector` 등 perception 전용, 우리는 perception 비활성).
- `height` 태그 (`validation.cpp:109-112`) — **있음**(0.4).
- `ele` (`validation.cpp:66-71`) — **전 노드 있음**.
- `turn_direction` (`validation.cpp:141-146`, conflicting lanelet 인데 없으면 경고) — junction 내부 707개 전부 보유.
- `location=urban`, `one_way`, `participant:*` — Autoware 소스에 직접 참조 **0건**(lanelet2 core 내부에서만 해석). 현 태그로 충분.

### 2. Routing graph 실측 (lanelet2 python, `traffic_rules Germany + Vehicle`)

```
로드 lanelet 2480, 경고/에러 0
following 2500 / previous 2500 / left 997 / right 997 / adjacentLeft 206 / adjacentRight 206
```

| 검사 | 결과 |
|---|---|
| (a) successor 없음 | **108개** (lane_graph 기준 114개). 맵 가장자리 30m 이내 11개, 내부 막다른 97개 |
| (a′) predecessor 없음 | 189개 |
| (a″) 완전 고립 | **3개** — `122179`(1306,699) / `116659`(1188,545) / `116758`(1201,549). 전부 폭 0 퇴화 |
| (b) 경로 라우팅 | route_example 8wp·real_route_path1 14wp **전부 성공, 실패 leg 0** |
| (b′) 경로 규모 (방향 고려 후보 선택) | route_example **lanelet 25개 / 896m / 차로변경 3회**, real_route_path1 **lanelet 127개 / 4772m / 차로변경 9회** (lane_graph 4927m·9회와 일치) |
| (b″) 경로 규모 (최근접 스냅) | real_route_path1 **lanelet 281개 / 11106m / 차로변경 31회** ← **D-1** |
| (c) 차로변경 관계 vs roadMark | left/right 관계 **997개**가 전부 `line_thin/dashed/lane_change=yes` 경계. xodr 인접 driving 경계 mark 분포 `broken/both 998`, `solid/none 30`, `none 177` → **998 중 997 재현**(1개는 퇴화 lanelet 손실), solid 30 → adjacentLeft 29 + 1, none 177 → virtual 177 |
| (c′) 표본 20곳 실선/점선 대조 | 불일치 **0/20**. 전수 1205쌍 대조도 **불일치 0** |

**(c) 해석**: 차로변경 태그는 `mark_rec_at(섹션 중점)` 근사임에도 섹션 중점 기준으로는 100% 정확하다.
근사가 틀리는 곳은 **한 섹션 안에서 마크가 바뀌는 32개 섹션**뿐이고 그 길이는 **938m / 전체 경계 45,069m = 2.08%** (D-4).

### 3. 신호등·정지선

| 검사 | 결과 |
|---|---|
| (a) ref_line 교차·끝 5m 이내 | (lanelet, 규제요소) 쌍 **353개 전부 교차**, 끝까지 거리 median 0.15 / p90 0.15 / **max 0.35m**, 위반 **0건**. 정지선 s/lanelet길이 > 0.9 인 것이 353/353 |
| (b) 접근로 단위 묶음의 타당성 | Autoware 소스 확인 결과 **문제없음** — 아래 상세 |
| (c) xodr signal 매핑 누락 | signal 646(전부 dynamic) → traffic_light way 646, 고유 id 646, **미포함 0개**, refers 미참조 **0개**. controller 214 → **미사용 0개** |
| (c′) 접근로 그룹 | 130개 (= 규제요소 130개). `no_target` 0, `far_from_end`(정지선이 lanelet 끝에서 5m 초과) **0** |
| (c″) 방향 판정 출처 | `validity` 50 / `junction_remap` 2 / `road_end` 162. 독립 증거 대조 → validity 일치 50/50, 정지선 t부호 일치 **197/197**, 모순 **0**, 증거없음 15 |
| (c‴) 정지선 s 이동(object − signal.s) | median +0.000m, p5 −0.15, p95 +0.15, **|Δ|>1m 1개**, object 미채택(signal.s 폴백) **11개** |
| (d) 정지선이 폭을 가로지르는가 | 130/130 이 부착 lanelet 전부의 좌우 경계를 포함. 길이 median 6.81m (min 3.35 / max 16.80), 경계 투영이 항상 [0.30, L−0.30] 안 (설계상 양끝 0.3m 여유) |
| (d′) 규제요소당 부착 lanelet 수 | {1:11, 2:63, 3:23, 4:18, 5:15} |
| (e) 경로상 15m 이내 정지선 쌍 | route_example **0쌍**, real_route_path1 **0쌍** |
| (e′) 경로상 신호등 | route_example 신호등 lanelet 4개 / 고유 규제요소 **2개**. real_route_path1 신호등 lanelet 20개 / 고유 규제요소 **16개** |
| (b′) turn_direction 출처 | 노면 화살표 334 / junction 기하 19. 화살표 종류 ST 118·RT 47·LT 57·SRT 56·SLT 29·LUT 25·UT 2 |
| (b″) turn_direction 교차검증 | 신호등 lanelet 353개 중 junction 연결 기하와 **일치 305 / 불일치 42 / 연결없음 6** |

**(b) Autoware `behavior_velocity_traffic_light_module` 소스 확인 결과** (제어기 PC
`~/2026-HL-FMA-VTD/hlfma_ws/src/autoware/.../autoware_behavior_velocity_traffic_light_module/src/manager.cpp`,
`~/autoware/src/` 와 byte-identical):

- `manager.cpp:129` `const auto lane_id = traffic_light_reg_elem.second.id();` → **module_id = lanelet id**
  (regelem id 도 stop_line way id 도 아님).
- `manager.cpp:116` `if (!stop_line) { RCLCPP_FATAL(... "No stop line at traffic_light_reg_elem_id"); continue; }`
  → **`ref_line` 이 없으면 모듈 자체가 안 생긴다**. 우리 맵은 130/130 보유 → 통과.
- `manager.cpp:206-212` `hasSameTrafficLight` 는 **refers 되는 traffic_light way id** 로 중복을 판정한다.
- `scene.cpp:376-388` `planner_data_->getTrafficSignal(traffic_light_reg_elem_.id(), ...)`,
  `data_processing.cpp:33` `result.raw[signal.traffic_light_group_id] = traffic_signal;`
  → **`TrafficSignal.traffic_light_group_id` = 규제요소 relation id**. 브리지는 151xxx~153xxx relation id 로 발행해야 한다.
- **같은 regelem 을 여러 lanelet 이 공유하는 것 자체는 무해**하다. 경로(`path.points[].lane_ids`)에는 주행 중인
  한 차선 체인만 들어가므로 옆 차선(직진/좌회전 차선)은 애초에 `getRegElemMapOnPath` 에 잡히지 않는다.
- **단 하나의 조건부 위험**: 경로가 같은 접근로의 두 lanelet 을 연달아 포함하면(접근로에서 차로변경할 때)
  `hasSameTrafficLight` 로 **먼저 등장한 lanelet 하나만** 모듈이 되고, `isTrafficSignalStop` 은 그 첫 lanelet 의
  `turn_direction` 을 쓴다. real_route_path1 에서 실제로 4곳이 이에 해당(규제요소 152150·152207·152172·151415 가
  각각 lanelet 2개에 걸침 — 예: 152150 → 57800 `straight` 다음 57846 `right`).
  브리지가 **원형 녹색/적색만 발행**하는 설계(R4)이면 `traffic_light_utils.cpp:70` 의 `CIRCLE+GREEN → return false`
  가 먼저 걸려 turn_direction 이 아예 쓰이지 않으므로 **현 설계에서는 무해**. 화살표 신호를 쓰기 시작하면 되살아난다.
- `traffic_light_utils.cpp:66-105` — `turn_direction` 속성이 없으면 `"else"` → **원형 녹색이 아닌 모든 상태에서 정지**
  (UNKNOWN 포함). 우리 맵은 신호등 lanelet 353개 전부 보유 → 통과.

### 4. xodr 대비 누락

| 요소 | xodr 실측 | v2 맵 | 영향 |
|---|---|---|---|
| lane type | driving 2480 / border 1903 / sidewalk 1027 / none 323 / parking 1 | **driving 2480 → lanelet 2480 (누락 0)**, 나머지 전부 미출력 | 정상. 단 sidewalk/border 가 없으므로 **항목 5(보도 침범) 판정 근거가 맵에 없다** — drivable area 밖으로 안 나가는 것으로만 대응 |
| junction 내부 lanelet | junction 94개, 내부 lanelet 707개 | subtype 은 `road` 유지 + `turn_direction` 부여 (straight 394 / right 170 / left 143, **none 0**) | Autoware 관례와 일치. `is_intersection_lanelet` = `hasAttribute("turn_direction")` 이므로 intersection 모듈이 정상 활성화 |
| 신호 없는 교차로 | 진입로 있는 junction 94개 중 **신호 접근로 보유 45개 / 무신호 49개** | `right_of_way` 규제요소 **0개** | intersection 모듈이 turn_direction 만으로 동작. 우선권 명시 없음 → 교차 차량 대응은 전적으로 objects 기반 |
| 일부 접근로만 신호 | **7개 junction** (2, 10, 19, 21, 45, 70, 82 — 각각 3~4 접근로 중 2~3개만) | 그대로 | xodr 원본이 그러함. 신호 없는 접근로에서 진입하면 신호 없이 통과 |
| 정지선 object | `Rm_StopLine_300cm_JPN_01` **710개 / (road,s) 301곳** | 신호등 접근로 **130곳만** ref_line 으로 출력. 나머지 **171곳 미출력** | Autoware `stop_line` 모듈은 `traffic_sign`+`type=stop_sign` 규제요소만 소비(`autoware_behavior_velocity_stop_line_module/src/manager.cpp:45-64`)하므로 **지금 구조로 출력해도 무효**. 정식화하려면 `traffic_sign/stop_sign` regelem 이 필요 |
| 횡단보도 | `Rm_Warning_Crosswalk_JPN_03` 400개 → (road,s) **197곳** | **0개** (crosswalk lanelet 0, crosswalk 규제요소 0) | **D-2. 항목 10(보행자, 중대)·12(횡단보도 정차, 경미) 무방비** |
| 속도 표지 object | `roadmark_speed_30` 71개 / road 35개, `RM_517_50` 69개 / road 16개 (**road 교집합 0**) | 미반영. 30 표시 road 위 lanelet **186개도 전부 `speed_limit=50 km/h`** | **D-3. 항목 2(보호구역) 대응 불가** |
| 붉은 노면(보호구역) | xodr 에 없음 (osgb 텍스처만) | 없음 | 별도 경로(osgconv) 필요 — 9/1 결론 유지 |

### 5. 알려진 결함 지대

**road 1927 s≈106.7, (1063,−881)** — 강제 용접 대상 7곳 중 최악:

```
⚠ 3.30m  (1927,3,5)→(1927,2,5)  at (1063,-880)
⚠ 3.30m  (1927,3,4)→(1927,2,4)  at (1063,-880)
⚠ 3.30m  (1927,3,5)→(1927,2,5)  at (1065,-882)
⚠ 0.17m  (2190,0,-2)→(2172,0,2) at (966,-539)
⚠ 0.15m  (2625,1,3)→(2625,0,3)  at (1027,74)
⚠ 0.15m  (2625,1,2)→(2625,0,2)  at (1027,74)
⚠ 0.10m  (173,4,-2)→(1225,0,-1) at (905,188)
```

강제 용접 쌍 전체 4988개의 경계 간격 **p99 0.051m**, 0.10m 초과는 위 7곳뿐. 병합점은 클러스터 중점이므로
3.30m 간극은 양쪽 노드를 각각 **1.65m 이동**시킨다 — 채점 세계(xodr) 대비 최대 1.65m 어긋난 지점이 (1063,−881) 부근 1곳.

폭 그래프(재현):
```
(1927,2,4) id 47950  길이 42m  폭 0.00 → 3.30 (min 0.00 max 3.54)
(1927,2,5) id 47995  길이 42m  폭 3.97 → 3.05
(1927,3,4) id 48442  길이 53m  폭 3.30 → 0.00 (min 0.00 max 3.72)
(1927,3,5) id 48498  길이 53m  폭 3.15 → 3.97 (min 2.71)
```
lane 4 는 0→3.3→0 으로 나타났다 사라지는 확폭 차로. **두 경로 어디에도 포함되지 않음.**

**막다른 차선**: routing graph 기준 successor 없는 lanelet **108개**(lane_graph 기준 114개).
9/1 문서의 "90개"는 재현되지 않으며 **정확한 수치는 108(라우팅) / 114(xodr link)** 이다. 차이 6은
용접이 복구한 연결 7건 − 퇴화로 끊긴 1건. 맵 가장자리(경계 30m 이내) 11개, 내부 막다른 97개.
**두 경로에는 포함되지 않는다.**

### 6. 시각 자료

`/home/a/HL_FMA/controller/out/` 에 저장 (lanelet 폴리곤 / 진행방향 화살표 / 정지선 굵은 흑선 + regelem id /
신호등 refers 적색 점 / 신호등 lanelet 은 turn_direction 별 색(청=straight, 주=left, 녹=right) /
real_route_path1 경로 적색 / CSV 점 노란 별):

| 파일 | 내용 |
|---|---|
| `audit_int_A.png` | real_route_path1 신호 교차로 @(929,−530), regelem 152150·152268·152290·152334 |
| `audit_int_B.png` | 신호 교차로 @(1550,−196), regelem 152172 (좌회전 lanelet 포함) |
| `audit_int_C.png` | 신호 교차로 @(933,192), regelem 152935 (경로가 좌회전) |
| `audit_junction21.png` | 맵 경계 junction 21 (road 568,1187,1190,1193,1196,1199,1202,1208,1211,1217,1218,1219) @(233,186) — 4 접근로 중 3개만 신호 |

육안 확인 결과: 정지선이 접근로 폭을 정확히 가로지르고 lanelet 끝에 놓임, 화살표가 전부 진입 방향,
turn_direction 배열이 안쪽=left → straight → 바깥=right 로 실제 노면 화살표 배치와 부합.

---

## ③ 결함 목록

### 🚨 D-1 [치명 · 맵 아님, 소비측] waypoint 를 방향 무시하고 최근접 lanelet 에 스냅하면 경로가 2.3배로 부푼다

`개발계획_0902.md §2` route_node 설계의 "각 점 heading = **그 지점 lanelet 방향**" 은 순환 정의이고,
실제로 실행하면 무너진다. 실측(`aud/a5.py`, `a6.py`, `a8.py`):

| 후보 선택 규칙 | lanelet | 길이 | 차로변경 |
|---|---|---|---|
| 최근접 1개(경로 없으면 다음 후보) | 281 | **11,106 m** | **31회** |
| 총 경로길이 최소가 되는 후보(DP) | 127 | **4,772 m** | **9회** |
| 참고: `lane_graph.build_route` | — | 4,927 m | 9회 |

원인: **다차로 양방향 도로에서 반대 방향 lanelet 이 항상 더 가깝다.** real_route_path1 실측 —

```
wp4 (1457,-549) 진입코스 30°:  최근접 62132 d=1.61m hdg=-112°  ← 역방향
                               정방향 60729 는 d=4.36m hdg=68°
wp5 (1453,-188) 진입코스 91°:  최근접 85429 d=1.42m hdg=-173°  ← 역방향
                               정방향 85587 은 d=1.47m hdg=7°
wp7 (1166,-490) 진입코스 162°: 최근접 73364 d=1.47m hdg=-52°   ← 역방향
```
역방향 lanelet 을 잡으면 라우팅은 실패하지 않고 **U턴 대신 블록을 한 바퀴 돈다**:
wp3→wp4 leg 이 직선 389m 인데 경로 **2924m**, wp4→wp5 leg 이 직선 360m 인데 **3407m**.
20분 제한에서 6.3km 초과 주행 = 미완주 확정.

**수정안**: route_node 의 waypoint heading 을 **CSV 인접점 방향**(`atan2(y[i+1]-y[i], x[i+1]-x[i])`,
마지막 점은 직전 방향)으로 정하고, 후보 lanelet 중 **중심선 방향과 ±90° 안**인 것만 채택.
그래도 후보가 2개 이상이면 `getRoute` 총 길이가 최소인 것을 고른다(DP, `aud/a8.py` 참조 — 14wp·10후보에 수 초).
`mission_planner` 에 `set_route_points` 로 넘길 때도 **각 점의 orientation 을 반드시 이 heading 으로 채울 것**
(빈 quaternion 을 넘기면 Autoware 내부 스냅이 같은 함정에 빠진다).

### 🔴 D-2 [높음] 횡단보도 197곳이 여전히 0개

`validate_semantics` 재실행: `crosswalk lanelet 0개 / crosswalk 규제요소 0개`.
xodr `Rm_Warning_Crosswalk_JPN_03.flt` **400개 → (road,s) 197곳** 이 그대로 남아 있다.
Autoware `crosswalk` 모듈이 대상 요소가 없어 동작하지 않음 → 항목 10(보행자, **중대 −6**)·12(횡단보도 정차, 경미)를
objects 기반 obstacle_stop 에만 의존. **9/4 이후 익스포터 작업 최우선.**

### 🔴 D-3 [높음] `speed_limit` 2480개 전부 "50 km/h"

`roadmark_speed_30` 노면표시가 있는 road 35개 위의 lanelet **186개도 50 km/h**.
30/50 표시 road 집합은 교집합 0으로 깨끗이 분리되므로, 최소한 이 186개는 `30 km/h` 로 태깅 가능하다.
(단 9/1 결론대로 "30 표시 = 보호구역" 은 아님 — 보호구역 확정 좌표는 osgb 추출 필요. 저속은 무벌점이므로
합집합 태깅이 실무적 절충.) 현재는 항목 2 대응이 `zone_speed_node` yaml 임시안에 100% 의존.

### 🟠 D-4 [중] lane_change 태그 섹션 중점 근사 — 32개 섹션 938m(2.08%) 오분류, **route_example 이 그 중 1곳을 지난다**

전수 측정(`aud/a14.py`): 인접 driving 쌍 1205개 중 공유 경계의 roadMark 레코드가 2개 이상인 것 **32개**,
그 32개 전부 섹션 안에서 type/laneChange 가 바뀐다. 중점 값으로 섹션 전체를 덮어 생기는 오분류 길이 **938m / 45,069m**.

경로 교차 확인:
- **route_example → road 429 sec5**: `[(0.0,'solid','none'), (25.0,'broken','both')]`, 섹션 길이 101.4m.
  중점이 broken 이므로 **앞 25m 의 실선 구간이 `lane_change=yes` 로 태깅됨** → 그 구간에서 Autoware 가
  차로변경을 시작하면 **항목 6 실선 차로변경(중대 −6)**.
- real_route_path1 → road 173 sec0 (42m), road 2096 sec0 (38.8m), road 2305 sec4 (15.0m).
  이 셋은 전부 `broken → none → broken` 패턴(교차로/진출입로 개구부)이라 반대 방향 오류
  (실제 개구부인데 dashed 로 남음 = 차로변경 허용) — 감점 위험은 낮음.

**수정안**: `export_lanelet2.py:121,147,150` 의 `mark_rec_at(mid_ds)` 를 버리고,
roadMark 레코드 경계에서 way(및 lanelet)를 s-분할한다. 32개 섹션만 해당하므로 비용은 작다.
당장은 최소한 **"섹션 안에 solid 레코드가 하나라도 있으면 그 섹션 전체를 `lane_change=no`"** 로 보수화하면
route_example 의 위험은 즉시 사라진다(오탐으로 차로변경 기회를 조금 잃을 뿐).

### 🟠 D-5 [중] 퇴화 lanelet — 폭 0 인 것 4개, 고립 3개, 최소폭 0.1m 미만 232개

```
완전 퇴화(전 구간 폭<0.10m): 87309=(2470,2,-4)  116659=(2810,2,2)  116758=(2810,5,1)  122179=(2816,2,1)
routing 완전 고립: 122179(1306,699), 116659(1188,545), 116758(1201,549)
xodr link 이 있는데 연결 소실: (2810,2,2)→(2810,1,2)
평균폭 <2m 186개 / <1m 56개, 최소폭 <0.10m 232개
```
원인: xodr width 다항식이 섹션 끝에서 0 으로 수렴하는 확폭·감폭 차로를 **폭 0 구간까지 그대로 lanelet 으로 출력**.
Lanelet2 로드는 통과하지만 좌우 경계가 겹치는 폴리곤이라 drivable area·`lane_departure_checker`·
`path_optimizer` 에서 0 나눗셈/뒤집힌 폴리곤이 나올 수 있다(**미확인 — 실기 관찰 필요**).
두 경로에는 포함되지 않으므로 사전테스트 위험은 없다.

**수정안**: 익스포터에서 `lane.width_at` 최대값이 0.5m 미만인 차선은 lanelet 생성 자체를 건너뛰고,
폭이 0 으로 수렴하는 끝단은 폭 0.5m 지점에서 잘라낸다.

### 🟠 D-6 [중] road 1927 (1063,−881) 강제 용접이 원본 기하를 1.65m 이동시킨다

3.30m 간극을 클러스터 중점으로 병합 → 양쪽 노드가 각각 1.65m 이동. 채점 세계(xodr)와 최대 1.65m 어긋난다.
경로에 포함되지 않지만, 당일 CSV 가 이 지점을 지나면 차로 폭(3.3m)의 절반이므로 **항목 3·4 위반 유발 가능**.
나머지 6곳은 ≤0.17m 로 무해. 대안은 "용접하지 않고 succ 관계만 태그로 표현"이지만 Lanelet2 라우팅이
점 공유로만 성립하므로 불가 → **당일 CSV 가 road 1927 을 지나는지 `check_route.py` 로 확인하는 것이 실무적 대응.**

### 🟡 D-7 [중저] 비신호 정지선 171곳 미출력 — 현 구조로는 출력해도 무효

xodr 정지선 object 는 (road,s) 기준 301곳인데 신호등 접근로 130곳만 ref_line 이 됐다.
다만 Autoware `stop_line` 모듈은 `traffic_sign` + `type=stop_sign` 규제요소만 소비하므로
(`autoware_behavior_velocity_stop_line_module/src/manager.cpp:45-64`),
`type=stop_line` way 를 더 만들어도 **아무 동작도 하지 않는다**. 정식화하려면 규제요소 종류를 바꿔야 한다.
현행 규정상 무신호 정지선 정지 의무는 채점 15항목에 없으므로 **우선순위 낮음**.

### 🟡 D-8 [중저] turn_direction 이 junction 연결 기하와 42/353 불일치

노면 화살표(334개)를 우선으로 삼고 junction 기하(19개)를 폴백으로 쓰는데, 둘 다 계산 가능한 347개 중
**42개가 불일치**한다(예: `(429,5,-3)` 화살표 left vs 기하 straight, `(1869,0,2)` 화살표 straight vs 기하 left).
화살표가 더 신뢰할 만하지만(겸용 차로 SLT/SRT 85개를 straight 로 접은 것이 불일치의 상당 부분),
**42개 전수의 옳고 그름은 미확인**. 브리지가 원형 녹색/적색만 발행하는 한 무해(§②-3 (b) 참조).
화살표 신호를 쓰기 시작하면 이 42개가 오정지·오통과 원인이 된다.

### 🟡 D-9 [중저] 같은 접근로에서 차로변경하면 traffic_light 모듈이 첫 lanelet 의 turn_direction 만 쓴다

`manager.cpp:206-212 hasSameTrafficLight` 는 refers way id 로 중복을 제거하므로, 경로에 같은 규제요소를
가진 lanelet 이 둘 이상 오면 **먼저 등장한 것만 모듈이 되고 뒤엣것은 `updateStopLine` 만 받는다**.
real_route_path1 에 4곳(152150, 152207, 152172, 151415). 브리지 원형-only 설계에서는 무해하지만,
`manager.cpp:148-164 getModuleExpiredFunction` 이 `scene_module` 인자를 `[[maybe_unused]]` 로 무시하고
경로 전체 단위로 만료를 판정하는 것과 겹치면 **리스폰·경로 갱신 후 오래된 신호등 모듈이 남을 수 있다**(미확인).

### 🟢 D-10 [양호 — 반증된 우려]

- 인접 way 공유 1205/1205, 방향 뒤집힘 0, 자기교차 0, id 중복 0, 로드 경고 0.
- 신호등 방향 판정: 독립 증거 247건(validity 50 + 정지선 197)과 **모순 0건**. B-2 는 완전히 해소됐다.
- z: `ele` 35.55~79.29, z==0 노드 0개, 용접 클러스터 내 z 차이 **최대 0.0000m**(신호등/정지선 way 를 용접 대상에서
  제외한 조치가 실제로 동작). 9/1 미검증 위험 ②(신호등 z 가 0 으로 끌려내려감)는 **발생하지 않음**.
- 정지선 위치: object 기준 전환으로 signal.s 대비 이동 median 0.000m, |Δ|>1m 는 1개뿐. 9/1 의 "이상치 32개, 최악 171.5m" 해소.
- 익스포터 재실행이 배포 맵과 **바이트 동일** — 재현성 확보.

---

## ④ 미확인 사항

1. **Autoware 자체 traffic rules 로 만든 routing graph 는 검증하지 못했다.** 우리 venv 에는
   `autoware_lanelet2_extension` 이 없어 `traffic_rules Locations.Germany` 로만 확인했다.
   `route_handler.cpp:324` 는 `lanelet::autoware::DefaultLocation("autoware")` 로도 그래프를 만든다.
   `location=urban` 태그와 "autoware" location 조합에서 `speedLimit()` 이 우리 `speed_limit` 태그를 읽는지
   **실기에서 `ros2 topic echo /planning/.../path` 의 목표속도로 확인 필요.**
2. **폭 0 퇴화 lanelet 232개가 Autoware 하위 모듈에서 실제로 문제를 일으키는지** 미확인
   (drivable area 생성, `lane_departure_checker`, `path_optimizer`). 두 경로에 없어 우선순위는 낮다.
3. **turn_direction 42개 불일치의 정답** 미확인. 노면 화살표 object 의 t 밴드 판정(`ARROW_MAX_DIST=60m`)이
   옆 차로 화살표를 잘못 집었을 가능성 vs junction 기하가 다중 후보인 경우, 어느 쪽인지 표본 검증 안 함.
4. **`light_bulbs` 부재의 실제 영향** 미확인. `validation.cpp:89-99` 는 경고만 내지만,
   `AutowareTrafficLight::lightBulbs()` 를 쓰는 planning 쪽 코드가 있는지 전수 grep 하지 않았다.
5. **`getModuleExpiredFunction` 의 경로 단위 만료**(D-9)가 리스폰 시 실제로 stale 모듈을 남기는지 미확인.
6. **정지선 증거 없는 controller 15개**(road 1928, 2077×2, 2195×2, 2115, 2116, 2575, 146×2 …)의 방향은
   road_end 휴리스틱 단독 판정이다. 이 중 road 146 은 양쪽에 정지선 object 가 2개씩 있어 증거가 상쇄된다.
   → 실기에서 이 접근로를 지날 때 정지 위치를 로깅해 확인.
7. **경계 way `road_border` 가 단 1개**인 점. 최외곽 주행 차로의 바깥 경계가 거의 전부 roadMark 를 가져
   `line_thin` 으로 나가는데, Autoware drivable area 계산이 이를 "넘어갈 수 있는 선"으로 보는지 미확인
   (항목 5 보도 침범). sidewalk lanelet 을 안 만들었으므로 방어선이 얇다.

---

## 최상위 결함 5개

1. **D-1** — 경로 CSV 점을 방향 무시하고 최근접 lanelet 에 스냅하면 real_route_path1 이 4,772m·차로변경 9회 → **11,106m·31회**로 부풀어 20분 완주가 불가능해진다. route_node 는 waypoint heading 을 CSV 인접점에서 계산해 ±90° 필터 후 총길이 최소 후보를 골라야 한다 (맵 결함 아님, 소비측 필수 수정).
2. **D-2** — 횡단보도 규제요소·lanelet 이 여전히 **0개** (xodr 에 197곳 재료가 있음) → crosswalk 모듈 무효, 항목 10(중대)·12 무방비.
3. **D-3** — `speed_limit` 이 2480개 전부 "50 km/h"; `roadmark_speed_30` road 위 lanelet 186개도 50 → 항목 2 보호구역을 맵으로 못 푼다.
4. **D-4** — lane_change 태그가 섹션 중점 근사라 32개 섹션 938m(2.08%)이 오분류되고, 그 중 **road 429 sec5 의 앞 25m 실선이 `lane_change=yes` 로 태깅된 채 route_example 경로 위에 있다** → 실선 차로변경(중대 −6) 위험.
5. **D-5/D-6** — 폭 0 퇴화 lanelet 4개(고립 3개)와 최소폭 0.1m 미만 232개가 그대로 남아 있고, road 1927 (1063,−881)의 3.30m 강제 용접이 원본 기하를 1.65m 이동시킨다 (둘 다 현재 두 경로 밖이지만 당일 CSV 가 지나면 즉시 위험).

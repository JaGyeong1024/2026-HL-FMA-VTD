# 담당영역 A 검토 보고서 — VTD↔Autoware 브리지 / 통신 프로토콜

작성 2026-09-02. 대상: `scratchpad/ctrl/repo/ros2_ws/src/vtd_autoware_bridge/`
검증 방법: (a) 코드 정독, (b) 실제 맵(`/home/a/HL_FMA/controller/out/livinglab_lanelet2_native.osm`, 33MB)으로 tl_router 직접 실행, (c) 제어기 PC `~/autoware/src` 원본 grep. 추측은 "미검증"으로 명시.

---

## ① 구조 요약

```
VTD(TCP 9910) ──1109B@20Hz──> rx_loop(전용 스레드)
                                └ publish_state(): Odometry/TF/Accel/Velocity/Steering/Gear/Mode/Turn/Hazard
                                  + publish_objects()  (PredictedObjects 30슬롯)
                                  + publish_traffic_light() (TrafficLightGroupArray, TlRouter 경유)
Autoware ── /control/command/control_cmd ──> on_control()  ─┐
         ── /control/command/turn_indicators_cmd ──> on_turn()┤ cmd_lock 보호 공유변수
         ── /planning/mission_planning/route ──> on_route() → 별도 스레드 → TlRouter.set_route()
                                                             tx_tick(20Hz 타이머) ──9B──> VTD
```

- 속도·각속도·가속은 **pose 차분 + 1차 저역통과(α=0.35)** 로 추정(패킷에 속도 없음 — 대회정보.md §6, 9/2 공식답변과 일치).
- 좌표계 변환 없음: VTD 월드 = Autoware `map`, ego 원점 = 후축 = `base_link`. 대회정보.md:82(`좌표 원점(ego 기준점) = 차량 뒷바퀴 축 중심(지면)`) 및 vehicle_info 패치(wheel_base 2.944 + front_overhang 0.864 = 3.808 = DistFront)와 **정합 확인. 문제 없음.**
- 조향 부호 steer_sign=+1.0 도 대회정보.md §6 실측 확정(+=좌=Autoware 규약)과 일치. **문제 없음.**
- 프로토콜 struct 포맷(`<6f`+`I8f`*30+`iB`=1109B / `<ffB`=9B), 리틀엔디안, id==0 필터, 1109B 재조립·최신패킷만 사용, NaN/Inf 필터, 조향 ±0.48 클램프, 가속 [-6,3] 클램프 — **스펙과 전부 일치. 문제 없음** (`protocol.py:6-10,32-35`, `bridge_node.py:48-49,174-178,359-368`).

즉 **바이트 레벨 하위 계층은 건전하다. 결함은 전부 상위 계층 — Autoware 토픽 계약과 신호등 경로다.**

---

## ② 확인된 결함

### A-1 [치명] 차량상태 토픽명이 Autoware 규약과 다름 → 트래젝토리 추종기가 영원히 대기, 차가 움직이지 않음

- 브리지: `bridge_node.py:96` `/vehicle/status/velocity_report`, `:97` `/vehicle/status/steering_report`
- Autoware 규약(제어기 PC 실원본): `/vehicle/status/velocity_status`, `/vehicle/status/steering_status`
  - `~/autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_control_launch/launch/control.launch.xml:166`
    `<remap from="~/input/current_steering" to="/vehicle/status/steering_status"/>` (trajectory_follower `controller_node_exe`)
  - 같은 파일 `:96` vehicle_cmd_gate `input/steering`, `:144` operation_mode_transition_manager `steering` — 셋 다 `steering_status`
  - `:252` autonomous_emergency_braking `~/input/velocity` → `/vehicle/status/velocity_status`
  - 전수 확인: autoware_launch + autoware_universe 전체에서 `/vehicle/status/` 토픽 출현 = steering_status 28, velocity_status 19, control_mode 11, actuation_status 6, turn_indicators_status 5, gear_status 5, hazard_lights_status 3. **`*_report` 이름은 단 한 건도 없다.**
- 파급: `~/autoware/src/universe/autoware_universe/control/autoware_trajectory_follower_node/src/controller_node.cpp:190`
  `is_ready &= getData(current_steering_ptr_, sub_steering_, "steering");` → 조향 상태 미수신 → `is_ready=false` → **`/control/trajectory_follower/control_cmd` 자체를 발행하지 않음** → gate도 발행 안 함 → 브리지 `on_control` 미호출 → cmd_steer/cmd_accel=0 유지 → VTD로 (0,0,0)만 전송 → **차량이 출발하지 않는다.**
  operation_mode_transition_manager도 steering 미수신 → `is_autonomous_available` 미성립 가능성 큼(engage 거부, todo0902 §5 A4 부채의 실제 원인일 확률 높음).
- **왜 psim에서 안 걸렸나**: psim은 `simple_planning_simulator`가 올바른 이름으로 직접 발행한다. 브리지는 psim 경로에 없다. todo0902 §1의 "psim E2E 통과"는 이 결함에 대해 아무 보증도 못 한다.
- 나머지 4개(`gear_status`, `control_mode`, `turn_indicators_status`, `hazard_lights_status`)는 이름이 맞다.

### A-2 [치명] TlRouter가 실제 맵에서 100% 예외 — 신호등 토픽이 단 한 번도 발행되지 않음

`tl_router.py:91`, `:103`의 `e.clear()` 들여쓰기 오류. `ET.iterparse`는 자식 원소(`<nd>`, `<tag>`)의 end 이벤트를 부모보다 먼저 내는데 두 루프는 **모든** 원소에 `clear()`를 호출한다. `Element.clear()`는 속성까지 지우므로 `<nd ref=...>`의 ref가 None이 된다.

실제 맵으로 직접 실행한 결과:
```
parse 2.0s, RSS 504 MB
tl groups 646  lanelets with tl 312  lanelet_ways 2480
set_route  EXC: TypeError int() argument must be ... not 'NoneType'   ← _load_centroids:88
next_group EXC: IndexError list index out of range                    ← centroids == []
```

파급 사슬:
1. `on_route` → `_apply_route`(`bridge_node.py:333-337`)가 TypeError를 잡아 `경로 신호등 구축 실패` 1줄만 로깅. 그런데 `set_route`(`tl_router.py:67-73`)는 예외 **전에** `route_ids`/`route_tls`를 이미 채워 놓는다.
2. 이후 `next_group`(`tl_router.py:120-121`)의 가드 `if not self.route_ids or not self.route_tls`를 통과 → `self.centroids[i]`에서 IndexError.
3. 이 예외는 `publish_traffic_light` → `publish_state` → `rx_loop`의 `except Exception`(`bridge_node.py:182-183`)이 삼켜 `상태 발행 오류`를 **20Hz로 스팸**. 신호등 그룹은 영영 발행되지 않는다.
4. `start_autoware.sh`가 `is_simulation:=true`를 주므로 Autoware는 신호 정보 없는 신호등을 **정지가 아니라 통과**로 처리한다:
   `~/autoware/src/universe/.../autoware_behavior_velocity_traffic_light_module/src/scene.cpp:203-205`
   ```cpp
   if (!traffic_signal_stamp_) { return !planner_data_->is_simulation; }   // is_simulation=true → false = "정지 안 함"
   ```
   → **모든 적색신호 무정차 통과.** 채점 항목 7 중대 위반(–6)이 교차로마다 누적.
5. 조용히 실패한다는 점이 최악. 로그를 안 보면 "잘 달리는데 신호만 무시"로 보인다.

부수 결함(같은 파일):
- `set_route`가 `route_ids`/`route_tls`를 먼저 쓰고 `centroids`는 2.7초 뒤 채움 → 위 버그를 고쳐도 경로 갱신 직후 **2.7초간 IndexError**. 지역변수로 만든 뒤 원자적 교체 필요(락 포함).
- TlRouter 상태를 route 스레드가 쓰고 rx 스레드가 읽는데 **락이 없다**(`bridge_node.py:331`).
- `_parse_relations`가 node/way를 clear하지 않아 RSS 504MB(실측). 동작은 하나 낭비.

### A-3 [치명] state 4(적+좌회전 화살표)는 우리 맵에서 "영구 정지"로 해석된다

Autoware 판정(`~/autoware/src/universe/autoware_universe/common/autoware_traffic_light_utils/src/traffic_light_utils.cpp:65-102`):
- 초록 **CIRCLE**이 있으면 통과.
- 없으면 `lanelet.attributeOr("turn_direction","else")`를 보고 `"left"`일 때만 GREEN+LEFT_ARROW로 통과. **`"else"`(속성 없음)이면 무조건 정지.**

실제 맵 실측:
```
신호등 달린 lanelet 312개의 turn_direction:  (none) 308 / straight 4
전체 lanelet turn_direction:  (none) 1773 / straight 394 / right 170 / left 143
```
→ 신호등 규제요소가 붙은 lanelet의 **98.7%에 turn_direction이 없다.** 따라서 `TL_STATE_MAP[4]`(`bridge_node.py:57`)의 RED CIRCLE + GREEN LEFT_ARROW는 **정지**로 해석된다. 좌회전 보호신호 구간에서 출발 못 하고 현시가 끝나면 다시 적색 → 사실상 교차로 영구 정지(미완주).
- state 5(녹+좌)는 GREEN CIRCLE 포함이라 통과된다. 4번만 깨진다.
- 근본 원인은 맵(담당 B)이지만 **브리지에서 값싸게 회피 가능**: GT state는 이미 "Ego 진행방향에 매핑된 제어기 상태"(대회정보.md §6 9/1 Q&A)이므로 방향 화살표로 재인코딩할 이유가 없다. `4 → GREEN CIRCLE` 매핑이면 Autoware의 방향 재해석을 통째로 우회. (단 ③-3 실측 전제 확인 필요.)

### A-4 [높음] state 6(점멸)이 Autoware에서 "영구 적색"이 된다

`isTrafficSignalStop`은 `element.status`(SOLID_ON/FLASHING)를 **전혀 보지 않는다** — color와 shape만 본다(위 파일 65-102행). `TL_STATE_MAP[6] = RED CIRCLE + FLASHING`(`bridge_node.py:59`)은 그냥 적색 → **점멸 교차로에서 무한 대기.**
대회정보.md §6·todo0902의 방침("state 6 = 정지선에서 0.5초 일시정지 후 통과")을 Autoware가 대신 해주지 않는다. 브리지가 상태기계를 가져야 한다: state 6 진입 → RED 유지 → ego 속도<0.1m/s가 0.5초 이상 → 해당 신호그룹에 한해 GREEN CIRCLE로 전환(교차로 통과까지 래치).

### A-5 [높음] 제어 명령 워치독 없음 — 마지막 명령 무한 반복 전송

`tx_tick`(`bridge_node.py:375-383`)은 `cmd_steer/cmd_accel`을 조건 없이 20Hz 재전송. Autoware 플래닝이 죽거나 멎으면 마지막 값(최악 `accel=+3.0`)이 **영원히** VTD로 나간다 → 제동 없이 가속 유지 → 코스 이탈 → 리스폰(–6)/사고.
`on_control`(`:359`)에 수신 시각을 기록하고 0.3~0.5초 경과 시 `(steer 유지, accel=-3.0)` 등 안전값으로 페일세이프해야 한다.

### A-6 [중간] tx_tick / rx_loop 소켓 경합 → 노드 사망 가능

`bridge_node.py:376`의 `if not self.connected or self.sock is None: return` 검사와 `:381` `self.sock.sendall(pkt)` 사이에 rx_loop(`:171`)이 `self.sock = None`으로 만들 수 있다. `AttributeError: 'NoneType' object has no attribute 'sendall'`는 `except OSError`(`:382`)로 **안 잡힌다**. 타이머 콜백에서 나간 예외는 `rclpy.spin`을 뚫고 `main` finally로 가 **노드가 죽는다**(`:394-400`). 재연결 순간에만 열리는 좁은 창이지만 재연결은 대회 중 실제로 일어난다. 소켓을 지역변수로 받아 쓰고 `except Exception`으로 넓힐 것.

관련: rx_loop 스레드가 예상 못한 예외로 죽으면 **아무도 모른다**(데몬 스레드, 로그 없음). 노드는 살아있고 데이터만 멎는다. `report_tick`이 `rx_count` 증가 여부를 감시해 경보해야 한다.

### A-7 [중간] 리스폰(좌표 점프) 방어 전무

대회정보.md:75-77 — 오프로드 0.3초/경로이탈 0.5초면 자동 리스폰, 구간 시작점으로 복귀(구간별 1회 초과 –6).

1. **속도 추정 폭주**: `publish_state`(`:191-205`)의 dt 게이트는 `1e-4 < dt < 0.5`뿐, **거리 점프 검사가 없다.** 5m 점프/0.05s = 100 m/s → α=0.35로 vx_f 즉시 35 m/s, ax_f ~700 m/s². 이 값이 `/localization/kinematic_state`, velocity, `/localization/acceleration`으로 그대로 나간다(가속은 behavior_velocity_planner의 **필수** 입력 — `~/autoware/src/core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/node.cpp:224`). 종방향 제어기는 최대 감속을 때린다. 회복에 0.3~0.5초. 리스폰 직후 급제동 → 재이탈 위험.
   → `hypot(dx,dy) > 1.5m` 등 임계 초과 시 **추정기 리셋**.
2. **TlRouter cur_idx 단조 증가**: `next_group`(`tl_router.py:123`)의 탐색창이 `cur_idx-2 ~ cur_idx+8`이라 뒤로 2 lanelet까지만 되돌아간다. 리스폰은 수십 lanelet 뒤로 보내므로 **cur_idx가 앞에 남아** 이미 지난 신호그룹 id를 계속 발행 → 실제 다음 신호등은 데이터 없음 → is_simulation=true로 **적색 통과**. 좌표 점프 감지 시 cur_idx 전역 재탐색 필요.
3. Autoware 플래닝 자체의 재정렬은 미검증. 위 둘은 확정 결함.

### A-8 [중간] steering 보고가 명령값 복사라 MPC 상태가 개루프

`bridge_node.py:247-251` `steer.steering_tire_angle = self.cmd_steer`. A-1을 고쳐 토픽명을 맞추면 이 값이 MPC의 조향 상태변수로 들어간다. MPC는 상태 [횡오차, 방위오차, 조향]에서 조향을 실측으로 보정하고 `input_delay`/조향률 제한을 보상하는데, 실측=지령이면 **조향 축 피드백이 사라진다.** 횡오차는 odom으로 들어오므로 발산하진 않겠으나 VTD 실제 조향 응답 지연만큼 위상 여유가 줄어 고속·급커브 진동 가능. 또 operation_mode_transition_manager의 지령-실측 조향 편차 검사가 무조건 통과되어 이상 상황을 못 거른다.
→ 최소: 1차 지연(τ≈0.2s) 필터 후 보고. 근본: RDB raw(48190)에서 실조향 취득(대회정보.md §6에 채널 존재).

### A-9 [중간] gear 명령 미처리 + goal_planner 후진 위험

브리지는 `/control/command/gear_cmd`를 구독하지 않고 `GearReport.DRIVE` 고정 발행(`:253-256`). CtrlPacket에 기어 필드가 없어 후진 표현 수단 자체가 없다. 그런데 todo0902 §1에 "goal_planner pull-over 후보 12개 생성"이 있어 goal_planner가 활성이다. 종료지점에서 pull-over/후진 기동이 나오면 Autoware는 후진 전제로 지령하는데 VTD는 전진 가속으로 해석 → 종료지점 부근 폭주. 완주 판정은 "후축이 종료 좌표 통과"(대회정보.md:196)이므로 **goal을 종료지점 너머로 잡고 pull-over를 끄는 것**이 안전. (완전 검증 못함 — ③-2)

### A-10 [낮음~중간] objects 변환의 근사

- **크기 휴리스틱 분류**(`:288-293`): `length<1.2 and width<1.2` → PEDESTRIAN. VTD가 콘·표지·소형 정적물을 objects로 준다면 전부 보행자가 되고, crosswalk 모듈은 보행자에 훨씬 보수적으로 정지 → 불필요 정지(항목 12 횡단보도 정차 금지와 충돌 가능). 실기 로깅 필요.
- **등속 직선 예측 8초, confidence 1.0**(`:303-316`): 교차로에서 회전 중 차량의 예측이 직진으로 나가 (a) 없는 충돌로 급제동, (b) 실제로 앞으로 꺾여 들어오는 차량은 예측 못함. `map_based_prediction`이 실기 모드에서 꺼져 있어(launch_perception:=false) 이 예측이 유일하다. 8초는 과함 — 3~4초로 줄이고 confidence를 낮추면 오탐 제동 감소.
- **id 안정성 미활용**: 9/2 답변대로 id는 유지되나 브리지는 프레임별 무상태 변환만 한다. 속도가 `speed` 스칼라뿐이라 가속·회전율·진행방향 변화를 못 본다. id로 이전 프레임 위치를 물어 yaw rate를 추정하면 예측 품질이 크게 오른다(권장).
- 30슬롯·80m 절단은 "가까운 순 정렬"이라 플래닝에 실질 손실 없음. **문제 없음.**
- `object_id`를 `struct.pack('<IIII', oid,0,0,0)`로 만든 것은 id 안정성을 그대로 계승 — 적절.

### A-11 [낮음] 소켓·프로토콜 운영 세부

- **TCP_NODELAY 미설정**. 주최측 예제는 설정한다(`TCP, UDP 통신 예제/tcp_manual_controller.py:55`). 직결 링크에선 통상 무해하나 9B 패킷이 Nagle에 걸리면 최대 40ms 지연. 한 줄이므로 넣을 것.
- **9910 단일 접속 대응 부족**: `settimeout(2.0)`(`:151`)이라 2초 무수신이면 소켓을 닫고 즉시 재접속. VTD가 이전 연결을 잡고 있으면 거부/지연 가능(미검증). 실패 시 2초 sleep → 최악 4초 공백. 그 사이 `tx_tick`은 전송하지 않고, 스펙상 VTD는 steering=0/accel=0 처리 → **제동 없이 타력주행.** 곡선에서 끊기면 이탈·리스폰. 백오프 0.2초로 단축 + 끊김 시각 명확 로깅.
- **연결 끊김 시 Autoware 거동**: 브리지는 아무것도 안 한다. odom/velocity가 멎으면 controller_node의 `getData` 실패로 제어 지령이 멎고 behavior_velocity_planner도 "Waiting for ..."로 정지 — Autoware는 안전하게 멎지만 VTD는 이미 0지령 타력주행. 최소한 큰 경고 로그(가능하면 diagnostics) 필요.
- `protocol.py`의 `VTDClient`(46-71행)는 브리지가 쓰지 않는다(rx_loop이 동일 로직 재구현). 이중 관리 부채.
- `bridge_node.py:22` `import xml.etree.ElementTree as ET` 미사용.
- `bridge.launch.xml`에 `ctrl_rate_hz` 미노출(코드엔 존재).

### A-12 [정보] 신호등 규제요소 다중 참조는 문제 없음(단, is_simulation 의존)

실측: 신호등 붙은 lanelet 312개 중 **1개짜리는 하나도 없고** 3개(92), 4개(6), 6개(209), 9개(4), 10개(1). 다만 한 lanelet에 붙은 규제요소들의 `ref_line`(정지선) 좌표는 **전부 동일**했다(예: lanelet 578의 6개 규제요소 모두 stopline (16.1289, 38.0846)). 한 정지선에 신호등 헤드가 여러 개인 구조다.
브리지는 `route_tls`의 첫 항목만 발행(`tl_router.py:134-136`)하므로 나머지 2~9개 모듈은 데이터 없음 → is_simulation=true 덕에 통과. **동작하지만 is_simulation에 100% 의존.** 해당 lanelet의 **모든** 그룹 id에 같은 state를 실으면 견고해진다(정지선이 같으므로 부작용 없음).
참고: lanelet 578/622/666은 정지선이 lanelet 끝에서 40m 떨어져 있다 — 규제요소가 긴 진입차로에 붙었을 가능성(담당 B 사안). 정지선이 경로 뒤쪽이면 traffic_light 모듈이 정지점을 삽입 못해 **조용히 통과**한다는 점은 브리지에도 영향.

---

## ③ 미검증 위험

1. **실기 모드 engage 가능 여부**(todo0902 §5 A4). `launch_localization:=false`라 `/localization/initialization_state` 발행 노드가 없다. `~/autoware/src/universe/.../compatibility/autoware_state.cpp:86`은 `localization_state != INITIALIZED`면 `/autoware/state`를 INITIALIZING에 묶는다. operation_mode 전환이 이를 요구하는지는 미확인. A-1이 겹쳐 있으므로 **A-1을 먼저 고친 뒤 재판정**. 브리지가 `/localization/initialization_state`를 INITIALIZED로 위조 발행해야 할 수도 있다.
2. **goal_planner 후진 기동 실제 발생 여부**(A-9). pull-over 비활성 파라미터 미확인.
3. **VTD state 4의 의미.** Q&A를 그대로 읽으면 state 4 = ego가 좌회전 가능. 다만 교차로 현시를 방향 무관하게 4로 보고할 가능성도 배제 못 함. A-3 회피안(4→GREEN CIRCLE)은 **직진 중 state 4 수신 사례가 없는지 실기 로깅 확인 후** 적용. 확인 전이면 "직진 lanelet이면 정지, 좌회전 경로면 통과"를 브리지가 경로로 직접 판단하는 편이 안전.
4. **VTD가 재접속을 즉시 받아주는지**(A-11). 시뮬 PC 부재로 확인 불가.
5. **objects의 실제 구성**(정적물 포함 여부, heading/speed 부호 규약) — 실기 로깅 필요.
6. `/perception/obstacle_segmentation/pointcloud` 부재의 영향. behavior_velocity_planner는 `required_subscriptions.no_ground_pointcloud`가 false면 통과(node.cpp:239-241)라 기본 구성엔 문제 없어 보이나 전수 확인 안 함.

---

## ④ 개선 제안 (우선순위)

### P0 — 오늘 안에, 사전테스트(9/3) 전 필수
1. **토픽명 2개 수정**(`bridge_node.py:96-97`): `velocity_report` → `velocity_status`, `steering_report` → `steering_status`. 이것 없이는 실기 모드에서 차가 한 발짝도 안 나간다.
2. **`tl_router.py:91,103` `e.clear()` 들여쓰기 수정** — `if` 블록 안으로(way/node일 때만 clear). 검증된 패치 결과: `set_route 2.7s / centroids None 0개 / next_group → 151066 정상`.
3. **회귀 스모크**(VTD 없이 오늘 가능): `mock_vtd.py`를 127.0.0.1:9910으로 띄우고 `./start_autoware.sh 127.0.0.1` → ①`/vehicle/status/steering_status` echo ②route 주입 후 `/perception/traffic_light_recognition/traffic_signals` 발행 ③`/control/command/control_cmd` 발행 확인. **주의: `mock_vtd.py`는 tl_state=0만 보내 A-2 경로를 타지 않는다.** 검증하려면 mock이 tl_state=1을 보내도록 임시 수정 필요.

### P1 — 본선 전 필수
4. **state 4 처리 결정**(A-3): 실기 로그로 ③-3 확인 → `4 → GREEN CIRCLE` 또는 경로 기반 좌회전 판정. 현 상태로는 좌회전 교차로 미완주.
5. **state 6 상태기계**(A-4): 정지 0.5초 래치 후 GREEN CIRCLE 전환. 대회정보.md 방침의 구현체가 어디에도 없다.
6. **제어 워치독**(A-5): 0.3s 무명령 → 감속 페일세이프.
7. **리스폰 점프 감지**(A-7): 변위 임계 초과 시 속도추정기 리셋 + `TlRouter.cur_idx` 전역 재탐색.
8. **tx_tick 소켓 경합 수정**(A-6) + rx 스레드 사망 감시.
9. **TlRouter 원자적 교체 + 락**, 대상 lanelet의 **모든** 신호그룹에 state 발행(A-12).

### P2 — 여유 있으면
10. steering 보고 1차 지연 모델(A-8), 또는 RDB 48190에서 실조향/실속도 취득.
11. 객체 예측 8초→3~4초, id 기반 yaw rate 추정, 보행자 분류 임계 재검토(A-10).
12. TCP_NODELAY, 재연결 백오프 단축, `VTDClient` 중복 제거, 미사용 import 정리(A-11).
13. `is_simulation:=true`가 "신호 데이터 실패 = 무조건 통과"를 뜻하므로, 브리지가 **신호등 발행 건수를 5초마다 로깅/경보**할 것. 지금 구조에선 신호 로직이 죽어도 주행은 계속되어 육안 판별이 불가능하다.

# 담당영역 C — Autoware 통합·기동·운영 준비도 + 규정 15항목 갭 분석

작성 2026-09-02. 근거는 전부 제어기 PC(`100.120.213.124`) 실소스 `~/autoware/src` 및 저장소 사본 직접 확인.

**결론 한 줄**: 실기 모드(`./start_autoware.sh`)는 현재 상태로 **차가 1mm도 움직이지 않는다**. psim 성공은 실기 모드와 구성이 근본적으로 다르므로 아무것도 보증하지 못한다. 최소 6개의 독립적 "발행자 부재" 결함이 있고 각각이 단독으로 치명적이다.

---

## ① 구조 요약

### 1.1 실기 모드가 끄는 것

`start_autoware.sh:47-58` → `autoware.launch.xml`에서 살아있는 group: vehicle(description만) / map / planning / control / api / rviz.
죽는 group: **system 전부**(mrm_handler, mrm_*_operator, diagnostic_graph_aggregator + availability converter, component_state_monitor, hazard_status_converter), localization 전부(pose_initializer 포함), perception, sensing, vehicle_interface.

### 1.2 psim이 실기와 다른 점 (검증 무효화 근거)

`~/autoware/src/launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml`은
- `launch_system`을 **끄지 않는다**(기본 true) + `system_run_mode:=planning_simulation` + `launch_system_monitor:=false` + `launch_dummy_diag_publisher:=true`
- `localization_sim_mode:=api` → 초기화 상태 공급자 존재
- `tier4_simulator_component` 추가 → `simple_planning_simulator`(차량상태 토픽 전량), `dummy_perception_publisher`(`/perception/obstacle_segmentation/pointcloud`), `probabilistic_occupancy_grid_map`(`/perception/occupancy_grid_map/map`, `tier4_simulator_launch/launch/simulator.launch.xml:117-125`)

→ **todo0902 §1 "psim 자율주행 E2E 통과"는 실기 모드에 대해 증거값 0**. psim이 통과한 이유가 정확히 실기 모드에서 빠진 컴포넌트들이다.

---

## ② 확인된 결함

### D-1 【치명·확실】 브리지 차량상태 토픽 이름 오류 — 궤적추종기 영구 대기

브리지: `bridge_node.py:96` → `/vehicle/status/velocity_report`, `:97` → `/vehicle/status/steering_report`.

Autoware 구독(`tier4_control_launch/launch/control.launch.xml`): `:86` vehicle_cmd_gate `input/steering` → **`/vehicle/status/steering_status`**, `:167` trajectory_follower `~/input/current_steering` → 동일, `:147` omtm `steering` → 동일, AEB `~/input/velocity` → **`/vehicle/status/velocity_status`**.

전수 grep 결과 `vehicle/status/velocity_report`·`vehicle/status/steering_report`는 **Autoware 소스 전체에 0건**. 표준 이름은 `core/autoware_simple_planning_simulator/.../simple_planning_simulator.launch.py:56-62`에서 확인.

파급: `autoware_trajectory_follower_node/src/controller_node.cpp:190` `is_ready &= getData(current_steering_ptr_, sub_steering_, "steering")` → 실패 → `/control/trajectory_follower/control_cmd` 미발행 → `/control/command/control_cmd` 미발행. 연쇄로 `autoware_operation_mode_transition_manager/src/state.cpp:216-219`가 `trajectory_follower_control_cmd`·`control_cmd`를 요구하므로 `is_autonomous_mode_available=false`.
gear_status / control_mode / turn_indicators_status / hazard_lights_status 4개는 이름이 맞다.

### D-2 【치명·확실】 `use_emergency_handling: true` + system 미기동 → vehicle_cmd_gate 영구 "waiting topics"

`autoware_launch/config/control/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml`: `use_emergency_handling: true`, `system_emergency_heartbeat_timeout: 0.5`.
`autoware_vehicle_cmd_gate/src/vehicle_cmd_gate.cpp:342-350` `isDataReady()`는 `emergency_state_heartbeat_received_time_` 없으면 false, 이 값은 `:818-824 onMrmState()`에서만 설정. 입력 `/system/fail_safe/mrm_state`(control.launch.xml:112)의 발행자 `autoware_mrm_handler`는 `tier4_system_launch/launch/system.launch.xml:152-156` — system 그룹이라 죽어 있다.
`:453-455` `if (!isDataReady()) { ... return; }` → **control_cmd 영구 미발행**. D-1을 고쳐도 이것만으로 차는 못 움직인다.

### D-3 【치명·확실】 ADAPI `change_to_autonomous`가 구조적으로 항상 거부 (부채 A4의 정체)

`autoware_default_adapi_universe/src/operation_mode.cpp`: `:42-50` `/system/operation_mode/availability` 구독, `:57-61` **초기값 `mode_available_[AUTONOMOUS] = false`**, `:68-75` `!mode_available_[mode]`면 `ERROR_NOT_AVAILABLE` 예외.
해당 토픽 발행자 = diagnostic_graph_aggregator의 availability converter(`system.launch.xml:130-144`) — system 그룹이라 죽어 있다.
→ `/api/operation_mode/change_to_autonomous`는 **100% 실패**. A4는 "미검증"이 아니라 **불가능**.

### D-4 【치명·확실】 pointcloud·occupancy grid 부재로 플래닝 체인 정지 → trajectory 자체가 없다

`default_preset.yaml` 기본값으로 켜진 모듈들의 필수 구독 선언:

| 모듈 | 파일 | 요구 |
|---|---|---|
| ObstacleStop | `core/.../autoware_motion_velocity_obstacle_stop_module/src/obstacle_stop_module.hpp` | `no_ground_pointcloud` |
| ObstacleSlowDown | `.../obstacle_slow_down_module.hpp` | `no_ground_pointcloud` |
| ObstacleVelocityLimiter | `.../obstacle_velocity_limiter_module.hpp` | `no_ground_pointcloud`, **`occupancy_grid_map`** |
| BVP crosswalk | `.../autoware_behavior_velocity_crosswalk_module/src/manager.hpp:49-51` | `occupancy_grid_map` |
| BVP intersection | `.../experimental/manager.hpp:39-41` | `occupancy_grid_map` |
| BVP detection_area | `.../autoware_behavior_velocity_detection_area_module/src/manager.hpp:42-43` | `no_ground_pointcloud` |

게이팅: `core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner/src/node.cpp:144-197`(`check_with_log(..., required_subscriptions.no_ground_pointcloud)` → `is_ready=false`), `:306-307` `if (!update_planner_data(...)) return;`. 동일 패턴 `behavior_velocity_planner/src/node.cpp:229-244`.
리맵: `motion_planning.launch.xml:186` → `/perception/obstacle_segmentation/pointcloud`(`tier4_planning_component.launch.xml:14`), `:189` → `/perception/occupancy_grid_map/map`.
실기 모드에서 두 토픽 발행자 **없음** → **`/planning/trajectory` 미발행**.

### D-5 【중대·확실】 AutowareState 영원히 `Initializing`

`autoware_default_adapi_universe/src/compatibility/autoware_state.cpp:75-88`: component state 미수신 → INITIALIZING, `localization_state != INITIALIZED` → INITIALIZING.
`/localization/initialization_state` 발행자 = pose_initializer(localization launch) 또는 psim의 `localization_sim_mode:=api`. 실기엔 둘 다 없다.
파급: AEB `check_autoware_state:=true`(control component 기본) → **AEB 무력화**(규정 14), shift_decider 기어 명령 이상, RViz engage 버튼 무의미.

### D-6 【중대·확실】 신호등 state가 엉뚱한 regelem 하나에만 실림 → 적색 통과 가능성 높음

맵(`~/2026-HL-FMA-VTD/map/lanelet2_map.osm` → `livinglab_lanelet2_native.osm`) 실측:
- traffic_light regelem 646, lanelet 2480
- **신호등 보유 lanelet 312개, lanelet당 신호등 수 분포 = {3:92, 4:6, 6:209, 9:4, 10:1}. 1개인 lanelet은 0개** (교차로 전 방향 신호기가 각 진입 lanelet에 통째로 붙음)

`tl_router.py:70-74`는 문서 순서 **첫 그룹만** `route_tls`에 넣고 `next_group()`(`:118-137`)이 id 하나만 반환. Autoware는 regelem마다 scene module을 띄우고 **자기 `ref_line`이 ego 경로와 교차할 때만** 정지점을 삽입한다. state를 준 그룹의 stop line이 우리 방향이 아니면 그 모듈은 무동작, 정작 우리 stop line 모듈은 데이터가 없어 `traffic_light_module/src/scene.cpp:186 return !planner_data_->is_simulation` = **is_simulation:=true라 통과** 처리.
→ **적색에 그대로 진입**. `is_simulation:=true`는 D-6이 있는 한 안전장치가 아니라 위험 증폭기다.

### D-7 【중대·확실】 좌회전 화살표(state 4)에서 영구 정지 — turn_direction과 신호등이 다른 lanelet에 있음

`common/autoware_traffic_light_utils/src/traffic_light_utils.cpp:66-105`: 녹색 CIRCLE이 아니면 `turn_direction = lanelet.attributeOr("turn_direction","else")`, **`"else"`면 무조건 정지(`:79-81`)**. 좌회전 화살표 통과는 `turn_direction=="left"`일 때만(`:89-95`).
맵 실측: turn_direction 총 707개(left 143 / right 170 / straight 394)인데 **신호등 보유 312 lanelet 중 turn_direction 보유는 4개(전부 straight)뿐**.
→ state 4/5의 화살표를 줘도 `"else"`로 빠져 정지. 좌회전 교차로 무한 대기 → 규정 8 중대 –6 + 사실상 미완주.

### D-8 【중대·확실】 스쿨존 30km/h 수단이 맵에 없음

맵 speed_limit 태그 실측: `v="50 km/h"` **2480건 = 전 lanelet 동일, 예외 0**. max_vel 13.3 m/s(47.9km/h)가 보호구역에도 그대로 적용 → 규정 2 "5km/h 초과=중대" → **–6 확정**(1km/h 초과부터 감점이라 회피 불가).

### D-9 【중대·확실】 횡단보도 lanelet 0개 → 규정 10·12 담당 모듈 미활성

맵 실측: `k="subtype" v="crosswalk"` **0건**, walkway도 0건. 모듈이 true여도 등록 대상이 없어 한 번도 동작하지 않는다. `autoware_통합_검토_0901.md` §4의 "crosswalk 모듈 / 기본 동작" 매핑은 사실과 다르다.

### D-10 【중대·개연】 리스폰 시 속도 추정 폭주 + TlRouter 고장

`bridge_node.py:191-205`: 가드가 `1e-4 < dt < 0.5`뿐, **위치 점프 검사 없음**. 리스폰 수십 m 순간이동 → v 수백 m/s, α=0.35 저역통과라 수 프레임 오염, ax_f는 더 큼.
파급: vehicle_cmd_gate `filter_.setCurrentSpeed()`(`:709-718`) 왜곡, velocity_smoother 초기속도 오염, omtm `enable_engage_on_driving:false`라 **재-engage 시 `|v|>0.01`로 거부**.
동시에 `tl_router.py:122-124` 탐색창이 `cur_idx-2 ~ cur_idx+8`이라 **2 lanelet 이상 뒤로 리스폰되면 인덱스가 영구 고착**.

### D-11 【경미~중대】 신호등 정지 마진 0m

`config/planning/.../behavior_velocity_planner/traffic_light.param.yaml`: `stop_margin: 0.0`. Autoware는 범퍼(base_link + 0.864 + 2.944 = **3.808m**, 패치값 일치)를 정지선에 맞춘다 → 제어 오차가 전방이면 **정지선 초과 정차 = 중대 –6**. 규정은 2m 이내까지 정상이므로 0.8~1.2 여유가 순수 이득. `stop_line.param.yaml`도 `stop_margin: 0.0`.

### D-12 【중대】 경로 주입 스크립트 부재

`~/hlfma/route/`엔 `route_config.yaml` + 데모 CSV 2개뿐, `~/2026-HL-FMA-VTD/tools/`에 주입 스크립트 없음. 참고는 `tools/psim_e2e.sh:36`의 단발 `set_route_points`(goal 1개, waypoints 없음)뿐.
제약(소스 확인): `mission_planner.param.yaml` `goal_angle_threshold_deg: 45.0`, `check_footprint_inside_lanes: true`, `enable_correct_goal_pose: false` → goal heading이 차선 방향 ±45° 밖이면 거부(psim에서 0.509rad 거부/1.704rad 성공으로 실증). `default_planner.cpp:281 is_goal_valid`는 goal에만 적용, waypoint 매칭 규칙은 별도.
**추가 위험**: psim E2E는 `allow_goal_modification: true`로 호출해 goal_planner pull-over 후보 12개를 만들었다(todo0902 §1). 종료 판정은 "후축이 종료 좌표 통과"인데 pull-over는 종료점 **앞에서 갓길 정지**할 수 있다 → 미완주. 종료점은 `allow_goal_modification:=false` + 종료 좌표보다 20~30m 앞에 goal을 둘 것.

### D-13 【운영·중대】 psim 스크립트가 없어질 디렉토리에 출력

`tools/psim_e2e.sh:4` / `psim_smoke.sh:3`: `OUT=/tmp/claude-1000/-home-a-autoware/ddb88ad1-.../scratchpad`. `/tmp`이므로 **재부팅 시 소멸** → todo0902 §6-2의 "아침에 psim_e2e.sh부터"가 전 리다이렉트 실패로 무의미해진다. `sleep 150` 후 판정이라 실패도 늦게 안다.

### D-14 【운영】 자동 로그인 미설정 / ROS_DOMAIN_ID 불일치

`/etc/gdm3/custom.conf`의 `AutomaticLoginEnable`은 **여전히 주석 처리** → 콜드부팅 후 수동 로그인 필요(20분 시계가 로그인 화면에서 돈다).
`~/.bashrc:125`의 `ROS_DOMAIN_ID=43`은 인터랙티브 셸 전용이고 `start_autoware.sh`는 이를 **설정하지 않는다**(psim 스크립트는 명시). 런처·systemd·비대화 ssh 실행 시 도메인 0으로 떠 그래프가 어긋난다.

### D-15 【운영】 기동 시간 예산

`psim_e2e.sh:20`이 노드 안정까지 `sleep 150` + daemon 8초(127노드). 실기는 브리지·rviz·VTD가 더해진다. 개략: 부팅+로그인 1.5~2분 → Autoware 2.5~3분 → 경로 주입·확인 1~2분 → 3km 주행 ≈6분 = 13~14분. **재시도 1회 여유가 거의 없다.** 본선용 `rviz:=false` 변형을 준비할 것.

---

## ③ 규정 15항목 × 컴포넌트 갭 표

| # | 항목 | 담당 | 현재 상태 | 근거 | 리스크 |
|---|---|---|---|---|---|
| 1 | 제한속도 | velocity_smoother + `common.param.yaml max_vel 13.3` | 구현·미검증 | `patches/autoware_launch_수정.patch` | 47.9km/h. 평가는 VTD RDB 실속도 → 12.8m/s 권고 |
| 2 | 보호구역 30 | 맵 speed_limit | **미구현→감점 확정** | speed_limit 2480건 전부 50km/h | **–6 확정**(D-8) |
| 3 | 차로 유지 | behavior_path + MPC + lane_departure_checker | 미검증 | control.launch.xml:190 | steering_report=명령값 근사(C10) |
| 4 | 중앙선 침범 | 맵 lane_change 태그 + routing graph | 구현 | lane_change no 1803 / yes 1082, solid 1787 / dashed 1082 | 태그 s-분할 근사 |
| 5 | 보도 침범 | drivable area + boundary_departure_prevention | 미검증 | preset true | — |
| 6 | 실선 차로변경 | lane_change 모듈 | 구현·미검증 | `.../lane_change_module/src/utils/utils.cpp:1387 attributeOr("lane_change")` | 태그 경계 정확도 |
| 7 | 적색신호 정지 | BVP traffic_light + TlRouter | **결함** | D-6, D-11 | 적색 통과(중대) / 정지선 초과(중대) |
| 8 | 녹색신호 통과 | 〃 | **결함** | D-7 | 화살표에서 영구 정지 → 중대+미완주 |
| 9 | 적색점멸 일시정지 | 〃 | **미구현** | `traffic_light_utils.cpp:66-105`가 `status`(FLASHING)를 **전혀 안 봄**; `bridge_node.py:59`가 FLASHING을 보내도 solid red 취급 | 점멸에서 **영구 정지** |
| 10 | 보행자 대응 | crosswalk 모듈 | **미구현** | D-9 | 횡단 완료 대기 로직 없음, 중대 |
| 11 | 장애물 대응 | MVP obstacle_stop | 미검증(현재 D-4로 정지) | 모듈 required subscription | pointcloud 해결 후 재평가 |
| 12 | 횡단보도 정차 금지 | crosswalk/stuck | **미구현** | D-9 | 경미 |
| 13 | 지시등 3초 | behavior_path → `/control/command/turn_indicators_cmd` → `bridge_node.py:370-373` | 구현·미검증 | `behavior_path_planner.param.yaml:30 turn_signal_search_time: 3.0` | 정확히 3.0초 경계값 → 4.0 권고 |
| 14 | 도로 이용자 충돌 | AEB + intersection + collision_detector | **약화** | D-5(AEB 무력화), `common.param.yaml limit.min_acc -2.5` | 최대 감속 –2.5m/s²만 사용(패킷은 –6 가능) |
| 15 | 리스폰 | 플래닝이 pose 점프 흡수 | **결함** | D-10 | 속도 추정 폭주 + TlRouter 고착 |

---

## ④ 미검증 위험

1. **TL regelem ↔ 진행방향 stop line 대응**: 6개 중 어느 `ref_line`이 우리 경로와 교차하는지 정적 확인 안 함. 문서 순서 첫 번째가 맞을 확률은 1/3~1/10.
2. **waypoint의 교차로 lanelet 매칭**: 직진/좌회전이 겹치는 구간에서 `DefaultPlanner`가 어느 lanelet에 붙이는지 미검증.
3. **리스폰 후 route 유지**: `allow_reroute_in_autonomous_mode: true`, `reroute_time_threshold: 10.0`, `minimum_reroute_length: 30.0`. 구간 시작점 복귀는 기존 route 집합 안이라 재계획 없이 복귀할 것으로 보이나 실증 없음.
4. **is_simulation:=true를 읽는 다른 노드**(부채 B6) 전수조사 미완. D-6이 있는 한 이 플래그는 위험 쪽으로 작용.
5. **OGM CUDA 커널**: D-4 해결에 OGM을 쓰기로 하면 "실기 미사용 경로라 리스크 없음"(todo0902 §5)의 근거가 무효화.
6. **acados 런타임**: `motion_planning.launch.xml:76`이 `/opt/acados/lib:$(env LD_LIBRARY_PATH)`를 컨테이너 env로 준다. `~/acados/lib`는 상속 뒷부분에 남아 psim에서 동작했으나 `start_autoware.sh` 밖(런처·systemd)에서 기동하면 깨진다.

---

## ⑤ 개선 제안 (우선순위)

### P0 — 실기 모드를 "psim과 구조적으로 동일"하게 (D-1~D-5 일괄 해소)

**5-A. 브리지 토픽 이름 수정** (`bridge_node.py:96,97`): `..._report` → `..._status`. 2줄.

**5-B. `start_autoware.sh` 인자 교체** — system을 켜되 psim과 같은 모드로:
```
launch_system:=true
system_run_mode:=planning_simulation      # component_state_monitor 요구 토픽을 psim 집합으로
launch_system_monitor:=false              # ★필수: autoware_system_monitor는 COLCON_IGNORE(목록 79행) → true면 launch 크래시
launch_dummy_diag_publisher:=true
```
이것으로 `/system/fail_safe/mrm_state`(D-2), `/system/operation_mode/availability`(D-3), `/system/emergency/control_cmd`가 정식 노드에서 공급된다.
`config/system/component_state_monitor/topics.yaml`의 planning_simulation 모드 요구 토픽 전수 검토 결과 — `/map/vector_map`, `/perception/object_recognition/objects`, `/planning/mission_planning/route`, `/planning/trajectory`, `/control/trajectory_follower/control_cmd`, `/control/command/control_cmd`, `/vehicle/status/velocity_status`, `/vehicle/status/steering_status`, `/tf`, `/system/emergency/control_cmd` — **5-A·5-C 적용 시 전부 충족**.

**5-C. 브리지 추가 발행 3종**
- `/perception/obstacle_segmentation/pointcloud` — **빈** `sensor_msgs/PointCloud2`(frame `base_link`) 10Hz. (VTD LiDAR 원본을 넣으면 지면 포인트로 obstacle_stop 영구 정지 → 금지)
- `/perception/occupancy_grid_map/map` — 빈 `nav_msgs/OccupancyGrid`(전부 free) 10Hz. 대안: 위 빈 pointcloud를 입력으로 이미 빌드된 `probabilistic_occupancy_grid_map` 별도 기동.
- `/localization/initialization_state` — `INITIALIZED`, transient_local, 1Hz (D-5).

**통과 판정 최소 조건**: `ros2 topic hz /control/command/control_cmd`가 돌고, `/api/operation_mode/state`의 `is_autonomous_mode_available: true`.

### P1 — 신호등 (D-6·D-7·규정 9)

- **5-D.** `tl_router.py:70-74`에서 첫 그룹만 넣지 말고 `self.lanelet_tls[lid]` **전체**를 넣고, `next_group()` → `next_groups()`(list)로 바꿔 `bridge_node.py:339-355`가 그룹 여러 개를 담아 발행. 우리 stop line과 교차하는 모듈만 실제 동작하므로 부작용 없이 "맞는 신호등 포함"이 보장된다.
- **5-E.** 좌회전 화살표: (i) 익스포터가 신호등 부착 lanelet에 `turn_direction`도 부여(근본 해결) 또는 (ii) 임시 — state 4/5일 때 브리지가 `GREEN/CIRCLE`도 함께 넣기(`traffic_light_utils.cpp:71-75`가 녹색 CIRCLE을 보면 즉시 통과, 좌회전 신호 진행은 합법). **시간 없으면 (ii)**.
- **5-F.** state 6: Autoware가 FLASHING을 무시하므로 브리지에서 상태기계로 — 처음 RED, 속도 0이 0.5초 지속되면 그 정지선을 벗어날 때까지 GREEN 전환.

### P2 — 맵

- **5-G. 스쿨존**: 붉은 노면 대응 xodr 요소 → 해당 lanelet `speed_limit` 30km/h. 못 찾으면 브리지가 위치 기반 `/planning/scenario_planning/max_velocity`(VelocityLimit) 발행/해제 — 좌표는 사람이 yaml에 적는 구조로.
- **5-H. 횡단보도**: 익스포터 crosswalk 출력 보강(현재 0건). 우선순위는 P0·P1 이하.

### P3 — 파라미터

`traffic_light.stop_margin 0.0→1.0`, `stop_line.stop_margin 0.0→1.0`, `turn_signal_search_time 3.0→4.0`, `max_vel 13.3→12.8`, `limit.min_acc -2.5→-4.0` 검토(패킷 한계 –6 이내).

### P4 — 리스폰 내성

- `bridge_node.py:191-205`에 점프 리젝트: `hypot(dx,dy) > 2.0`이면 `prev=None; vx_f=wz_f=ax_f=prev_vx=0.0`.
- `tl_router.py:122-124`: 최근접 거리 > 30m면 **전 경로 전역 재탐색** 폴백.

### P5 — 운영

- **경로 주입** 신규: `route_config.yaml` → 각 점 heading을 **그 지점 lanelet 방향**으로(이전 점 방향 아님) → `set_route_points(goal=마지막점보다 25m 앞, waypoints=중간점, allow_goal_modification=false)` → **응답 status 반드시 출력**(adaptor는 실패를 로깅 안 함, todo0902 §3). 설정파일+독립 실행 단계 구조 유지.
- psim 스크립트 `OUT`을 `~/hlfma/logs/` 등 영속 위치로 + `mkdir -p`.
- gdm3 자동 로그인 활성화 + 콜드부팅→`control_cmd` 발행까지 **실측 리허설 1회**.
- `start_autoware.sh`에 `export ROS_DOMAIN_ID=43` 명시, 본선용 `rviz:=false` 변형 준비.

### ⑥ 검증 순서 (시뮬 PC 켠 뒤 최단 경로)

1. 브리지만 → `ros2 topic hz /vehicle/status/steering_status` (5-A)
2. 전체 기동 → `ros2 topic hz /planning/trajectory` (D-4 해소)
3. `/api/operation_mode/state`의 `is_autonomous_mode_available: true` (D-2·D-3 해소)
4. 경로 주입 → `change_to_autonomous` → `/control/command/control_cmd` hz
5. 교차로 접근 시 신호등 그룹 id와 RViz 정지선 마커 대조 (D-6)
6. 리스폰 유발 후 `kinematic_state` twist 스파이크 관찰 (D-10)

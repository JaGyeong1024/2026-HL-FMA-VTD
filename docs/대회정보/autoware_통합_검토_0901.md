# Autoware 통합 검토 — 범위·코드·규정 매핑 (2026-09-01)

> 목적: 내일(9/2) 출근 후 "스크립트 실행 → 시뮬 연동 → 인지·제어 확인"만 하면 되도록,
> 오늘 결정·구현한 전체를 코드 단위로 검토하고 남기는 문서.
> 위치: 노트북 `/home/a/HL_FMA/` + 제어기 PC `~/2026-HL-FMA-VTD/docs/대회정보/`

---

## 1. 전체 아키텍처

```
┌────────────┐ TCP 9910 (1109B@20Hz / 9B)  ┌──────────────────┐   ROS 2 토픽    ┌─────────────────────┐
│  VTD (시뮬/ │◄───────────────────────────►│ vtd_autoware_    │◄──────────────►│  Autoware 1.9.0      │
│  대회장 PC) │  192.168.50.11              │ bridge (rclpy)   │                │  (플래닝+컨트롤만)     │
└────────────┘                             └──────────────────┘                └─────────────────────┘
                                             측위·인지·차량IF를                    맵: livinglab_lanelet2
                                             토픽으로 "대체 공급"                   _native.osm (local proj)
```

- **Autoware가 하는 일**: 맵 로드 → 경로(mission) → behavior/motion 플래닝 → 궤적 추종(MPC) → control_cmd
- **브리지가 하는 일**: VTD GT 데이터를 Autoware가 기대하는 측위·인지·차량상태 토픽으로 변환, Autoware 제어 출력을 9B 패킷으로 변환
- **꺼놓은 Autoware 서브시스템**: perception(NN), sensing(드라이버), localization(NDT/EKF), vehicle_interface, system 모니터 → `start_autoware.sh`의 launch 인자로 비활성

## 2. 빌드 범위 결정 (트림 빌드)

- 전체 488패키지 중 **플래닝·컨트롤·맵·API·시뮬레이터 + 모든 behavior/motion 모듈**을 최상위 타깃(약 50개)으로 지정, `colcon --packages-up-to`가 의존성 폐쇄를 자동 계산 (약 300개 예상)
- **제외**: perception NN(TensorRT), sensing 드라이버, NDT/EKF, CUDA 계열 — 빌드 의존이 아예 없는 것만. 런타임엔 브리지가 대체
- launch-only 패키지(autoware_launch, tier4_*_launch, sample_*_launch)는 `--packages-select`로 별도 등록 (의존성 폐쇄 없이 파일만 설치)
- **빠진 패키지 발견 시**: `cd ~/autoware && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to <패키지명>` — 증분이라 몇 분이면 됨. 트림 빌드 산출물은 전체 빌드로 전환해도 그대로 재사용됨
- 빌드 설정: `--parallel-workers 4` + `MAKEFLAGS=-j4` (32GB RAM OOM 방지). 로그: `~/autoware/full-build.log`, 완료 마커 `BUILD-ALL-DONE`

## 3. 브리지 코드 단위 검토 (`ros2_ws/src/vtd_autoware_bridge/`)

### 3.1 수신 경로 (RX: VTD → Autoware)

| 코드 | 발행 토픽 | 대체하는 Autoware 컴포넌트 | 비고 |
|---|---|---|---|
| `rx_loop()` | — | — | 1109B 재조립, 밀리면 최신만, 끊기면 2초 간격 재접속 |
| `publish_state()` ego | `/localization/kinematic_state`(Odometry), `/tf` map→base_link, `/localization/acceleration` | EKF localizer | VTD 월드좌표 = map 프레임 (local projector, 변환 0). **속도·각속도·가속은 pose 미분+저역(α=0.35) 추정** — GT에 ego 속도 없음 |
| 〃 | `/vehicle/status/velocity_report·steering_report·gear_status·control_mode(AUTONOMOUS)·turn_indicators·hazard_lights` | 차량 인터페이스 | **steering_report = 마지막 명령값** (실측 아님 — MPC 성능에 문제 시 개선 필요, §6) |
| `publish_objects()` | `/perception/object_recognition/objects`(PredictedObjects) | 인지+예측 스택 전체 | 크기 휴리스틱 분류(<1.2m 보행자 / <2.8m 이륜 / 그 외 차량), **등속 직진 8초 예측경로**(0.5s 간격). uuid=VTD id |
| `publish_traffic_light()` + `tl_router.py` | `/perception/traffic_light_recognition/traffic_signals` | 신호등 인식 | §3.3 |

### 3.2 송신 경로 (TX: Autoware → VTD)

- 구독: `/control/command/control_cmd` (vehicle_cmd_gate 통과한 최종값) → steering_tire_angle(rad)·acceleration(m/s²)
- 구독: `/control/command/turn_indicators_cmd` → 0/1/2 (Autoware가 차로변경·회전 시 자동 점등 → 규정 13번 자동 충족)
- `tx_tick()` 20Hz: `steer_sign(+1.0, 실측 확정) × steering`, 조향 ±0.48 클립, 가속 [-6,+3] 클립, NaN 가드
- VTD 9910은 **동시 접속 1개만** — 주행 중 다른 툴 접속 금지 (실측)

### 3.3 신호등 재설계 (`tl_router.py`) — 9/1 Q&A 반영

- 배경: trafficLightId ≠ 맵 signal id (공식). state만 유효
- 설계: `/planning/mission_planning/route`(LaneletRoute, transient_local) 구독 → 경로 lanelet 순서 확보 → OSM에서 lanelet→traffic_light regulatory element 매핑(시작 시 1회 파싱) → **ego 위치로 경로상 진행도를 추적해 "다음 신호등 그룹"에 state 적용**
- 경로 수신 시 해당 lanelet들의 centroid만 골라 파싱(3-pass iterparse, 수 초) — 별도 스레드로 20Hz 루프 비블로킹
- ⚠ **미검증**: 내일 실기에서 확인할 것 — ①경로 설정 후 로그에 "경로상 신호등 N개" 찍히는지 ②교차로 접근 시 RViz에서 해당 신호등이 적/녹으로 바뀌는지 ③멈춰야 할 정지선에 실제로 멈추는지

## 4. 대회 규정 15항목 → Autoware 매핑

| # | 항목 | 담당 | 오늘 반영 | 내일/추후 튜닝 |
|---|---|---|---|---|
| 1 | 제한속도 (초과 1km/h 허용) | velocity_smoother + common.param | **max_vel 4.17→13.3 m/s (≈48km/h, 50 대비 –2 마진)** | 맵 speed_limit 태그(50) 연동 확인 |
| 2 | 보호구역 속도 | 맵 speed_limit | — (붉은 노면 좌표 미확보, Q&A 대기) | 확보 후 익스포터에서 해당 lanelet 30km/h |
| 3 | 차로 유지 | behavior_path + MPC | 기본 동작 | 횡오차 모니터링 |
| 4 | 중앙선 침범 | 맵 lane_change 태그 + planner | 익스포터가 중앙선 yellow·차로변경 가능성 태그 반영 | 실주행 확인 |
| 5 | 보도 침범 | drivable area | 기본 동작 | — |
| 6 | 실선 차로변경 | lane_change 모듈 + 맵 태그 | 맵 태그 완비 | lane_change 모듈이 태그 준수하는지 확인 |
| 7 | 적색신호 정지 (범퍼 기준 2m 이내) | behavior_velocity traffic_light 모듈 | TlRouter (§3.3) | stop_margin 튜닝: 범퍼가 정지선 0.5~1.5m 전 정지하도록 |
| 8 | 녹색신호 통과 (불필요 정차 금지) | 〃 | 〃 | 과보수 정지(데드락) 모니터링 |
| 9 | 적색점멸 일시정지 | 〃 (state 6) | FLASHING RED로 전달 | Autoware가 점멸을 일시정지 후 통과로 다루는지 확인 — 아니면 커스텀 |
| 10 | 보행자 대응 (횡단 완료까지) | crosswalk 모듈 | objects로 보행자 공급 | crosswalk 파라미터(통과 판단 시점) 확인 |
| 11 | 장애물 대응 | motion_velocity obstacle_stop/slow_down + static avoidance | objects로 공급 | **장애물이 objects에 실리는지 실기 확인(v6/v7)** — 안 실리면 LiDAR 필요 |
| 12 | 횡단보도 위 정차 금지 | crosswalk/stuck 방지 | 기본 동작 | 익스포터 횡단보도 출력 보강과 연계 |
| 13 | 차로변경 지시등 3초 | behavior_path → turn_indicators_cmd | 브리지가 그대로 전달 | 점등 타이밍이 3초 전인지 실측 |
| 14 | 도로 이용자 충돌 (과실 무관) | obstacle_stop + AEB + intersection 모듈 | AEB·intersection 빌드 포함 | **교차 차량(신호무시) 대응은 intersection 모듈 담당 — 동작 확인 필수** |
| 15 | 리스폰 | — (플래닝이 급격한 pose 점프 겪음) | 브리지는 GT 그대로 전달 | 리스폰 후 Autoware가 경로 재추종하는지 확인 — route 재설정 필요할 수도 (핵심 리스크 §6) |

## 5. 오늘 수정한 파라미터·파일

| 파일 | 변경 | 근거 |
|---|---|---|
| `sample_vehicle_description/config/vehicle_info.param.yaml` (launcher·core 2곳) | wheel_base 2.944, front_overhang 0.864, rear 1.04, tread 1.63, height 1.507, **max_steer 0.48**, wheel_radius 0.354 | 아이오닉6 실제원 (대회정보.md §4). 폭 1.886 = 1.63+2×0.128 정확히 일치 |
| `autoware_launch/config/.../common.param.yaml` | max_vel 4.17 → **13.3 m/s** | 도심 50km/h, +1km/h 허용 대비 –2km/h 마진 |
| 브리지 | steer_sign +1.0 / host 192.168.50.11 / TlRouter | 실측·Q&A·대회장 IP |

주의: vehicle_info를 sample_vehicle에 **덮어썼음** (별도 차량 패키지 안 만들고). upstream 업데이트 시 유실 가능 — 기술부채로 기록.

## 5B. 필드 수준 호환 검증 (9/2 00시 — 소비 코드 직접 확인)

토픽 이름·타입을 넘어, 소비하는 쪽 소스에서 실제로 읽는 필드를 확인한 결과:

| 소비자 (확인한 코드) | 기대하는 것 | 브리지 충족 |
|---|---|---|
| traffic_light 모듈 (`scene.cpp findValidTrafficSignal`) | lanelet의 regulatory element id로 TrafficLightGroupArray 조회 | ✅ TlRouter가 같은 OSM의 relation id를 group_id로 발행 (map_loader와 동일 소스) |
| 〃 (`isStopSignal`) | **데이터 없는 신호등: 실환경=정지, 시뮬=통과** | ⚠→✅ **is_simulation:=true로 수정** (우리는 "다음 신호등"에만 데이터를 주므로 필수. 안 고쳤으면 뒤쪽 신호등마다 영구 정지였음) |
| 〃 (`isTrafficSignalTimedOut`) | tl_state_timeout(≈1s) 내 갱신 | ✅ 20Hz 발행 |
| crosswalk 모듈 (`scene_crosswalk.cpp`) | `classification.front().label`(비어있으면 안 됨), `kinematics.initial_twist...linear`(객체 속도), `kinematics.predicted_paths` 순회 | ✅ label 1개 append, twist.linear.x=speed, 등속 8초 경로 1개 |
| obstacle_stop 모듈 (`resample_highest_confidence_predicted_paths`) | predicted_paths + **confidence**, label 필터, **shape.dimensions.z로 높이 게이팅** | ✅ confidence=1.0, z=height(기본 1.6) 설정, 객체·ego 모두 월드 z 전달 |
| 플래닝 공통 | kinematic_state의 pose(map)+twist(base_link), acceleration | ✅ 부호 있는 속도(후진 감지 포함)+yaw rate |

남은 필드 수준 우려: **objects uuid의 프레임 간 안정성** — obstacle 모듈들이 uuid로 이력 추적을 하므로 VTD id가 흔들리면 추적이 끊김 (Q&A 답변 대기, 미답변 4번 항목).

## 6. 리스크 / 미검증 (우선순위순)

1. **엔드투엔드 미검증** — Autoware 노드가 브리지 토픽으로 실제 궤적을 내는 건 내일이 처음
2. **리스폰 대응** — pose가 순간이동하면 플래닝·컨트롤이 어떻게 반응하는지 미지. 경로 재설정(route 재주입) 로직이 필요할 가능성 높음
3. **TlRouter 미검증** (§3.3)
4. **launch_system:=false의 부작용** — ADAPI engage(operation mode 전환)가 system 노드 없이 되는지. 안 되면 `launch_system:=true`로 켜고 dummy diag 조정
5. steering_report=명령값 근사가 MPC에 미치는 영향
6. 경로 주입 방법 — 내일은 RViz 2D Goal Pose로 수동. 대회용은 route CSV→`/planning/mission_planning/route` 자동 주입 스크립트 필요 (다음 작업)
7. state 6(점멸)의 적/황 구분 불가 (Q&A 답변 대기)

## 7. 내일(9/2) 테스트 절차

```bash
# 0) 빌드 결과 확인 (밤새 빌드)
tail -5 ~/autoware/full-build.log     # "BUILD-ALL-DONE" 확인
# 브리지 재빌드가 안 돼 있으면:
source /opt/ros/jazzy/setup.bash && source ~/autoware/install/setup.bash
cd ~/2026-HL-FMA-VTD/ros2_ws && colcon build --symlink-install

# 1) VTD 없이 맵·플래닝 확인 (5분)
cd ~/2026-HL-FMA-VTD && ./start_autoware.sh psim
#    RViz에서 맵 보이는지 → 2D Pose Estimate로 초기위치 → 2D Goal Pose → 궤적 나오는지

# 2) 실기 연동 (시뮬 PC에서 VTD 먼저: run.sh license → sim_start --setup=00_HL_VTD --autoConfig,
#    시나리오 로드는 tools/scp_ctrl.py 또는 GUI)
cd ~/2026-HL-FMA-VTD && ./start_autoware.sh
#    확인 순서:
#    a. /tmp/bridge.log — "VTD 연결됨", TlRouter 로드
#    b. RViz에서 ego가 맵 위 올바른 위치에 (측위 OK)
#    c. objects 마커 보임 (인지 OK)
#    d. Goal 찍고 경로 생성 → 로그 "경로상 신호등 N개"
#    e. RViz AutowareStatePanel에서 engage (또는:
#       ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode {})
#    f. 차가 움직이고 VTD 화면(후방 10m 부감 카메라)에서 주행 확인 (제어 OK)

# 문제 시: 누락 패키지 → §2 증분 빌드 / launch 인자 → start_autoware.sh 수정
```

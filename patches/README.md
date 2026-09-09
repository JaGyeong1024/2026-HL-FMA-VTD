# patches — gitignore 된 업스트림(autoware) 소스에 대한 우리 수정

`hlfma_ws/src/autoware/` 는 .gitignore 대상(업스트림은 `hlfma.repos` 로 vcs import 재현).
따라서 그 안의 우리 수정은 여기 .patch 로 보존한다. **클린 재import 후 반드시 재적용.**

## route_handler_getMainLanelets_guard.patch
- 대상: `hlfma_ws/src/autoware/core/autoware_core/planning/autoware_route_handler/src/route_handler.cpp`
- 내용: `RouteHandler::getMainLanelets` 바깥 while 루프에 **방문 front-lanelet 가드** 추가.
  자동생성 lanelet2 맵의 좌/우 평행차선 위상 순환 시 무한 append(→ set_route 타임아웃/hang)를 끊는다.
  (근본은 맵 위상 결함 회피. gdb 로 무한루프 지점 확정 후 적용. 2026-09-03)
- 적용:
  ```
  cd ~/2026-HL-FMA-VTD
  patch -p0 < patches/route_handler_getMainLanelets_guard.patch
  colcon build --packages-select autoware_route_handler
  ```

## acados (MPC) 런타임 — 시스템 등록 (env 핵 아님)
path_optimizer 의 acados 솔버가 /opt/acados/lib 를 런타임에 찾도록 시스템 링커에 등록:
```
echo /opt/acados/lib | sudo tee /etc/ld.so.conf.d/acados.conf
sudo ldconfig
```
→ start_autonomous.sh 는 LD_LIBRARY_PATH env 를 쓰지 않는다(제거됨).


## external_request_lane_change_non_preferred.patch

- 대상: `autoware_behavior_path_lane_change_module`의 `get_target_neighbor_lanes`.
- 재현: 자차가 우선 차로가 아닌 상태에서 외부 요청 우회를 검토하면 기존 함수는
  자차 차로를 제외한다. `is_lanes_available()`이 false가 되어 후보가 생성되지 않는다.
  2026-09-06 주행에서 전방 정지차 약 10.9m, 우측 후보 0점과
  `lane_change.EXTERNAL_REQUEST: lanes are not available` 경고를 관측했다.
- 변경: `EXTERNAL_REQUEST`에 한해 현재 차로 열을 출발 차로 후보로 유지한다.
  목표 인접 차로는 기존 라우팅 그래프로 선택하고 경로 유효성·충돌 검사를 거친다.
  일반 차선변경과 회피 차선변경의 우선 차로 조건은 기존과 같다.
- 회귀 테스트: 기존 테스트 맵의 우선 차로가 아닌 자차 차로가 외부 요청의
  출발 차로 목록에 포함되는지 확인한다.
- 저장소 루트에서 `patch -p1 < patches/external_request_lane_change_non_preferred.patch`
  적용 후 `hlfma_ws`에서 `colcon build --packages-select autoware_behavior_path_lane_change_module`.
- 우측 우회 후 좌회전까지의 시뮬레이터 통과 여부는 별도 주행 검증 대상이다.


## lane_change_dist_buffer_degeneracy.patch

- 대상: `autoware_behavior_path_lane_change_module` 의 `calculation.cpp`
  (`calc_shift_intervals`, `calc_distance_buffer`)
- 증상: 정체 우회 상황에서 차선변경 후보가 하나도 생성되지 않는다.
  DEBUG 로그에 `Skip: prepare length out of expected range. length: 0.0,
  threshold min: -32.25, max: -1.797e+308` 이 초당 수천 건 찍힌다.
- 원인:
  1. `calc_shift_intervals` 가 `current_lanes.back()` **하나만** 조회한다.
     먼 구간의 preferred 가 진행 방향과 반대쪽이면
     `getLateralIntervalsToPreferredLane` 이 빈 배열을 반환한다.
  2. 그 빈 배열이 `calc_distance_buffer` 에서 `DBL_MAX` 로 바뀌고,
     `scene.cpp` 의 `dist_to_terminal_start = dist_to_terminal_end - DBL_MAX`
     가 `-DBL_MAX` 로 퇴화한다.
  3. `max_length_threshold = -DBL_MAX` 이므로
     `prepare_length > max_length_threshold` 가 항상 참 → 모든 후보 폐기.
- 변경:
  1. `calc_shift_intervals` 는 먼 쪽부터 자차 쪽으로 내려오며 첫 유효값을 사용한다.
  2. `calc_distance_buffer` 는 빈 배열에서 `DBL_MAX` 대신 `0.0` 을 반환한다
     (빈 배열 = 차선변경 불필요 = 필요 여유 0).
- 성격: 안전 완화가 아니라 **퇴화값 방어**다. 실선·RSS·obstacle_stop 등
  실제 안전 게이트는 그대로다.
- 적용: 저장소 루트에서
  `patch -p1 < patches/lane_change_dist_buffer_degeneracy.patch` 후
  `cd hlfma_ws && colcon build --packages-select autoware_behavior_path_lane_change_module`
- 검증 상태: 2026-09-08 적용, 주행 검증 진행 중.


## lane_change_yaw_threshold.patch

- 대상: `autoware_behavior_path_lane_change_module` 의 `utils/path.cpp`
- 증상: 차선변경 경로가 앞 정지차를 스치고 지나가 회피가 되지 않는다.
  전이 구간이 75 m 로 길어 자차가 통과하는 27 m 구간에서 횡변위가 0.2 m 밖에 안 된다.
- 원인: `constexpr auto yaw_diff_th = deg2rad(5.0);` (path.cpp)
  준비 구간과 전이 구간의 진행 방향 차이가 5도를 넘으면 후보를 폐기한다.
  3.0 m 차로를 5도 이내로 건너려면 직선 34 m, S자 약 69 m 가 필요하다.
  속도와 무관한 고정 상수라 저속 회피에서 지나치게 보수적이다.
  차량 실제 한계는 `max_steer_angle 0.48 rad`(27.5도, 회전반경 5.65 m)로 여유가 크다.
- 변경: 고정값을 **속도 비례**로 교체했다.
    R_geom = wheel_base / tan(max_steer_angle)        (저속: 조향각 한계, 5.65 m)
    R_dyn  = v^2 / a_lat_max                          (고속: 횡가속도 한계)
    yaw_th = clamp(input_path_interval / max(R_geom, R_dyn), 1도, interval/R_geom)
  속도별 상한: 2.6 m/s 이하 20.3도 / 5.2 m/s 5.1도 / 8.8 m/s 1.8도 / 11.5 m/s 1.0도
  원래 고정 5도는 약 5.2 m/s 기준값이었다.
- 검증: `Excessive yaw difference` 거부 0건. 다른 파라미터(lat_acc 하한 상향)와 함께
  실주행 횡변위 0.21 m → 1.17 m 로 개선 확인 (2026-09-08).
- 남은 과제: 속도 비례로 바꾸는 것이 정석이다.
  저속에서 20도, 고속에서 5도로 보간하면 안전 논리를 유지하며 필요한 곳만 완화된다.
- 적용: 저장소 루트에서
  `patch -p1 < patches/lane_change_yaw_threshold.patch` 후
  `cd hlfma_ws && colcon build --packages-select autoware_behavior_path_lane_change_module`

## avoidance_never_target_defer_to_ambiguous.patch

- 대상: `autoware_behavior_path_static_obstacle_avoidance_module` 의 `isNeverAvoidanceTarget`.
- 재현: 2026-09-09 시나리오 2, 자차로에 정차한 차량 2대(횡편차 -0.05 / -0.38 m)가
  `IS_NOT_PARKING_OBJECT` 로 무조건 제외되어 회피 대상이 0건. 자차 (291.8, -6.2) 영구 교착.
- 원인: `object.is_on_ego_lane` 블록이 "객체 차로 옆이 road_shoulder 가 아니면 절대 회피 금지" 를
  하드 리턴한다. 파라미터가 없고, Autoware 자신의 `avoidance_for_ambiguous_vehicle` 정책
  (주차인지 단순 정차인지 애매한 차량 처리)에 도달조차 못 한다.
  맵에 road_shoulder 를 넣어도 다음 줄의 `is_disjoint_right_lane` 에서 다시 걸린다
  (객체가 그 갓길 차로를 실제로 물고 있어야 하는데, 차로 정중앙에 서 있으므로 겹치지 않음).
- 변경: 정지 시간 > `th_stopped_time` 이고 이동 거리 < `th_moving_distance` 이면 하드 리턴을
  건너뛰고 ambiguous 정책이 판단하도록 넘긴다. 방향/시나리오 상수 없음.
  `isCloseToStopFactor` 게이트에도 같은 조건을 적용했다.
  `policy_ambiguous_vehicle: "ignore"` 로 두면 기존 거동과 완전히 동일 → 롤백은 파라미터 한 줄.
- 짝이 되는 파라미터: `avoidance_for_ambiguous_vehicle.policy: manual -> auto`,
  `condition.th_stopped_time: 3.0 -> 0.5`.
  (0.5 인 이유: `object.stop_time` 이 1.2~2.8 s 부근에서 동결되는 현상 관측. 원인 미규명 — TODO)

## avoidance_direction_by_available_space.patch

- 대상: 같은 모듈의 `StaticObstacleAvoidanceModule::createObjectData`.
- 재현: 위 패치로 회피 대상 등록에는 성공했으나 `necessity: true` 인 채
  `INSUFFICIENT_DRIVABLE_SPACE` 로 회피 포기. 여전히 교착.
- 원인: 회피 방향이 경로 대비 횡편차의 **부호만으로** 결정된다.
  ```
  object_data.direction = calc_lateral_deviation(...) > 0.0 ? LEFT : RIGHT;
  ```
  차단 차량은 차로 정중앙(-0.05 / -0.38 m)이라 부호가 음수 → RIGHT(객체가 우측)
  → 자차는 **좌측으로** 회피 → `getRoadShoulderDistance` 가 좌측 경계만 측정.
  실측: 좌측 여유 0.50~1.07 m / 우측 여유 3.52~7.27 m (필요값 2.386 m).
  모듈이 보고한 `to_drivable_bound` 0.83 / 1.04 가 좌측 실측값과 일치해 확증.
- 변경: 편차가 `threshold_distance_object_is_on_center`(1.0 m) 미만이면
  주행가능 경계까지 여유가 넓은 쪽으로 회피 방향을 정한다.
  갓길에 붙어 선 차량(편차 >= 임계값)은 기존 거동 유지.
- 검증: 2026-09-09. `obstacle_stop.lateral_margin` 정상값 0.2 로 6대 봉쇄 통과.
  (290.5,-2.7) v=5.91 -> (298.0,-24.6) v=7.06, 무정차. 이전에는 마진 0.05 에서만 통과했다.
- 남은 문제: 통과 후 (306.2, -42.1) 에서 재정지. 이 시점 객체는 전부 OUT_OF_TARGET_AREA 로
  회피와 무관 — 좌회전 차로 진입 문제로 추정.

## 2026-09-09 후속: 모듈 슬롯 굶주림과 공간 기반 회피 방향

avoidance_direction_by_available_space.patch 를 거리 기준에서 **공간 기준**으로 다시 작성했다.
기존: 편차가 threshold_distance_object_is_on_center(1.0m) 미만일 때만 개입.
문제: route preferred 가 좌회전 차로라 참조 경로가 좌1 을 따라가고, 자차로의 차단 차량이
      경로 기준 2.3~6.4m 떨어져 보여 개입 조건이 성립하지 않았다.
변경: 부호로 정해진 쪽의 경계 여유가 getAvoidMargin() 최소 요구치에 못 미치고
      반대쪽은 충족하면 방향을 뒤집는다. 요구치는 기존 파라미터로만 계산한다
      (lateral_hard_margin + 0.5W + hard_drivable_bound_margin + 0.5W = 2.386m).

### scene_module_manager.param.yaml (핵심)

planner_manager.cpp:517 getRequestModules 에 이 조건이 있다.
    exclusive_module_exist_in_approved_pool = any(!isSimultaneousExecutableAsApprovedModule)
    if (is_this_not_joinable) continue;   // isExecutionRequested 조차 호출되지 않음
승인 풀에 배타 모듈이 하나라도 있으면 **다른 모든 모듈이 평가에서 통째로 제외**된다.
blocked_route_detour 가 외부 차선변경을 상시 요청하는데 그 모듈이 as_approved:false 라,
승인되는 순간 static_obstacle_avoidance 가 영구 굶주림에 빠졌다.
계측: createObjectData 호출이 67회에서 멈춤 -> 수정 후 3079회.

  external_request_lane_change_left/right, avoidance_by_lane_change
      enable_simultaneous_execution_as_approved_module: false -> true
  static_obstacle_avoidance
      enable_simultaneous_execution_as_candidate_module: false -> true

### 검증 (1회, 반복 확인 필요)

obstacle_stop.lateral_margin 을 **원래값 0.3** 으로 되돌린 상태에서 6대 봉쇄를 무정차 통과.
  (289.0,  2.2) v=6.96
  (292.7,-11.5) v=6.73   <- 기존 교착 지점 (291.7,-6.3) 을 감속 없이 통과
  (306.8,-37.2) v=2.88
  (309.3,-41.1) v=0      <- 차선 복귀 후 정지
MRM 없음, VTD 수신 정상, 자력 정지. 회피 방향 판정은 전 객체 LEFT(=우측회피).

미해결: (309.3,-41.1) 에서 좌회전 차로로 진입하지 못한다.
       DETOUR(외부 우측 차선변경)는 여전히 85회 전부 prepare_samples=0 으로 거부된다.

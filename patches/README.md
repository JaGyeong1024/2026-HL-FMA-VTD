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

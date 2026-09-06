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

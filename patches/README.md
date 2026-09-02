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

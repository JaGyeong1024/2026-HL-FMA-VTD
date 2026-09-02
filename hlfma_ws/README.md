# hlfma_ws — HL FMA 단일 ROS 2 워크스페이스

```
hlfma_ws/
  build_targets.txt      빌드 최상위 타깃 (launch 패키지 + 런타임 플러그인). 의존성 폐쇄는 colcon 이 계산
  colcon_ignore_list.txt 빌드하지 않는 패키지 목록 (src/autoware 기준 경로) — 재구성 시 아래 한 줄로 마커 복원
  hlfma.repos            업스트림 Autoware 1.9.0 저장소 목록 (재현: vcs import src/autoware < hlfma.repos)
  src/autoware/          업스트림 소스 (git 추적 안 함)
  src/hlfma/             우리 패키지 — autoware_launch(설정·런치 복사본, 업스트림 것을 가림), hlfma_vehicle_launch(아이오닉6), vtd_autoware_bridge(어댑터)
```

## 일상 사용
```bash
cd ~/2026-HL-FMA-VTD/hlfma_ws
colcon build                                  # 전체 (증분)
colcon build --packages-select vtd_autoware_bridge autoware_launch   # 우리 패키지만 (수 초)
```
빌드 후 `source install/setup.bash` (새 터미널은 .bashrc 가 한다). colcon 기본 인자(symlink-install, Release, 4워커)는 ~/.colcon/defaults.yaml, make 병렬도는 .bashrc 의 MAKEFLAGS. acados 는 /opt/acados (path_optimizer CMake 기본 경로, launch 가 /opt/acados/lib 를 자동 추가).

## 처음부터 재구성
```bash
cd ~/2026-HL-FMA-VTD/hlfma_ws
vcs import src/autoware < hlfma.repos
grep -v "^#" colcon_ignore_list.txt | xargs -I{} touch src/autoware/{}/COLCON_IGNORE
colcon build
```

## 주의
- 업스트림 패키지를 고쳐야 하면 src/autoware 를 건드리지 말고 src/hlfma/ 로 복사한 뒤 업스트림 쪽에 COLCON_IGNORE (autoware_launch 가 그 예).
- colcon_ignore_list.txt 는 "이 PC에서 빌드 불가(CUDA·TensorRT 등)"인 패키지만 담는 것이 원칙. 필요한 패키지를 골라 빌드하는 용도로 쓰지 말 것.

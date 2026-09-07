# tools/harness — 수정 단위별 E2E 하네스 (시뮬 PC 없이, mock VTD 기반)

구성 (원커맨드 마법 없음: 설정 파일 + 독립 단계)
- `lib.sh`      공통 단계: 로그 디렉터리, mock 기동, 스택 기동(start_autonomous.sh mock), 준비 대기, engage, 캡처, 종료
- `place.py`    경로 CSV 기준으로 "시작점 전방 D m·횡 L m" 객체를 절대 좌표(mock `--obj-abs`)로 변환. 첫 정지선 거리 계산
- `metrics.py`  mock trace CSV(스텝 단위)에서 정지거리·최대감속·재출발 시간·이동객체 최소거리·lanelet 열(차선변경) 계산
- `cases/*.conf` 케이스 설정 (bash 로 source). ROUTE / MOCK_ARGS / RUN_SEC / 기대치
- `h0_startup.sh` `h1_stop_restart.sh` `h2_detour.sh` `h3_lane_change.sh` `hA13_respawn.sh` 수정 단위별 실행기

실행 (제어기 PC, 이 클론 루트에서)
    bash tools/harness/h1_stop_restart.sh                 # cases/h1_*.conf 전부
    bash tools/harness/h1_stop_restart.sh h1_car_ahead    # 하나만
    HARNESS_DOMAIN=53 …                                   # 다른 클론 스택이 43 으로 떠 있으면 도메인 분리 (기본 53)

산출: `~/hlfma/logs/harness/<케이스>_<시각>/` — mock.log, trace.csv, autoware.log, bridge.log, ready.txt, engage.txt,
       bag/ (선택 토픽), metrics.json, result.txt(PASS/FAIL 근거)

주의
- 포트 9910·ROS 도메인은 한 번에 한 케이스. 케이스당 스택 기동 ~90 s + 주행 RUN_SEC.
- 판정 기준은 각 conf 의 EXPECT_* 로 표현. 수정 전 baseline 은 BASELINE_0907.md.
- 이동객체/스케줄 시각은 `--clock move` 로 "ego 첫 이동 후 경과 초" 기준 (engage 시점 편차 무관).

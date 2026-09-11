# 실행 순서 (제어기 PC)

## 기동 + 출발 — 한 번에 (터미널 1)

    cd ~/2026-HL-FMA-VTD-JG
    ./start.sh

브리지 + Autoware 기동 → `route_config.yaml` 의 CSV 주입 → 경로 SET 대기 →
자율주행 가능 대기 → `change_to_autonomous` 서비스콜까지 자동으로 한다.

    ./start.sh mock      # 시뮬 PC 없이 (별도 터미널에서 python3 mock_vtd.py 먼저)
    ./start.sh psim      # Autoware 내장 planning_simulator (브리지 없음)
    ./start.sh <host>    # 다른 VTD 호스트

주요 환경변수

    ENGAGE=false ./start.sh      기동만 하고 출발은 수동
    LANE_PLAN=true ./start.sh    판단 노드(lane_planner) 기동 — 정지차 회피에 필요
    ROUTE_CSV=/path/route.csv    경로 지정 (기본: route_config.yaml 의 csv_path)
    ROUTE_CSV=none               경로 주입 안 함 (rviz 2D Goal Pose 수동)

로그

    ~/hlfma/logs/bridge_<시각>.log     브리지·route_node·lane_planner
    ~/hlfma/logs/autoware_<시각>.log   Autoware
    ~/hlfma/logs/engage_<시각>.log     출발 대기·서비스콜 결과

## rviz (선택, 터미널 2 — 제어기 화면에서, ssh 불가)

    ROS_DOMAIN_ID=43 ./rviz.sh

## 종료

    Ctrl+C          start.sh 가 브리지·Autoware 를 함께 정리한다
    ./stop.sh       정리가 미진할 때

## 계측 (주행 기록)

    python3 tools/trace.py ~/hlfma/logs/trace_$(date +%m%d_%H%M%S).jsonl 200

판단·제어·상태가 바뀔 때만 JSONL 한 줄씩 남긴다(한 판 수 MB).
담기는 것: routing/operation_mode/MRM, 루트 preferred, **모듈 승인·후보 풀**,
전 모듈 planning_factors(정지 사유와 거리), cooperate_status(safe 플래그),
궤적 길이·최소속도, 근접 객체, 제어 명령, 방향지시등.

    # lane_change 내부 판단까지 보려면 (요구 판정·후보 거절 사유·안전검사)
    lane_change.param.yaml 의 verbose: true  → 로그가 lane_change.NORMAL.left/.right 로 나뉜다

## 하네스 (VTD 없이 검증, 1회 약 5분)

    bash tools/harness/h3_lane_change.sh h3_route_lc    # 정상 경로 차선변경
    bash tools/harness/h1_stop_restart.sh               # 정지·재출발
    LANE_PLAN=true bash tools/harness/h4_pocket.sh h4_auto   # 정지차 회피

자세한 내용은 `tools/harness/README.md`.

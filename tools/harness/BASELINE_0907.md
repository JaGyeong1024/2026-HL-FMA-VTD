# BASELINE_0907 — 수정 전 기준 측정 (JG 클론 develop-jg 070f213, 설정 = 사전테스트 27fd15e)

환경: 제어기 PC 단독, mock VTD(폐루프 자전거 모델, 데드밴드 없음), ROS_DOMAIN_ID=53(NG 클론 잔존 스택이 43), 경로 route_pretest_2.csv,
Ego 시작 (239.8,146.0) lanelet 16249. 로그: `~/hlfma/logs/harness/<케이스>_<시각>/` (trace.csv, bag/, metrics.json, lc.json, result.txt).
재현: `bash tools/harness/h1_stop_restart.sh` 등 (README.md). 판정값 정의는 metrics.py 주석.

## 1. 케이스별 결과

| 단위 | 케이스 | 설계 기대(수정 후) | 수정 전 실측 | 판정 | 로그 |
|---|---|---|---|---|---|
| 0 기동 | h0_startup | 결함 주입 시 중단 | CSV 없음: exit 1 ✔ / **VTD 미연결인데 Autoware 기동 강행** / 중복 기동 차단 ✔(자기 디렉터리 기준) / 정리 훅: INT 로는 잔존 25(90 s 후에도) → **TERM 이면 3 s 내 0** | 2 FAIL | h0_startup_0907_145619 |
| 1 정지 | h1_car_ahead | 정지차 뒤 2~8 m 정지 | 간격 **4.74 m** 정지, 통과 없음, 감속 실측 **−1.11 m/s²**(=normal.min_acc −1.0), 정지 유지 명령 −1.5 | PASS | h1_car_ahead_0907_143042 |
| 1 정지 | h1_red_green | 적신호 정지 → 녹색 10 s 내 재출발 | 정지선 앞 정지 ✔, 녹색 후 **1.9 s** 재출발, 접근 감속 −1.14 | PASS | h1_red_green_0907_143818 |
| 1 정지 | h1_red_green_deadband | (데드밴드 0.6 가정) 재출발 | **초기 출발부터 불가**: 정지 상태 출발 명령 최대 0.5 m/s² < 0.6 | 무효(미출발) | h1_red_green_deadband_0907_150014 |
| 1 정지 | h1_pedestrian | 급출발 보행자 충돌 없음 | 36 km/h 에서 **−2.7 m/s²** 제동(obstacle_stop, 횡단 객체), 최소거리 11.4 m, 충돌 없음 | PASS | h1_pedestrian_0907_145813 |
| A13 | hA13_respawn | 리스폰 30 s 내 재출발 | 리스폰 후 **60 s 정지**, `Ego is out of route` ×18, route SET·AUTONOMOUS 유지 | FAIL (D8 재현) | hA13_respawn_0907_150311 |
| 3 차선변경 | h3_route_lc (49 km/h) | 필수 차선변경 전부 실행 | 지시등 에피소드 5 (L,R,R,R,L) 중 **4 실행·1 미실행**(R@71 s), 마지막 L 은 정지 동반(43 s 정지 후 종료) | FAIL | h3_route_lc_0907_150551 |
| 3 차선변경 | h3_route_lc_30 (30 km/h) | 같음 | **속도 제한 미적용**(발행 단계 무한 대기 사건, §4) → 49 km/h 로 주행, 결과 h3_route_lc 와 동일. 재실행 필요 | 무효 | h3_route_lc_30_0907_153035 |
| 2 우회 | h2_free_left | 우회 yes | 정지차 뒤 4.76 m 정지, 차선전환 0 | FAIL(설계 대비) | h2_free_left_0907_150948 |
| 2 우회 | h2_moving_left | 양보 후 우회 yes | 정지차 뒤 4.75 m 정지, 차선전환 0 | FAIL(설계 대비) | h2_moving_left_0907_152811 |
| 2 우회 | h2_red_queue | 우회 no(대기열 합류) | 대기열 차 뒤 **15.7 m** 정지(정지점 = 자차 184 m, §3), 우회 없음 | PASS | h2_red_queue_0907_151721 |
| 2 우회 | h2_stopline_25 | 우회 no | 정지차 뒤 12.7 m 정지 | PASS | h2_stopline_25_0907_152007 |
| 2 우회 | h2_stopline_30 | 우회 yes | 정지차 뒤 7.7 m 정지 | FAIL(설계 대비) | h2_stopline_30_0907_152248 |
| 2 우회 | h2_stopline_35 | 우회 yes | 정지차 뒤 **2.7 m** 정지 | FAIL(설계 대비) | h2_stopline_35_0907_152528 |

공통 관측: 최고속 47.1 km/h(max_vel 13.6 → 실속 13.1). 정지 상태 출발 명령: 첫 +명령 0.18 → 0.5 s 뒤 0.46 → 이동 전 최대 **0.50~0.57 m/s²**(engage·녹색 재출발 동일).

## 2. 해석 (수정 타겟과의 대응)

- **묶음 1**: 감속 −1.1 m/s² 가 실측으로 확인됨(정지거리 49 km/h 기준 ~99 m). 보행자 케이스는 obstacle_stop 이 −2.7 로 세움(스무더 계획 −1.0 을 제어 오차로 넘김) — 궤적 정지점이 늦어 급제동으로 귀결되는 구조.
- **묶음 3**: 필수 차선변경 5 중 4 실행. 미실행 1 건(우측)과 마지막 좌측 변경 후 43 s 정지(차선 끝 정지 의심)가 사전테스트 "차선변경 미발동" 의 mock 재현. 30 km/h 재측정 필요.
- **묶음 2**: 자차 차선 정지차는 전부 정지(회피 모듈 off). **정지선 25/30/35 m 케이스 모두 자차가 시작점 ~184 m 지점에 정지**(정지선 38 m 전) — 정지 위치가 차 위치와 무관: bag 의 STOP factor 는 `lane_change_left: no safe path`(94→67 m) 와 `obstacle_stop`. 즉 경로상 필수 좌측 차선변경이 앞 정지차 때문에 "안전 경로 없음" 이 되고, 차선변경 터미널 정지 + obstacle_stop 이 겹쳐 선다. 사전테스트 "좌회전 차선 정지차 → 정지" 와 같은 메커니즘. 판단 노드가 이 상황에서 외부요청 LC 를 승인해야 함(정지선 ≥28 m 조건).
- **A13**: 리스폰 후 route uuid 불변 → `Ego is out of route` 로 영구 정지. 설계(change_to_stop → change_route_points → change_to_autonomous) 의 필요성 확인.
- **h0**: VTD 미연결 강행·정리 훅 문제 확인(§4).

## 3. 정지 위치가 차와 무관했던 케이스의 수치 (h2 정지선 시리즈)

| 케이스 | 정지차 위치(시작점 기준) | 자차 정지 위치 | 간격 |
|---|---|---|---|
| red_queue(적신호) | 199.9 m | 184.2 m | 15.7 m |
| stopline_25 | 196.9 m | 184.2 m | 12.7 m |
| stopline_30 | 191.9 m | 184.2 m | 7.7 m |
| stopline_35 | 186.9 m | 184.2 m | 2.7 m |

자차 경로 차선열 16249→15750→15414→15207→14890→**14633**(정지선 221.9 m)→14611(좌측 이웃, 좌회전 차선)→18858. 184 m 는 14890 안. 수정 후 이 시리즈로 "우회 vs 대기" 경계를 25/30/35 m 에서 실측한다.

## 4. 하네스 자체 사건과 해결 (수정안 검증에 그대로 해당되는 것 포함)

1. **준비 실패(is_autonomous_mode_available=False 170 s)·미출발** — hA13·h3·h2_free_left(1차)·h1_pedestrian(1차).
   원인: 하네스 bag recorder 가 케이스 종료 후 살아남음(서브셸 pid 오기록 + SIGINT 로 안 죽음) → `/rosbag2_recorder` 노드 이름 중복 →
   `system.duplicated_node_checker` ERROR → `mrm_handler` NORMAL→MRM_OPERATING→MRM_FAILED(`EMERGENCY_STOP call timed out`) →
   hazard emergency=True → 자율주행 불가 또는 정지 유지(−1.5). bag 으로 확인(궤적은 정상 발행 중이었음).
   해결: recorder `--node-name rec_<케이스>`, 종료 TERM→KILL, harness_kill_leftovers 가 하네스 recorder 전부 정리. 재실행 후 전부 정상 준비(24~29 s).
   **실주행 함의**: 같은 이름의 노드가 둘이면(예: rviz 2개, record.sh 2회, 다른 클론이 같은 도메인) MRM 비상정지로 자율주행이 막힌다. 기동 체크리스트 항목.
2. **start_autonomous.sh 정리 훅**: 비대화형 셸이 백그라운드로 띄운 스크립트는 SIGINT 무시 상태를 상속해 `trap INT` 가 안 걸림 → INT 후 90 s 잔존 25. **TERM 이면 3 s 내 0**. 하네스·stop.sh 는 TERM 을 쓴다. (터미널 Ctrl+C 는 별개로 동작.)
3. **h3_route_lc_30 무한 대기**(15:31~16:46, 코디네이터가 pub 프로세스 종료): `ros2 topic pub --once /planning/scenario_planning/max_velocity_default` 가 QoS 불일치로 구독자 매칭을 기다리며 멈춤 → engage 단계로 못 감. 해결: `set_velocity_limit` = 파이썬 발행(`/planning/scenario_planning/max_velocity_candidates`, sender=harness, transient_local/reliable, timeout 20), lib.sh 외부 명령 전부 timeout, 케이스 종료 시 mock 시각이 RUN_SEC+300 을 넘으면 FAIL 기록.
4. **mock 데드밴드 부재**: 자전거 모델은 어떤 양의 가속에도 움직여 사전테스트의 재출발 교착(D1)이 재현되지 않음(녹색 후 1.9 s 재출발). 대리 지표 = 정지 상태 출발 명령 최대치 **0.50~0.57 m/s²**(baseline). `--start-deadband 0.6` 가정에선 초기 출발조차 불가. **실 VTD 데드밴드 실측이 별도 필요**(0단계). 묶음 1 적용 후 이 지표가 얼마나 오르는지로 비교.
5. **h1_pedestrian 1차 무효**: 1의 원인으로 자차 미출발(PASS 가 무의미) → 판정에 "ego 미출발이면 FAIL" 전제 추가 후 재실행(2차 유효).
6. 측정 보정: 최대감속은 주행 중(v>0.5) 구간만, 스택 종료 신호(`event stop`) 이후·리스폰 순간이동 스텝 제외(브리지 페일세이프 −3.0, 순간이동 −7.5 오염 제거). 재출발 판정은 이벤트 +0.5 s 이후 v>1.0 이 1 s 유지.
7. 케이스 순서·잔존: 도메인 43 의 NG 잔존 스택과는 분리됨(53). 다른 세션이 cwd /home/a, 도메인 미설정으로 autoware.launch.xml 을 간헐 기동(단명) — 하네스와 무관.

## 5. 남은 일

- h3_route_lc_30 재실행(속도 제한 발행 수정 후). h1_red_green_deadband 는 실측 데드밴드 값으로 재설정.
- 수정 묶음 적용 후 같은 케이스 재실행 → 이 표와 나란히 비교. 우회 케이스는 lc.json(지시등·실행)과 obj_passed 로 판정.
- bag 3.0 GB/28 실행: 오래된 실행 디렉터리는 정리 대상.

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

## 6. 묶음 1 적용 결과 — 보행자 급출발, 거리 트리거 케이스 (9/7 저녁, c6f2bdb + 정정 c929dbb)

설계 변경: 이전 h1_pedestrian 은 "ego 첫 이동 9 s 후" 출발이라 max_acc 1.0→1.5 로 ego 가 빨라지자 보행자 출발 시 ego 거리가 ~45→~26 m 로 줄어
물리적으로 정지 불가한 기하가 됐다(19:34 run: 26 m·13.4 m/s, obstacle_stop STOP 8 m 에서야, run_out 요인 없음 = 모듈 미탑재). 파라미터 비교가 되도록
**mover 출발을 ego 거리 기준(`--mover … d:D`)** 으로 바꾸고 D=50/40/30 m(우측 5.5 m, 2.5 m/s, 8 s 지속)로 측정.

발견: 묶음 1 커밋 c6f2bdb 의 preset 편집이 `launch_run_out_module` 이 아니라 `launch_obstacle_velocity_limiter_module` 을 true 로 바꿨음
(d50 bag 에 run_out 요인·로그 0건으로 확인) → 19:43 정정 커밋 c929dbb(run_out true, limiter false). 세트 1 의 d50/d40 은 run_out **없음**,
d30 은 정정 편집 직후 기동이라 run_out **있음**. 세트 2 는 셋 다 run_out 있음.

| 케이스 (출발 시 ego 거리·속도) | run_out | 결과 | 최소거리 (그때 ego v) | 횡단점 전 최저속 | 최대감속 실측 / 명령 | 첫 반응 모듈 (bag t, 거리) |
|---|---|---|---|---|---|---|
| d50 (49.7 m·10.5 m/s) | 없음 | 감속통과 | 7.45 m (4.6) | 0.73 m/s | −3.4 / −3.5 | obstacle_stop STOP 29 m → road_user_stop → obstacle_slow_down |
| d50 (50.0 m·10.5 m/s) | **있음** | 통과 | 6.63 m (6.6) | 5.6 m/s | −2.5 / −2.5 | obstacle_stop STOP 28 m (**run_out 요인 없음**) |
| d40 (39.5 m·12.0 m/s) | 없음 | 감속통과 | 4.32 m (5.4) | 4.0 m/s | −4.2 / −4.7 | obstacle_stop STOP 25 m → road_user_stop → slow_down |
| d40 (39.8 m·11.9 m/s) | **있음** | **정지** | 6.13 m (3.5) | 0.0 | −4.5 / −4.7 | **run_out SLOW 31 m → run_out STOP 28 m** → obstacle_stop STOP 25 m |
| d30 (29.6 m·13.2 m/s) | 있음 | 충돌 | 0.69 m (9.3) | 9.0 | −2.6 / −2.9 | run_out SLOW 21 m → STOP 24 m → obstacle_stop 10 m |
| d30 (29.7 m·13.1 m/s) | 있음 | 충돌 | 0.61 m (9.6) | 9.2 | −2.9 / −3.4 | run_out SLOW 21 m → STOP 24 m → obstacle_stop 10.5 m |

읽는 법
- **run_out 은 실제로 요인을 낸다**(d40·d30: SLOW → 0.2 s 뒤 STOP, obstacle_stop 보다 0.2~0.5 s·3~13 m 먼저). d50 에서는 run_out 요인이 없고
  obstacle_stop 이 처리 — 보행자 예측경로가 아직 궤적과 시간 겹침이 없어(if_ego_arrives_first 무시 규칙 또는 겹침 전) run_out 이 개입 안 한 것으로 보임.
  이 케이스에서 obstacle_stop 만으로 통과(최저속 5.6 m/s)했으므로 결과는 안전하지만 "정지" 는 아님.
- d40 이 유일하게 run_out 유무로 결과가 갈림: 없음=감속통과(4.3 m, 5.4 m/s), 있음=**정지**(6.1 m). 30~40 m 대역이 묶음 1 의 이득 구간.
- d30(13 m/s 에서 30 m)은 물리 한계 밖(−2.5 기준 정지거리 ~34 m + 반응 0.5 s×13 m = 40 m 이상): 두 세트 모두 충돌. run_out 은 STOP 을 냈지만
  이미 늦음. 이 대역은 판단 노드의 **도로변 보행자 전방 감속(30 km/h)** 규칙만이 답(8.3 m/s 면 정지거리 ~14 m + 반응 4 m).
- 사전테스트 재현 관점: "길가에서 뛰어나오는 보행자" 는 출발 시점 ego 거리로 결과가 정해진다. 재현 시나리오는 이 세 거리로 만든다.

d40 에서 실측 감속 −4.46 이 스무더 한계 −2.5 를 넘은 경로 (bag `decelpath.py`, run_out 첫 반응 t=18.9 부터)

| bag t | v 실제 | v 계획(자차점) | 계획 정지점까지 | gate 가속(→VTD) |
|---|---|---|---|---|
| 18.9 | 12.24 | 11.93 | — | +1.58 (가속 중) |
| 19.1 | 12.47 | 11.86 | 25.2 m | +0.53 |
| 19.5 | 12.58 | 10.99 | 20.1 m | −1.42 |
| 19.9 | 11.91 | 9.61 | 15.3 m | −3.37 |
| 20.1 | 11.27 | 8.84 | 13.0 m | −4.42 |
| 20.3 | 10.45 | 8.06 | 11.0 m | **−4.73** |
| 20.9 | 8.10 | 5.71 | 5.6 m | −2.5 |
| 21.9 | 5.57 | 0.55 | 3.3 m | −2.5 |
| 24.1 | 0.08 | 1.04 | 0.6 m | −1.55 |

경로: run_out/obstacle_stop 이 25 m 앞에 정지점 삽입(19.1) → 스무더가 −2.5·저크 −2.0 으로 계획하지만 자차는 그 순간 **+1.6 m/s² 가속 중**이라
계획 속도(자차점)가 실제보다 0.6→2.4 m/s 낮게 벌어짐 → PID 가 추종 오차로 −4.7 까지 명령 → **vehicle_cmd_gate 는 클램프하지 않음**
(로그의 gate 필터는 속도변화 ±0.15 m/s 한 번뿐, 종가속 한계에 안 걸림) → VTD 로 −4.7 전송 → 실측 −4.46. 즉 초과분은 스무더가 아니라
**PID 추종 오차**에서 나오고, 가속 중 급정지 요청일수록 커진다. 정지선 2 m 규정·승차감 관점의 상한을 두려면 gate 의 종가속 한계(현재 미작동)나
PID 출력 한계로 막아야 하며, 스무더 값으로는 제어되지 않는다. (trajectory_follower 원출력 토픽은 bag 미기록 → 다음 캡처 목록에 추가.)

관련 도구: `tools/harness/stopwhy.py`(모듈별 SLOW/STOP 타임라인·첫 반응), `decelpath.py`(계획 vs 실제 vs 명령), 케이스 `cases/h1_ped_d{50,40,30}.conf`.

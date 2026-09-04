# 2026 HL FMA — VTD 자율주행

# 운용 절차 (사전테스트 · 대회장 · 연구실)

## A. 대회장 / 사전테스트 (제어기 PC 본체만 지참, 케이블 꽂으면 192.168.50.10 자동)

```bash
# 0) 받은 경로 CSV 넣기 (seq,x,y) — 한 줄 수정
cp /media/.../route.csv ~/hlfma/route/            # USB 등
sed -i 's|^csv_path:.*|csv_path: /home/a/hlfma/route/route.csv|' ~/hlfma/route/route_config.yaml
python3 ~/2026-HL-FMA-VTD/tools/check_route.py ~/hlfma/route/route.csv   # 경로 점검 (선택)

# 1) 기동 (브리지 + Autoware + rviz, 경로 자동 주입)  — 터미널 1
cd ~/2026-HL-FMA-VTD && ./start_autonomous.sh
#    rviz 는 터미널2: ./rviz.sh — ego 위치, 경로(초록), 다음 신호등 확인. 브리지 로그: tail -f ~/hlfma/logs/bridge_latest.log

# 2) 기록 (선택, 터미널 2)
./record.sh 사전테스트1        # → test/<시각>_사전테스트1/ (VTD 이더넷 pcap + 네트워크 상태 + 로그)

# 3) 출발 (운영측 Start 후)  — 터미널 3
./start_hlfma.sh

# 종료: 터미널 1 에서 Ctrl+C (브리지도 같이 내려감)
```
- 자동 engage 를 원하면: `AUTO_ENGAGE=true ./start_autonomous.sh` (경로 SET 즉시 자율주행 전환)
- 폴백(자체 스택): `python3 ~/2026-HL-FMA-VTD/tools/run_real.py 192.168.50.11 ~/hlfma/route/route.csv`

## B. 연구실 (시뮬 PC 192.168.50.11)

```bash
# 시뮬 PC: 라이선스 + VTD (sudo 비번 필요)
~/HLFMA/sim_start.sh --setup=00_HL_VTD --autoConfig
#   VTD 가 CONFIG 단계에 머물면(9910 안 열림): 제어기에서  python3 tools/scp_ctrl.py 192.168.50.11 xml '<SimCtrl><Apply/></SimCtrl>'

# 제어기: 시나리오 로드·Init·Start + 관전 카메라 (연구실 전용 — 대회장 금지)
cd ~/2026-HL-FMA-VTD/tools && python3 lab_restart_scenario.py 192.168.50.11 HL_FMA_VTD_LivingLab_real.xml --cam high

# 제어기: 기동 (route_config.yaml 의 CSV 사용)
cd ~/2026-HL-FMA-VTD && ./start_autonomous.sh
#   또는 특정 CSV:  ROUTE_CSV=~/2026-HL-FMA-VTD/tools/real_route_path1.csv ./start_autonomous.sh

# mock 회귀 (시뮬 PC 없이, 약 6분): 9개 PASS 가 정상
bash ~/2026-HL-FMA-VTD/tools/regress_mock.sh
```

## C. 확인 명령

```bash
tail -f ~/hlfma/logs/bridge_latest.log                       # 연결·경로·신호등·워치독
ros2 topic echo /api/routing/state --once --qos-durability transient_local --qos-reliability reliable   # state 2 = SET
ros2 topic echo /api/operation_mode/state --once --qos-durability transient_local --qos-reliability reliable  # mode 2 = AUTONOMOUS
ros2 topic hz /planning/trajectory                            # 10Hz
bash ~/2026-HL-FMA-VTD/tools/check_topic_contract.sh          # 발행자 없는 구독 토픽 0 이어야 정상
```

## D. 빌드 (hlfma_ws — 새 터미널이면 환경은 .bashrc 가 잡음)
```bash
cd ~/2026-HL-FMA-VTD/hlfma_ws && colcon build                          # 전체 (증분)
colcon build --packages-select vtd_autoware_bridge autoware_launch      # 우리 것만 (수 초)
```

---

VTD(Virtual Test Drive) 시뮬레이션 환경에서 동작하는 자율주행 스택.
시뮬레이터로부터 센서 데이터를 수신해 **인지 → 판단 → 제어** 전 과정을 수행하고 제어 신호를 송출한다.

## 시스템 구성

```
┌──────────────┐   랜선 직결    ┌──────────────────────┐
│   VTD PC     │◄──────────────►│  제어기 (이 저장소)   │
│  시뮬레이션   │   enp89s0      │  인지 · 판단 · 제어   │
└──────────────┘  2.5GbE        └──────────────────────┘
```

---

## 제어기 PC 사양

기준일: 2026-08-13

### 하드웨어

| 항목 | 내용 |
|---|---|
| 제조사 / 모델 | Micro-Star International (MSI) — GS76 Stealth 11UG (REV 1.0) |
| CPU | Intel Core i9-11900H (Tiger Lake-H) — 8코어 16스레드, 0.8 ~ 4.9GHz |
| RAM | 32GB (가용 31GiB) + swap 8GB |
| GPU (외장) | NVIDIA RTX 3070 Laptop / Max-Q (GA104M, `10de:249d`) — VRAM 8GB, compute capability 8.6, SM 40개 |
| GPU (내장) | Intel UHD Graphics (Tiger Lake-H GT1, `8086:9a60`) — 드라이버 `i915` |
| 저장장치 | Micron 3400 NVMe 1TB (`MTFDKBA1T0TFH`, 953.9GB) |
| 파티션 | `/` = `/dev/nvme0n1p5`, ext4, 720.5GB (여유 630.7GB) |
| 유선 LAN | Killer E3000 2.5GbE (Realtek `10ec:3000`) — `enp89s0`, MAC `2c:f0:5d:fe:c6:19` |
| 무선 LAN | Intel Wi-Fi 6E AX210 (`8086:2725`) — `wlp92s0` |
| BIOS | AMI `E17M1IMS.116` (2022-01-10) |
| Secure Boot | 비활성화 |

### 소프트웨어

| 구성요소 | 버전 |
|---|---|
| OS | Ubuntu 24.04.4 LTS |
| 커널 | 7.0.0-28-generic |
| 세션 타입 | X11 |
| **ROS 2** | **Jazzy Jalisco** — `ros-jazzy-desktop` 0.11.0 (LTS, 2029.5까지) |
| NVIDIA 드라이버 | 595.84 (`nvidia-driver-595-open`, DKMS) |
| CUDA Toolkit | 13.3.73 (`/usr/local/cuda`) |
| cuDNN | 9.25.0 (cuda-13) |
| TensorRT | 11.2.1.2+cuda13.3 |
| gcc | 13.3.0 |
| Python | 3.12.3 (시스템) |
| NumPy | 1.26.4 (시스템) |

### 현재 런타임 상태

| 항목 | 상태 |
|---|---|
| GPU 커널 드라이버 | `nvidia` (`nvidia`, `nvidia_modeset`, `nvidia_drm`, `nvidia_uvm` 로드됨) |
| DKMS | `nvidia/595.84, 7.0.0-28-generic, x86_64: installed` |
| `enp89s0` | `DOWN` / `NO-CARRIER` (랜선 미연결) |
| `wlp92s0` | `UP` |
| CPU governor | `powersave` |
| 시각 동기 | `systemd-timesyncd` active — `ntp.ubuntu.com`, 타임존 `Asia/Seoul` |

---

## 환경 변수

`~/.bashrc`에 등록되어 있다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43
export CUDA_HOME=/usr/local/cuda
export PATH="${CUDA_HOME}/bin:${PATH}"
export LD_LIBRARY_PATH="${CUDA_HOME}/lib64:${LD_LIBRARY_PATH:-}"
```

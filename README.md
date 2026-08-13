# 2026 HL FMA — VTD 자율주행

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

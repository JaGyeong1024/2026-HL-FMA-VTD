# 실행 순서 (제어기 PC)

## 준비 (터미널 1)
cd ~/2026-HL-FMA-VTD
./start_autonomous.sh              # 브리지 + Autoware, route_config.yaml 의 CSV 자동 주입 (rviz 없음)
#   로그: tail -f ~/hlfma/logs/bridge_latest.log   → "경로 설정 성공" 확인

## rviz (선택, 터미널 2 — 제어기 화면에서, ssh 불가)
./rviz.sh

## 시작 (터미널 3, 운영측 Start 후)
./start_hlfma.sh                   # 경로 SET 확인 → 자율주행 전환

## 자동 시작 (준비와 동시에 engage)
AUTO_ENGAGE=true ./start_autonomous.sh

## 경로 CSV 교체
cp <받은파일>.csv ~/hlfma/route/
sed -i "s|^csv_path:.*|csv_path: /home/a/hlfma/route/<받은파일>.csv|" ~/hlfma/route/route_config.yaml

## 기록 (선택, 터미널 4)
./record.sh 이름                   # test/<YYYYMMDD_HHMMSS>_이름/  (Ctrl+C 종료)
#   net/vtd.pcap(VTD 이더넷 원본, 타임스탬프) net/(ip·ss·nstat·ping) logs/(bridge·autoware·ros) meta.txt
#   bag/(/vtd/raw_rx /vtd/raw_tx)  — BAG=0 이면 생략
#   ALL=1 ./record.sh 이름         # bag 에 전 토픽 (타이밍 문제 증거 보존용, 용량 큼)
#   pcap 1회 설정(완료): sudo setcap cap_net_raw,cap_net_admin=eip /usr/bin/tcpdump
## 종료
터미널 1 에서 Ctrl+C (브리지 같이 종료). 강제: pkill -f ros-args; pkill -f vtd_autoware_bridge

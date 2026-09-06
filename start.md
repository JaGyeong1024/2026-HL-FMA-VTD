# 실행 순서 (제어기 PC)

## 기동 + 출발 (터미널 1) — 한 번에
cd ~/2026-HL-FMA-VTD
./start_autonomous.sh              # 브리지 + Autoware 기동 → route_config.yaml 의 CSV 주입 → 경로 SET → 자율주행 가능 → engage
#   로그: tail -f ~/hlfma/logs/bridge_latest.log   → "경로 설정 성공" / engage 결과는 ~/hlfma/logs/engage_<시각>.log
#   기동만: ENGAGE=false ./start_autonomous.sh

## rviz (선택, 터미널 2 — 제어기 화면에서, ssh 불가)
./rviz.sh

## 수동 출발 (ENGAGE=false 로 기동했을 때, 터미널 3)
./start_hlfma.sh                   # 경로 SET 확인 → 자율주행 가능 대기 → 전환

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

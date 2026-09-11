#!/usr/bin/env bash
# VTD 데이터 포트가 실제로 스트리밍을 시작하면 곧바로 실주행 실험을 돌린다.
# 스택을 미리 띄워두면 브리지가 9910 을 점유해 직접 확인을 못 하므로, 여기서는
# 스택을 내려둔 채 포트만 짧게 찔러본다.
ROOT=/home/a/2026-HL-FMA-VTD-NG
DUR="${1:-150}"
LABEL="${2:-fix12}"
echo "[wait] VTD 스트리밍 대기 시작 $(date +%H:%M:%S)"
while true; do
  if python3 - <<'PY'
import socket, sys
try:
    s = socket.create_connection(("192.168.50.11", 9910), timeout=3)
    s.settimeout(3)
    d = s.recv(4096)
    s.close()
    sys.exit(0 if d else 1)
except Exception:
    sys.exit(1)
PY
  then
    echo "[wait] VTD 데이터 감지 $(date +%H:%M:%S) → 실험 시작"
    sleep 2
    exec "$ROOT/tools/run_eval.sh" "$DUR" "$LABEL"
  fi
  sleep 10
done

"""연구실 전용 — SCP(48179)로 VTD 시나리오 재시작 + 관전 카메라 설정.

**대회장에서는 절대 실행하지 말 것.** Stop→LoadScenario→Init→Start를 SCP로 보내면
운영측이 이미 돌리고 있는 채점 세션이 파괴된다. run_real.py는 이 파일을 import하지
않는다 — 재시작이 필요한 것은 연구실 mock_vtd/psim 검증 상황뿐이며, 그때만 명시적으로
이 스크립트를 실행한다.

usage: python3 lab_restart_scenario.py [host] [scenario.xml] [--cam preset]
  preset: high(기본) / topdown / far / chase — set_view_camera.PRESETS 참조

절차: SCP stop → load scenario → init(InitDone 대기) → IG ready 대기 → start
      → (옵션) 관전 카메라 설정.
"""
import sys
import time

import scp_ctrl
import set_view_camera

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.50.11"
SCENARIO = sys.argv[2] if len(sys.argv) > 2 else "HL_FMA_VTD_LivingLab_real.xml"
CAM_PRESET = None
if "--cam" in sys.argv:
    i = sys.argv.index("--cam")
    CAM_PRESET = sys.argv[i + 1] if i + 1 < len(sys.argv) else "high"


def main():
    print(f"[lab_restart_scenario] host={HOST} scenario={SCENARIO}")
    print("시뮬 재시작 (SCP stop→load→init: InitDone 대기→start)...")
    bus = scp_ctrl.connect(HOST)
    scp_ctrl.send(bus, '<SimCtrl><Stop /></SimCtrl>'); time.sleep(2.0)
    scp_ctrl.send(bus, f'<SimCtrl><LoadScenario filename="{SCENARIO}" /></SimCtrl>'); time.sleep(2.0)
    scp_ctrl.send(bus, '<SimCtrl><Init mode="operation" /></SimCtrl>')
    if not scp_ctrl.wait_for(bus, "InitDone", timeout=120.0):
        bus.close()
        raise SystemExit("Init 완료 신호(InitDone) 120초 내 미수신 — VTD 상태 확인 필요")
    print("InitDone 수신 → IG ready 대기")
    if not scp_ctrl.wait_for(bus, ['name="IG_', 'state="ready"'], timeout=90.0):
        print("  (IG ready 브로드캐스트 미수신 — 그래도 Start 시도)")
    time.sleep(3.0)
    print("Start 전송")
    scp_ctrl.send(bus, '<SimCtrl><Start /></SimCtrl>')
    bus.close()

    if CAM_PRESET:
        try:
            set_view_camera.send_scp(HOST, set_view_camera.PRESETS[CAM_PRESET])
            print(f"관전 카메라 설정: {CAM_PRESET}")
        except OSError as e:
            print(f"(카메라 설정 실패: {e})")


if __name__ == "__main__":
    main()

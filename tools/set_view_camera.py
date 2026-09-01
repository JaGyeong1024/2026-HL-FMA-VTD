"""VTD 관전 카메라 설정 — SCP(포트 48179)로 시점 변경.

usage: .venv/bin/python set_view_camera.py [host] [preset]
  preset: topdown(기본, ego 10m 위 조감) / chase(후방 추격) / far(30m 조감)

SCP 헤더: magic(u16)=40108, version(u16)=1, sender[64], receiver[64], len(i32) + XML
카메라 명령 근거: VTD_UserManual 'Special Message Formats' — <Camera><PosRelative/>
<ViewPlayer/><Set/>. PosRelative는 플레이어 기준 상대 위치에 카메라를 묶는다(tether).
"""
import socket
import struct
import sys

MAGIC = 40108
VERSION = 1

PRESETS = {
    # 기본: ego 후방 10m·고도 10m에서 전방 아래로 비스듬히 (ViewPlayer가 ego를 조준 → 자연스런 부감)
    "high":    '<Camera name="birdCam"><PosRelative player="Ego" dx="-10.0" dy="0.0" dz="10.0"/>'
               '<ViewPlayer player="Ego"/><Set/></Camera>',
    # ego 10m 위, 살짝 뒤에서 내려다보기 (정수직은 up-vector 특이점 → dx 약간 후방)
    "topdown": '<Camera name="birdCam"><PosRelative player="Ego" dx="-3.0" dy="0.0" dz="10.0"/>'
               '<ViewPlayer player="Ego"/><Set/></Camera>',
    "far":     '<Camera name="birdCam"><PosRelative player="Ego" dx="-8.0" dy="0.0" dz="30.0"/>'
               '<ViewPlayer player="Ego"/><Set/></Camera>',
    "chase":   '<Camera name="birdCam"><PosRelative player="Ego" dx="-12.0" dy="0.0" dz="4.0"/>'
               '<ViewPlayer player="Ego"/><Set/></Camera>',
}


def send_scp(host, xml, port=48179):
    pkt = struct.pack("<HH64s64si", MAGIC, VERSION,
                      b"hlfma-controller", b"TaskControl", len(xml)) + xml.encode()
    with socket.create_connection((host, port), timeout=5.0) as s:
        s.sendall(pkt)
    print(f"SCP 전송 OK → {host}:{port}\n{xml}")


if __name__ == "__main__":
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.50.11"
    preset = sys.argv[2] if len(sys.argv) > 2 else "high"
    send_scp(host, PRESETS[preset])

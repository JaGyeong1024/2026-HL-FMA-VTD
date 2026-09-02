"""VTD 관전 카메라 설정 — SCP(포트 48179)로 시점 변경.

usage: .venv/bin/python set_view_camera.py [host] [preset]
  preset: high(기본, 후방 16m·고도 20m 부감) / low(후방 10m·10m) / topdown / far(30m) / chase

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
    # 관전 화면(mainRS)만. 후방 15m·고도 20m 에서 ego 를 조준하되 피치를 들어 전방도 넓게.
    # <Set> 의 renderSurface 로 mainRS 에 한정 → LiDAR/Broadcast 프리뷰 시점은 안 건드림.
    "high":    '<Camera name="birdCam" renderSurface="mainRS">'
               '<PosRelative player="Ego" dx="-15.0" dy="0.0" dz="20.0"/>'
               '<Rotation h="0.0" p="-0.61" r="0.0"/>'          # p=-35° 내려다봄 (전방 조망 확보)
               '<Set renderSurface="mainRS"/></Camera>',
    # 이전(ego 조준, 후방 15m·20m) — 피치 자동
    "aim":     '<Camera name="birdCam" renderSurface="mainRS"><PosRelative player="Ego" dx="-15.0" dy="0.0" dz="20.0"/>'
               '<ViewPlayer player="Ego"/><Set renderSurface="mainRS"/></Camera>',
    "low":     '<Camera name="birdCam" renderSurface="mainRS"><PosRelative player="Ego" dx="-10.0" dy="0.0" dz="10.0"/>'
               '<ViewPlayer player="Ego"/><Set renderSurface="mainRS"/></Camera>',
    "topdown": '<Camera name="birdCam" renderSurface="mainRS"><PosRelative player="Ego" dx="-3.0" dy="0.0" dz="10.0"/>'
               '<ViewPlayer player="Ego"/><Set renderSurface="mainRS"/></Camera>',
    "far":     '<Camera name="birdCam" renderSurface="mainRS"><PosRelative player="Ego" dx="-25.0" dy="0.0" dz="30.0"/>'
               '<ViewPlayer player="Ego"/><Set renderSurface="mainRS"/></Camera>',
    "chase":   '<Camera name="birdCam" renderSurface="mainRS"><PosRelative player="Ego" dx="-12.0" dy="0.0" dz="4.0"/>'
               '<ViewPlayer player="Ego"/><Set renderSurface="mainRS"/></Camera>',
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

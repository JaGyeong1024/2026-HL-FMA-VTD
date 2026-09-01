"""VTD SCP(48179) 원격 제어 — 시나리오 로드/초기화/시작/정지.

usage: python3 scp_ctrl.py <host> <cmd> [arg]
  cmd: load <scenario.xml> | init | start | stop | pause | xml '<...>' | listen [sec]

listen: SCP 수신 패킷을 지정 시간(기본 5초) 동안 덤프 (상태 확인용).
헤더: magic(u16)=40108, ver(u16)=1, sender[64], receiver[64], len(i32) + XML
"""
import socket
import struct
import sys
import time

MAGIC, VERSION = 40108, 1
HDR = struct.Struct("<HH64s64si")


def pack(xml: str) -> bytes:
    return HDR.pack(MAGIC, VERSION, b"hlfma-controller", b"TaskControl", len(xml)) + xml.encode()


def recv_msgs(sock, duration):
    sock.settimeout(0.5)
    buf = b""
    end = time.time() + duration
    while time.time() < end:
        try:
            chunk = sock.recv(65536)
            if not chunk:
                break
            buf += chunk
        except socket.timeout:
            continue
        while len(buf) >= HDR.size:
            magic, ver, snd, rcv, n = HDR.unpack(buf[:HDR.size])
            if len(buf) < HDR.size + n:
                break
            xml = buf[HDR.size:HDR.size + n].decode(errors="replace")
            buf = buf[HDR.size + n:]
            sender = snd.split(b"\0")[0].decode()
            print(f"[{sender}] {xml}")


def connect(host, port=48179, timeout=5.0):
    import socket
    return socket.create_connection((host, port), timeout=timeout)


def send(sock, xml):
    sock.sendall(pack(xml))


def wait_for(sock, substrs, timeout=90.0):
    """SCP 버스에서 substrs(문자열 또는 목록 — 모두 포함)가 든 메시지 대기. 성공 시 True."""
    if isinstance(substrs, str):
        substrs = [substrs]
    sock.settimeout(1.0)
    buf = b""
    end = time.time() + timeout
    while time.time() < end:
        try:
            chunk = sock.recv(65536)
            if not chunk:
                return False
            buf += chunk
        except socket.timeout:
            continue
        while len(buf) >= HDR.size:
            magic, ver, snd, rcv, n = HDR.unpack(buf[:HDR.size])
            if len(buf) < HDR.size + n:
                break
            xml = buf[HDR.size:HDR.size + n].decode(errors="replace")
            buf = buf[HDR.size + n:]
            if all(s in xml for s in substrs):
                return True
    return False


def main():
    host = sys.argv[1]
    cmd = sys.argv[2]
    with socket.create_connection((host, 48179), timeout=5.0) as s:
        if cmd == "load":
            xml = f'<SimCtrl><LoadScenario filename="{sys.argv[3]}" /></SimCtrl>'
        elif cmd == "init":
            xml = '<SimCtrl><Init mode="operation" /></SimCtrl>'
        elif cmd == "start":
            xml = '<SimCtrl><Start /></SimCtrl>'
        elif cmd == "stop":
            xml = '<SimCtrl><Stop /></SimCtrl>'
        elif cmd == "pause":
            xml = '<SimCtrl><Pause /></SimCtrl>'
        elif cmd == "xml":
            xml = sys.argv[3]
        elif cmd == "listen":
            recv_msgs(s, float(sys.argv[3]) if len(sys.argv) > 3 else 5.0)
            return
        else:
            raise SystemExit(f"unknown cmd: {cmd}")
        s.sendall(pack(xml))
        print(f"SCP → {xml}")
        recv_msgs(s, 2.0)


if __name__ == "__main__":
    main()

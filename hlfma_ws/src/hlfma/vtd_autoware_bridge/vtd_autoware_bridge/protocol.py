"""HLVTD TCP 9910 프로토콜 — DataPacket(1109B) / CtrlPacket(9B). 실기·모의 공용."""
import socket
import struct
from dataclasses import dataclass

DATA_FMT = "<6f" + "I8f" * 30 + "iB"
DATA_SIZE = struct.calcsize(DATA_FMT)   # 1109
CTRL_FMT = "<ffB"
CTRL_SIZE = struct.calcsize(CTRL_FMT)   # 9
assert DATA_SIZE == 1109 and CTRL_SIZE == 9


@dataclass
class VehicleState:
    x: float; y: float; z: float
    heading: float; pitch: float; roll: float
    objects: list          # [(id, x, y, z, heading, speed, length, width, height)]
    tl_id: int
    tl_state: int


def pack_data(state: VehicleState) -> bytes:
    vals = [state.x, state.y, state.z, state.heading, state.pitch, state.roll]
    objs = list(state.objects)[:30]
    objs += [(0,) + (0.0,) * 8] * (30 - len(objs))
    for o in objs:
        vals.extend(o)
    vals += [state.tl_id, state.tl_state]
    return struct.pack(DATA_FMT, *vals)


def unpack_data(buf: bytes) -> VehicleState:
    v = struct.unpack(DATA_FMT, buf)
    objs = [tuple(v[6 + 9 * i: 6 + 9 * (i + 1)]) for i in range(30) if v[6 + 9 * i] != 0]
    return VehicleState(*v[0:6], objects=objs, tl_id=v[-2], tl_state=v[-1])


def pack_ctrl(steering: float, accel: float, turn_signal: int) -> bytes:
    return struct.pack(CTRL_FMT, steering, accel, turn_signal)


def unpack_ctrl(buf: bytes):
    return struct.unpack(CTRL_FMT, buf)


class VTDClient:
    """제어기 → VTD(또는 모의 서버) TCP 클라이언트. 1109B 재조립 수신."""

    def __init__(self, host, port=9910, timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)
        self._buf = b""

    def recv_state(self) -> VehicleState:
        while len(self._buf) < DATA_SIZE:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("연결 종료")
            self._buf += chunk
        # 밀린 경우 최신 패킷만 사용
        n = len(self._buf) // DATA_SIZE
        pkt = self._buf[(n - 1) * DATA_SIZE: n * DATA_SIZE]
        self._buf = self._buf[n * DATA_SIZE:]
        return unpack_data(pkt)

    def send_ctrl(self, steering, accel, turn_signal=0):
        self.sock.sendall(pack_ctrl(steering, accel, turn_signal))

    def close(self):
        self.sock.close()

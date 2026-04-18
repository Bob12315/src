from __future__ import annotations

import json
import socket

from models import CurrentTarget


class UdpPublisher:
    def __init__(self, udp_ip: str, udp_port: int) -> None:
        self.addr = (udp_ip, udp_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish(self, target: CurrentTarget) -> None:
        payload = json.dumps(target.to_dict(), ensure_ascii=False).encode("utf-8")
        self.sock.sendto(payload, self.addr)

    def close(self) -> None:
        self.sock.close()

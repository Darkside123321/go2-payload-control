"""Loopback-only interface between the simulator and the ROS 2 process."""
import json
import math
import socket
import time
import zlib

HOST = "127.0.0.1"
COMMAND_PORT = 26371
TELEMETRY_PORT = 26372
COMMAND_TIMEOUT_S = 0.6
MIN_SPEED = 0.02
MAX_SPEED = 0.10
MAX_DATAGRAM_BYTES = 1200
MAX_JSON_BYTES = 8192


def speed_valid(speed):
    return math.isfinite(speed) and (speed == 0 or MIN_SPEED <= speed <= MAX_SPEED)


def bind_udp(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, port))
    sock.setblocking(False)
    return sock


def send(sock, port, packet):
    data = json.dumps(packet, allow_nan=False, separators=(",", ":")).encode()
    encoded = zlib.compress(data, level=1)
    if len(encoded) > MAX_DATAGRAM_BYTES:
        raise ValueError("local message exceeds the UDP size budget")
    return sock.sendto(encoded, (HOST, port))


def packets(sock):
    # Bound socket work per tick and leave queue backpressure to the OS.
    for _ in range(64):
        try:
            raw, _ = sock.recvfrom(MAX_DATAGRAM_BYTES + 1)
        except BlockingIOError:
            return
        if len(raw) > MAX_DATAGRAM_BYTES:
            continue
        try:
            decoder = zlib.decompressobj()
            decoded = decoder.decompress(raw, MAX_JSON_BYTES)
            if not decoder.eof or decoder.unused_data:
                continue
            packet = json.loads(decoded)
        except (UnicodeDecodeError, json.JSONDecodeError, zlib.error):
            continue
        if isinstance(packet, dict):
            yield packet


class CommandWatchdog:
    def __init__(self, sock):
        self.sock = sock
        self.timestamp = None
        self.speed = None
        self.stop_reason = None
        self.rejected = 0

    def poll(self):
        now = time.monotonic()
        for packet in packets(self.sock):
            stamp, speed = packet.get("monotonic_s"), packet.get("forward_mps")
            if (type(stamp) not in (float, int) or type(speed) not in (float, int)
                    or not speed_valid(speed)
                    or not -0.05 <= now - stamp <= COMMAND_TIMEOUT_S
                    or (self.timestamp is not None and stamp <= self.timestamp)):
                self.rejected += 1
                continue
            if self.speed is None and speed > 0:
                self.speed = speed
                self.timestamp = stamp
            elif self.speed is not None and speed == 0:
                self.stop_reason = self.stop_reason or "zero velocity command"
                self.timestamp = stamp
            elif self.speed is not None and abs(speed - self.speed) < 1e-6:
                self.timestamp = stamp
            else:
                self.rejected += 1
        if self.timestamp is not None and now - self.timestamp > COMMAND_TIMEOUT_S:
            self.stop_reason = self.stop_reason or "command timeout"

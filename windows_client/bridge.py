"""PICO UDP receiver and an origin-bound, loopback-only browser stream."""

from __future__ import annotations

import hmac
import json
import secrets
import select
import socket
import threading
import time
from contextlib import ExitStack
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

from middleware.core.protocol import (
    DISCOVERY_BUSY_PREFIX, DISCOVERY_REQUEST, DISCOVERY_RESPONSE_PREFIX,
    RELEASE_REQUEST, PacketError, decode_pose_packet, is_sequence_newer,
)


PORT = 47653
FRESH_SECONDS = 0.35
DEVICE_LEASE = 3.0
DEFAULT_SITE = "https://557268-proxy-8089.dsw-gateway-cn-beijing.data.aliyuncs.com/teleoperation"


def site_origin(site):
    parts = urlsplit(site)
    if parts.username or parts.password or not parts.hostname or parts.query or parts.fragment:
        raise ValueError("请输入平台遥操作页面的完整地址")
    if parts.scheme != "https" and not (
        parts.scheme == "http" and parts.hostname in {"localhost", "127.0.0.1"}
    ):
        raise ValueError("平台地址须使用 HTTPS，本机测试可用 localhost")
    return f"{parts.scheme}://{parts.netloc}"


class Receiver:
    def __init__(self, pose_port=5005, discovery_port=5006):
        self.pose_port = pose_port
        self.discovery_port = discovery_port
        self.changed = threading.Condition()
        self.closed = threading.Event()
        self.peer = ""
        self.frame = None
        self.received = 0.0
        self.generation = 0
        self.error = ""
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._receive, daemon=True)
        self.thread.start()

    def close(self):
        self.closed.set()
        with self.changed:
            self.changed.notify_all()
        if self.thread:
            self.thread.join(timeout=1)

    def _receive(self):
        while not self.closed.is_set():
            try:
                with ExitStack() as sockets:
                    inputs = []
                    for port in (self.pose_port, self.discovery_port):
                        sock = sockets.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
                        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                            sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
                        sock.bind(("", port))
                        inputs.append(sock)
                    self.error = ""
                    while not self.closed.is_set():
                        ready, _, _ = select.select(inputs, [], [], 0.1)
                        for sock in ready:
                            packet, sender = sock.recvfrom(2048)
                            if packet == RELEASE_REQUEST:
                                with self.changed:
                                    if self.peer == sender[0]:
                                        self.peer, self.frame = "", None
                                        self.generation += 1
                                        self.changed.notify_all()
                            elif sock is inputs[1] and packet.strip() == DISCOVERY_REQUEST:
                                with self.changed:
                                    busy = self.peer and self.peer != sender[0] and time.monotonic() - self.received < DEVICE_LEASE
                                    reply = (DISCOVERY_BUSY_PREFIX + f"{self.pose_port}|{self.peer}".encode()
                                             if busy else DISCOVERY_RESPONSE_PREFIX + str(self.pose_port).encode())
                                sock.sendto(reply, sender)
                            elif sock is inputs[0]:
                                self.accept(packet, sender[0])
            except OSError as exc:
                self.error = f"UDP {self.pose_port}/{self.discovery_port} 无法接收：{exc}"
                self.closed.wait(2)

    def accept(self, data, peer):
        try:
            frame = decode_pose_packet(data)
        except PacketError:
            return
        now = time.monotonic()
        with self.changed:
            if self.frame is not None and now - self.received < DEVICE_LEASE:
                if peer != self.peer or not is_sequence_newer(frame.sequence, self.frame.sequence):
                    return
            self.peer, self.frame, self.received = peer, frame, now
            self.generation += 1
            self.changed.notify_all()

    def snapshot(self):
        with self.changed:
            age = time.monotonic() - self.received
            fresh = self.frame is not None and age <= FRESH_SECONDS
            return {
                "version": 1, "workstation": socket.gethostname(),
                "sent_ms": round(time.time() * 1000),
                "age_ms": round(age * 1000) if self.frame else None,
                "peer": self.peer if age < DEVICE_LEASE else "",
                "frame": self.frame.as_dict() if fresh else None, "error": self.error,
            }


class BrowserBridge(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = False

    def __init__(self, receiver, site, port=PORT):
        self.receiver = receiver
        self.origin = site_origin(site)
        self.lock = threading.Lock()
        self.ticket = self.grant = ""
        self.ticket_until = self.grant_until = 0.0
        self.active_stream = None
        self.threads_closed = threading.Event()
        super().__init__(("127.0.0.1", port), BrowserRequest)

    def issue(self):
        with self.lock:
            self.ticket = secrets.token_urlsafe(24)
            self.ticket_until = time.monotonic() + 180
            return self.ticket

    def exchange(self, ticket):
        with self.lock:
            if not ticket or time.monotonic() >= self.ticket_until or not hmac.compare_digest(self.ticket, ticket):
                return None
            self.ticket = ""
            self.grant = secrets.token_urlsafe(32)
            self.grant_until = time.monotonic() + 120
            self.active_stream = None
            return self.grant

    def stop(self):
        self.threads_closed.set()
        self.shutdown()
        self.server_close()


class BrowserRequest(BaseHTTPRequestHandler):
    server: BrowserBridge

    def log_message(self, *args):
        pass

    def _allowed(self):
        return (self.headers.get("Origin") == self.server.origin
                and self.headers.get("Host") == f"127.0.0.1:{self.server.server_port}")

    def _headers(self, status, media="application/json", length=None):
        self.send_response(status)
        if self._allowed():
            self.send_header("Access-Control-Allow-Origin", self.server.origin)
            self.send_header("Vary", "Origin")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Type", media)
        if length is not None:
            self.send_header("Content-Length", str(length))

    def _json(self, status, data):
        payload = json.dumps(data).encode()
        self._headers(status, length=len(payload))
        self.end_headers()
        self.wfile.write(payload)

    def do_OPTIONS(self):
        if not self._allowed() or self.path not in ("/pair", "/stream"):
            return self._json(403, {})
        self._headers(204, length=0)
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Authorization, Content-Type")
        self.send_header("Access-Control-Allow-Private-Network", "true")
        self.send_header("Access-Control-Max-Age", "300")
        self.end_headers()

    def _bearer(self):
        auth = self.headers.get("Authorization", "")
        return auth[7:] if auth.startswith("Bearer ") and auth.isascii() else ""

    def do_POST(self):
        if not self._allowed() or self.path != "/pair":
            return self._json(403, {})
        grant = self.server.exchange(self._bearer())
        self._json(200 if grant else 401, {"grant": grant} if grant else {})

    def do_GET(self):
        if not self._allowed() or self.path != "/stream":
            return self._json(403, {})
        server = self.server
        grant = self._bearer()
        stream = object()
        with server.lock:
            if not grant or time.monotonic() >= server.grant_until or not hmac.compare_digest(grant, server.grant):
                return self._json(401, {})
            if server.active_stream is not None:
                return self._json(409, {"error": "workstation already connected"})
            server.active_stream = stream
        try:
            self.connection.settimeout(1)
            self._headers(200, "application/x-ndjson")
            self.end_headers()
            while not server.threads_closed.is_set():
                with server.lock:
                    if server.active_stream is not stream:
                        break
                    server.grant_until = time.monotonic() + 120
                receiver = server.receiver
                with receiver.changed:
                    revision = receiver.generation
                    snapshot = receiver.snapshot()
                self.wfile.write(json.dumps(snapshot, separators=(",", ":")).encode() + b"\n")
                self.wfile.flush()
                with receiver.changed:
                    receiver.changed.wait_for(lambda: receiver.generation != revision or receiver.closed.is_set(), 0.15)
        except (ConnectionError, OSError):
            pass
        finally:
            with server.lock:
                if server.active_stream is stream:
                    server.active_stream = None

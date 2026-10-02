#!/usr/bin/env python3
"""User side (Windows/Mac/Linux): listens on 127.0.0.1:2222 and relays each
connection over wss to the VM through the Cloudflare Worker relay.

Requires: Python 3.8+ (no extra packages).
Run:  python user_client.py
Then: ssh -p 2222 -i <你的私钥> relayuser@127.0.0.1
"""
import socket, ssl, base64, os, struct, select

WORKER_HOST = "ssh-relay.pitrc47.workers.dev"   # Worker 的自定义域名(部署后如有变化请改这里)
SECRET = "67bc3d81eff4c918bbe368249d0b2f5ee0eed42aaf5d30410aa4624660d958d9"  # 必须与 worker.js 里的 SECRET 一致
LISTEN_PORT = 2222
VM_ID = "vm-hatch-c51f"  # 这台VM的独立房间ID


def bchr(n):
    return bytes((n,))


class WSParser:
    def __init__(self):
        self.buf = b""

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            if len(self.buf) < 2:
                break
            b1, b2 = self.buf[0], self.buf[1]
            opcode = b1 & 0x0F
            masked = (b2 & 0x80) != 0
            ln = b2 & 0x7F
            idx = 2
            if ln == 126:
                if len(self.buf) < idx + 2:
                    break
                ln = struct.unpack("!H", self.buf[idx:idx + 2])[0]
                idx += 2
            elif ln == 127:
                if len(self.buf) < idx + 8:
                    break
                ln = struct.unpack("!Q", self.buf[idx:idx + 8])[0]
                idx += 8
            if masked:
                if len(self.buf) < idx + 4:
                    break
                mask = self.buf[idx:idx + 4]
                idx += 4
            if len(self.buf) < idx + ln:
                break
            payload = self.buf[idx:idx + ln]
            if masked:
                payload = bytes(p ^ mask[i % 4] for i, p in enumerate(payload))
            self.buf = self.buf[idx + ln:]
            out.append((opcode, payload))
        return out


def ws_send(sock, payload, opcode=0x2):
    mask = os.urandom(4)
    ln = len(payload)
    hdr = bchr(0x80 | opcode)
    if ln < 126:
        hdr += bchr(0x80 | ln)
    elif ln < 65536:
        hdr += bchr(0x80 | 126) + struct.pack("!H", ln)
    else:
        hdr += bchr(0x80 | 127) + struct.pack("!Q", ln)
    sock.sendall(hdr + mask + bytes(p ^ mask[i % 4] for i, p in enumerate(payload)))


def ws_connect():
    raw = socket.create_connection((WORKER_HOST, 443), timeout=20)
    ctx = ssl.create_default_context()
    tls = ctx.wrap_socket(raw, server_hostname=WORKER_HOST)
    key = base64.b64encode(os.urandom(16)).decode()
    tls.sendall(
        f"GET /relay/{VM_ID} HTTP/1.1\r\nHost: {WORKER_HOST}\r\n"
        "Upgrade: websocket\r\nConnection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n".encode()
    )
    resp = b""
    while b"\r\n\r\n" not in resp:
        c = tls.recv(4096)
        if not c:
            raise RuntimeError("handshake closed")
        resp += c
    if b" 101 " not in resp.split(b"\r\n")[0]:
        raise RuntimeError(f"WS handshake failed: {resp[:160]!r}")
    ws_send(tls, ('{"role":"client","secret":"%s"}' % SECRET).encode(), opcode=0x1)
    return tls


def drain(sock):
    chunks = []
    while True:
        try:
            d = sock.recv(65536)
        except (BlockingIOError, ssl.SSLWantReadError):
            break
        if not d:
            return None
        chunks.append(d)
        if len(d) < 65536:
            break
    return b"".join(chunks)


def bridge(tcp, tls):
    tcp.setblocking(False)
    tls.setblocking(False)
    parser = WSParser()
    try:
        while True:
            r, _, _ = select.select([tcp, tls], [], [], 45)
            if not r:
                ws_send(tls, b"", opcode=0x9)
                continue
            if tcp in r:
                data = drain(tcp)
                if data is None:
                    break
                if data:
                    ws_send(tls, data, opcode=0x2)
            if tls in r:
                data = drain(tls)
                if data is None:
                    break
                for opcode, payload in parser.feed(data):
                    if opcode == 0x8:
                        return
                    elif opcode == 0x9:
                        ws_send(tls, payload, opcode=0xA)
                    elif opcode in (0x1, 0x2):
                        tcp.sendall(payload)
    finally:
        for s in (tcp, tls):
            try:
                s.close()
            except Exception:
                pass


def main():
    ls = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ls.bind(("127.0.0.1", LISTEN_PORT))
    ls.listen(5)
    print(f"[relay] listening on 127.0.0.1:{LISTEN_PORT}, upstream wss://{WORKER_HOST}/")
    print(f"[relay] then run: ssh -p {LISTEN_PORT} -i <你的私钥文件> relayuser@127.0.0.1")
    while True:
        conn, _ = ls.accept()
        print("[relay] incoming connection, dialing worker...", flush=True)
        try:
            tls = ws_connect()
        except Exception as e:
            print(f"[relay] dial failed: {e}", flush=True)
            conn.close()
            continue
        print("[relay] bridged.", flush=True)
        bridge(conn, tls)
        print("[relay] connection closed.", flush=True)


if __name__ == "__main__":
    main()

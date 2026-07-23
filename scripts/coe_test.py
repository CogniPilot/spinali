import socket, struct, sys, time

BOARD = "192.0.2.3"
PORT = 20000

def enc(canid, data, fd=False, brs=False, eff=False):
    cid = canid | (0x80000000 if eff else 0)
    if fd:
        return struct.pack(">IBB", cid, len(data) | 0x80, 0x01 if brs else 0) + data
    return struct.pack(">IB", cid, len(data)) + data

def pkt(seq, frames):
    return struct.pack(">BBBH", 2, 0, seq, len(frames)) + b"".join(frames)

def dec(p):
    ver, op, seq, cnt = struct.unpack(">BBBH", p[:5])
    out, off = [], 5
    for _ in range(cnt):
        cid, ln = struct.unpack(">IB", p[off:off+5]); off += 5
        fd = bool(ln & 0x80); ln &= 0x7F
        flags = 0
        if fd:
            flags = p[off]; off += 1
        data = p[off:off+ln]; off += ln
        out.append((cid & 0x1FFFFFFF, bool(cid & 0x80000000), fd, bool(flags & 1), data))
    return out

rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx.bind(("0.0.0.0", 20001))
rx.settimeout(3.0)
tx1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tx1.bind(("0.0.0.0", 20000))  # so replies for bus0 come back to us too
tx1.settimeout(3.0)

results = []

# Test A: classic frame, bus0 -> wire -> bus1 -> udp/20001
tx1.sendto(pkt(1, [enc(0x123, bytes([0xDE,0xAD,0xBE,0xEF]))]), (BOARD, PORT))
try:
    d, addr = rx.recvfrom(2048)
    f = dec(d)[0]
    ok = f[0] == 0x123 and f[4] == bytes([0xDE,0xAD,0xBE,0xEF]) and not f[2]
    results.append(("A classic std bus0->bus1", ok, f))
except socket.timeout:
    results.append(("A classic std bus0->bus1", False, "timeout"))

# Test B: extended-ID FD frame with BRS, 64 bytes, bus0 -> bus1
payload = bytes(range(64))
tx1.sendto(pkt(2, [enc(0x1ABCDE, payload, fd=True, brs=True, eff=True)]), (BOARD, PORT))
try:
    d, addr = rx.recvfrom(2048)
    f = dec(d)[0]
    ok = f[0] == 0x1ABCDE and f[1] and f[2] and f[3] and f[4] == payload
    results.append(("B FD/BRS ext 64B bus0->bus1", ok, (hex(f[0]), f[1], f[2], f[3], len(f[4]))))
except socket.timeout:
    results.append(("B FD/BRS ext 64B bus0->bus1", False, "timeout"))

# Test C: reverse direction, bus1 -> wire -> bus0 -> udp/20000
tx2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tx2.sendto(pkt(3, [enc(0x456, bytes([0x01,0x02,0x03]))]), (BOARD, 20001))
try:
    d, addr = tx1.recvfrom(2048)
    f = dec(d)[0]
    ok = f[0] == 0x456 and f[4] == bytes([0x01,0x02,0x03])
    results.append(("C classic bus1->bus0", ok, f))
except socket.timeout:
    results.append(("C classic bus1->bus0", False, "timeout"))

# Test D: burst - 50 frames, count arrival
sent = 50
for i in range(sent):
    tx1.sendto(pkt(10+i, [enc(0x300+i, struct.pack(">I", i))]), (BOARD, PORT))
    time.sleep(0.002)
got = 0
rx.settimeout(1.0)
try:
    while True:
        d, _ = rx.recvfrom(2048)
        got += len(dec(d))
except socket.timeout:
    pass
results.append((f"D burst {sent} frames", got == sent, f"received {got}/{sent}"))

for name, ok, detail in results:
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
sys.exit(0 if all(r[1] for r in results) else 1)

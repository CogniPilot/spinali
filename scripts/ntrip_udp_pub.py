#!/usr/bin/env python3
# Copyright (c) 2026 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0
"""NTRIP to spinali RTCM3 bridge.

Connects to an NTRIP caster, wraps each RTCM3 frame in a synapse_pb
Frame{rtcm3} and sends it over UDP to the rtk-gnss board's eth_rx port.

Requires: pip install protobuf grpcio-tools
The synapse_pb python bindings are generated on first run from the
workspace proto files (west workspace layout assumed).

Usage:
  ntrip_udp_pub.py --caster host:port --mount MOUNT --user U --password P \
                   [--target 192.0.2.3] [--port 4242] [--gga "$GPGGA,..."]
"""

import argparse
import base64
import importlib
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

PROTO_ROOT = Path(__file__).resolve().parents[2] / "modules/lib/synapse_pb/proto"


def load_synapse_pb():
    gen = Path(tempfile.gettempdir()) / "synapse_pb_py"
    gen.mkdir(exist_ok=True)
    protos = [str(p.relative_to(PROTO_ROOT)) for p in PROTO_ROOT.rglob("*.proto")]
    subprocess.run(
        [sys.executable, "-m", "grpc_tools.protoc", f"-I{PROTO_ROOT}",
         f"--python_out={gen}", *protos],
        check=True,
    )
    sys.path.insert(0, str(gen))
    frame = importlib.import_module("synapse_pb.frame_pb2")
    return frame


def rtcm3_frames(stream_read):
    """Incremental RTCM3 framer: yields complete frames from a byte source."""
    buf = b""
    while True:
        chunk = stream_read(4096)
        if not chunk:
            return
        buf += chunk
        while True:
            start = buf.find(b"\xd3")
            if start < 0:
                buf = b""
                break
            buf = buf[start:]
            if len(buf) < 3:
                break
            length = ((buf[1] & 0x03) << 8) | buf[2]
            total = 3 + length + 3
            if len(buf) < total:
                break
            yield buf[:total]
            buf = buf[total:]


def ntrip_connect(caster, mount, user, password, gga):
    host, port = caster.rsplit(":", 1)
    sock = socket.create_connection((host, int(port)), timeout=10)
    auth = base64.b64encode(f"{user}:{password}".encode()).decode()
    req = (
        f"GET /{mount} HTTP/1.1\r\nHost: {caster}\r\n"
        f"Ntrip-Version: Ntrip/2.0\r\nUser-Agent: NTRIP spinali/1.0\r\n"
        f"Authorization: Basic {auth}\r\nConnection: close\r\n\r\n"
    )
    sock.sendall(req.encode())
    hdr = b""
    while b"\r\n\r\n" not in hdr:
        hdr += sock.recv(1)
    status = hdr.split(b"\r\n", 1)[0].decode()
    if "200" not in status:
        raise RuntimeError(f"caster refused: {status}")
    if gga:
        sock.sendall((gga.strip() + "\r\n").encode())
    return sock


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--caster", required=True, help="host:port")
    ap.add_argument("--mount", required=True)
    ap.add_argument("--user", default="")
    ap.add_argument("--password", default="")
    ap.add_argument("--gga", default="", help="initial NMEA GGA sentence for VRS casters")
    ap.add_argument("--target", default="192.0.2.3")
    ap.add_argument("--port", type=int, default=4242)
    args = ap.parse_args()

    frame_pb2 = load_synapse_pb()
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        try:
            ntrip = ntrip_connect(args.caster, args.mount, args.user,
                                  args.password, args.gga)
            print(f"connected to {args.caster}/{args.mount}", flush=True)
            n = 0
            for rtcm in rtcm3_frames(ntrip.recv):
                if len(rtcm) > 1030:
                    print(f"drop oversize frame ({len(rtcm)} B)", flush=True)
                    continue
                frame = frame_pb2.Frame()
                frame.rtcm3.data = rtcm
                udp.sendto(frame.SerializeToString(), (args.target, args.port))
                n += 1
                if n % 50 == 0:
                    print(f"{n} frames forwarded", flush=True)
        except (OSError, RuntimeError) as e:
            print(f"reconnect after error: {e}", flush=True)
            time.sleep(5)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
MCUboot Device Information Tool

A script that uses the SMP (Simple Management Protocol) over UDP/Ethernet
to query device information from MCUboot-enabled devices.

This tool can retrieve:
- Bootloader information (MCUboot mode, version)
- OS/Application information (kernel, version, build info)
- Image state (slots, versions, hashes, boot status)
- Task statistics
- Memory pool statistics
- MCUmgr parameters

Usage:
    python mcuboot_device_info.py <device_ip> [--port PORT] [--timeout TIMEOUT]

Example:
    python mcuboot_device_info.py 192.0.2.1
    python mcuboot_device_info.py 192.0.2.1 --port 1337 --all
"""

import argparse
import re
import socket
import struct
import subprocess
import sys
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Optional

try:
    import cbor2
except ImportError:
    print("Error: cbor2 library is required. Install with: pip install cbor2")
    sys.exit(1)


# SMP Protocol Constants

class SMPOp(IntEnum):
    """SMP Operation codes"""
    READ = 0        # Read request
    READ_RSP = 1    # Read response
    WRITE = 2       # Write request
    WRITE_RSP = 3   # Write response


class SMPGroup(IntEnum):
    """SMP Management Group IDs"""
    OS = 0          # Default/OS management group
    IMG = 1         # Image management group
    STAT = 2        # Statistics management group
    SETTINGS = 3    # Settings management group
    LOG = 4         # Log management group (unused)
    CRASH = 5       # Crash/coredump management group (unused)
    SPLIT = 6       # Split image management (unused)
    RUN = 7         # Run-time tests (unused)
    FS = 8          # File system management group
    SHELL = 9       # Shell management group
    ZBASIC = 63     # Zephyr basic management group


class OSMgmtCmd(IntEnum):
    """OS Management Group Command IDs"""
    ECHO = 0
    CONSOLE = 1
    TASKSTAT = 2
    DATETIME = 4
    RESET = 5
    MCUMGR_PARAMS = 6
    INFO = 7
    BOOTLOADER_INFO = 8


class ImgMgmtCmd(IntEnum):
    """Image Management Group Command IDs"""
    STATE = 0
    UPLOAD = 1
    FILE = 2
    CORELIST = 3
    CORELOAD = 4
    ERASE = 5
    SLOT_INFO = 6


MCUBOOT_MODE_NAMES = {
    -1: "Unknown",
    0: "Single application",
    1: "Swap using scratch partition",
    2: "Overwrite (upgrade-only)",
    3: "Swap without scratch",
    4: "Direct XIP without revert",
    5: "Direct XIP with revert",
    6: "RAM loader",
    7: "Firmware loader",
    8: "RAM load with network core",
    9: "Swap using move",
}


# SMP Header format (8 bytes, big-endian)
# Byte 0: Res(3 bits) | Ver(2 bits) | OP(3 bits)
# Byte 1: Flags
# Bytes 2-3: Data Length
# Bytes 4-5: Group ID
# Byte 6: Sequence Number
# Byte 7: Command ID
SMP_HEADER_FORMAT = ">BBHHBB"
SMP_HEADER_SIZE = 8


@dataclass
class SMPHeader:
    """SMP packet header"""
    op: int
    version: int = 1
    flags: int = 0
    length: int = 0
    group: int = 0
    seq: int = 0
    cmd_id: int = 0

    def pack(self) -> bytes:
        """Pack header into bytes"""
        byte0 = ((self.version & 0x03) << 3) | (self.op & 0x07)
        return struct.pack(
            SMP_HEADER_FORMAT,
            byte0,
            self.flags,
            self.length,
            self.group,
            self.seq,
            self.cmd_id
        )

    @classmethod
    def unpack(cls, data: bytes) -> "SMPHeader":
        """Unpack header from bytes"""
        byte0, flags, length, group, seq, cmd_id = struct.unpack(
            SMP_HEADER_FORMAT, data[:SMP_HEADER_SIZE]
        )
        op = byte0 & 0x07
        version = (byte0 >> 3) & 0x03
        return cls(
            op=op,
            version=version,
            flags=flags,
            length=length,
            group=group,
            seq=seq,
            cmd_id=cmd_id
        )


class SMPClient:
    """SMP client for communicating with MCUboot devices over UDP"""
    DEFAULT_PORT = 1337
    DEFAULT_TIMEOUT = 5.0

    def __init__(self, host: str, port: int = DEFAULT_PORT, timeout: float = DEFAULT_TIMEOUT):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.seq = 0
        self.sock: Optional[socket.socket] = None

    def connect(self) -> None:
        """Create and configure UDP socket"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.timeout)

    def close(self) -> None:
        """Close UDP socket"""
        if self.sock:
            self.sock.close()
            self.sock = None

    def __enter__(self) -> "SMPClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    def _next_seq(self) -> int:
        """Get next sequence number"""
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFF
        return seq

    def _send_request(self, group: int, cmd_id: int, op: int = SMPOp.READ,
                      data: Optional[dict] = None) -> dict:
        """Send SMP request and receive response"""
        if self.sock is None:
            raise RuntimeError("Not connected")

        # Encode CBOR payload
        payload = cbor2.dumps(data if data else {})

        # Build header
        header = SMPHeader(
            op=op,
            version=1,  # SMP v2
            length=len(payload),
            group=group,
            seq=self._next_seq(),
            cmd_id=cmd_id
        )

        # Send packet
        packet = header.pack() + payload
        self.sock.sendto(packet, (self.host, self.port))

        # Receive response
        try:
            response_data, addr = self.sock.recvfrom(4096)
        except socket.timeout:
            raise TimeoutError(f"No response from {self.host}:{self.port}")

        # Parse response header
        if len(response_data) < SMP_HEADER_SIZE:
            raise ValueError("Response too short")

        resp_header = SMPHeader.unpack(response_data)

        # Check sequence number matches
        if resp_header.seq != header.seq:
            raise ValueError(f"Sequence mismatch: expected {header.seq}, got {resp_header.seq}")

        # Parse CBOR payload
        if resp_header.length > 0:
            cbor_data = response_data[SMP_HEADER_SIZE:SMP_HEADER_SIZE + resp_header.length]
            result = cbor2.loads(cbor_data)
        else:
            result = {}

        # Check for errors
        if "rc" in result and result["rc"] != 0:
            raise RuntimeError(f"SMP error: rc={result['rc']}")
        if "err" in result:
            err = result["err"]
            raise RuntimeError(f"SMP error: group={err.get('group')}, rc={err.get('rc')}")

        return result

    # OS Management Group Commands

    def echo(self, message: str = "hello") -> str:
        """Send echo request"""
        result = self._send_request(
            SMPGroup.OS, OSMgmtCmd.ECHO,
            op=SMPOp.WRITE,
            data={"d": message}
        )
        return result.get("r", "")

    def get_task_stats(self) -> dict:
        """Get task/thread statistics"""
        return self._send_request(SMPGroup.OS, OSMgmtCmd.TASKSTAT)

    def get_datetime(self) -> str:
        """Get device date/time"""
        result = self._send_request(SMPGroup.OS, OSMgmtCmd.DATETIME)
        return result.get("datetime", "")

    def get_mcumgr_params(self) -> dict:
        """Get MCUmgr parameters"""
        return self._send_request(SMPGroup.OS, OSMgmtCmd.MCUMGR_PARAMS)

    def get_os_info(self, format_str: str = "a") -> str:
        """
        Get OS/Application information

        Format specifiers:
        - s: Kernel name
        - n: Node name
        - r: Kernel release
        - v: Kernel version
        - b: Build date and time
        - m: Machine
        - p: Processor
        - i: Hardware platform
        - o: Operating system
        - a: All fields
        """
        result = self._send_request(
            SMPGroup.OS, OSMgmtCmd.INFO,
            data={"format": format_str}
        )
        return result.get("output", "")

    def get_bootloader_info(self, query: Optional[str] = None) -> dict:
        """
        Get bootloader information

        Args:
            query: Optional query string (e.g., "mode" for MCUboot mode)
        """
        data = {"query": query} if query else {}
        return self._send_request(SMPGroup.OS, OSMgmtCmd.BOOTLOADER_INFO, data=data)

    def reset(self, force: bool = False) -> dict:
        """Reset the device"""
        data = {"force": 1} if force else {}
        return self._send_request(
            SMPGroup.OS, OSMgmtCmd.RESET,
            op=SMPOp.WRITE,
            data=data
        )

    # Image Management Group Commands

    def get_image_state(self) -> dict:
        """Get state of all images/slots"""
        return self._send_request(SMPGroup.IMG, ImgMgmtCmd.STATE)

    def get_slot_info(self) -> dict:
        """Get slot information"""
        return self._send_request(SMPGroup.IMG, ImgMgmtCmd.SLOT_INFO)


def format_bytes(size: int) -> str:
    """Format byte size to human-readable string"""
    for unit in ['B', 'KB', 'MB', 'GB']:
        if size < 1024:
            return f"{size:.1f} {unit}"
        size /= 1024
    return f"{size:.1f} TB"


def get_mac_address(ip: str) -> Optional[str]:
    """Get MAC address from ARP table for the given IP"""
    try:
        # Try 'ip neigh' first (modern Linux)
        result = subprocess.run(
            ["ip", "neigh", "show", ip],
            capture_output=True, text=True, timeout=2
        )
        if result.returncode == 0 and result.stdout:
            match = re.search(r'lladdr\s+([0-9a-fA-F:]{17})', result.stdout)
            if match:
                return match.group(1).upper()

        # Try 'arp' command (older systems)
        result = subprocess.run(
            ["arp", "-n", ip],
            capture_output=True, text=True, timeout=2
        )
        if result.returncode == 0 and result.stdout:
            match = re.search(r'([0-9a-fA-F:]{17})', result.stdout)
            if match:
                return match.group(1).upper()
    except Exception:
        pass
    return None


def format_hash(hash_bytes: bytes) -> str:
    """Format hash bytes to hex string"""
    if isinstance(hash_bytes, bytes):
        return hash_bytes.hex()
    return str(hash_bytes)


def print_section(title: str) -> None:
    """Print section header"""
    print(f"\n{'=' * 60}")
    print(f" {title}")
    print('=' * 60)


def print_device_info(client: SMPClient, verbose: bool = False) -> None:
    """Print comprehensive device information"""

    # Echo test
    print_section("Connection Test")
    try:
        response = client.echo("MCUboot device info tool")
        print(f"Echo response: {response}")
        print("✓ Device is responding to SMP commands")
    except Exception as e:
        print(f"✗ Echo failed: {e}")
        return

    # Hardware Identification
    print_section("Hardware Identification")
    print(f"IP Address:  {client.host}")
    mac = get_mac_address(client.host)
    if mac:
        print(f"MAC Address: {mac}")
    else:
        print("MAC Address: (not available in ARP table)")

    # Query chip hardware ID using custom 'h' format character
    try:
        hwid_info = client.get_os_info("h")
        # Response format: "hwid:XXXXXXXXXXXX"
        if hwid_info.startswith("hwid:"):
            hwid = hwid_info[5:]  # Strip "hwid:" prefix
            print(f"Hardware ID: {hwid.upper()}")
        elif hwid_info:
            print(f"Hardware ID: {hwid_info}")
    except Exception:
        pass  # Hardware ID not available (custom hook not present)

    # Bootloader Information
    print_section("Bootloader Information")
    try:
        info = client.get_bootloader_info()
        bootloader = info.get("bootloader", "Unknown")
        print(f"Bootloader: {bootloader}")

        if bootloader == "MCUboot":
            # Query MCUboot mode
            try:
                mode_info = client.get_bootloader_info("mode")
                mode = mode_info.get("mode", -1)
                mode_name = MCUBOOT_MODE_NAMES.get(mode, f"Unknown ({mode})")
                print(f"Mode: {mode_name}")
                if mode_info.get("no-downgrade"):
                    print("Downgrade Prevention: Enabled")
            except Exception as e:
                print(f"  (Could not query mode: {e})")
    except Exception as e:
        print(f"Could not retrieve bootloader info: {e}")

    # OS/Application Information
    print_section("OS/Application Information")
    os_info = None
    # Retry up to 3 times for reliability
    for attempt in range(3):
        try:
            # Query all fields in a single request
            # Format order: s=kernel, n=node, r=release, v=version, b=build, m=machine, p=processor, i=platform, o=os
            os_info = client.get_os_info("snrvbmpio")
            break
        except Exception:
            if attempt == 2:
                pass  # Give up after 3 attempts

    if os_info:
        # Parse space-separated fields
        # Build date has 5 parts (e.g., "Wed Dec 17 17:05:03 2025")
        parts = os_info.split()

        if len(parts) >= 13:  # 4 fields + 5 build parts + 4 more fields
            print(f"{'Kernel:':<18}{parts[0]}")
            print(f"{'Application:':<18}{parts[1]}")
            print(f"{'Git Hash:':<18}{parts[2]}")
            print(f"{'Kernel Version:':<18}{parts[3]}")
            build_date = " ".join(parts[4:9])
            print(f"{'Architecture:':<18}{parts[9]}")
            print(f"{'Processor:':<18}{parts[10]}")
            print(f"{'Board:':<18}{parts[11]}")
            print(f"{'OS:':<18}{parts[12]}")
            print(f"{'Build Date:':<18}{build_date}")
        else:
            # Fallback: just print raw output
            print(os_info)
    else:
        print("Could not retrieve OS info (timeout)")

    # Image State
    print_section("Image State")
    try:
        state = client.get_image_state()
        images = state.get("images", [])

        if not images:
            print("No images found")

        for img in images:
            image_num = img.get("image", 0)
            slot = img.get("slot", 0)
            version = img.get("version", "unknown")

            print(f"\nImage {image_num}, Slot {slot}:")
            print(f"  Version: {version}")

            if "hash" in img:
                print(f"  Hash: {format_hash(img['hash'])}")

            flags = []
            if img.get("bootable"):
                flags.append("bootable")
            if img.get("pending"):
                flags.append("pending")
            if img.get("confirmed"):
                flags.append("confirmed")
            if img.get("active"):
                flags.append("active")
            if img.get("permanent"):
                flags.append("permanent")

            if flags:
                print(f"  Flags: {', '.join(flags)}")
    except Exception as e:
        print(f"Could not retrieve image state: {e}")

    # Slot Information
    print_section("Slot Information")
    try:
        slot_info = client.get_slot_info()
        images = slot_info.get("images", [])

        for img in images:
            image_num = img.get("image", 0)
            print(f"\nImage {image_num}:")

            if "max_image_size" in img:
                print(f"  Max image size: {format_bytes(img['max_image_size'])}")

            for slot in img.get("slots", []):
                slot_num = slot.get("slot", 0)
                size = slot.get("size", 0)
                print(f"  Slot {slot_num}: {format_bytes(size)}")
                if "upload_image_id" in slot:
                    print(f"    Upload ID: {slot['upload_image_id']}")
    except Exception as e:
        print(f"Could not retrieve slot info: {e}")

    # MCUmgr Parameters
    print_section("MCUmgr Parameters")
    try:
        params = client.get_mcumgr_params()
        buf_size = params.get("buf_size", 0)
        buf_count = params.get("buf_count", 0)
        print(f"Buffer size: {format_bytes(buf_size)}")
        print(f"Buffer count: {buf_count}")
    except Exception as e:
        print(f"Could not retrieve MCUmgr params: {e}")

    # Verbose: Task Statistics
    if verbose:
        print_section("Task Statistics")
        try:
            stats = client.get_task_stats()
            tasks = stats.get("tasks", {})

            if not tasks:
                print("No task statistics available")
            else:
                # Check if stack info is available (any non-zero values)
                has_stack_info = any(
                    info.get("stkuse", 0) != 0 or info.get("stksiz", 0) != 0
                    for info in tasks.values()
                )

                if has_stack_info:
                    print(f"{'Task':<24} {'Prio':>5} {'State':>6} {'Stack Use':>10} {'Stack Size':>10}")
                    print("-" * 59)
                    for name, info in tasks.items():
                        prio = info.get("prio", 0)
                        state = info.get("state", 0)
                        stkuse = info.get("stkuse", 0)
                        stksiz = info.get("stksiz", 0)
                        print(f"{name:<24} {prio:>5} {state:>6} {stkuse:>10} {stksiz:>10}")
                else:
                    print(f"{'Task':<24} {'Prio':>5} {'State':>6}")
                    print("-" * 39)
                    for name, info in tasks.items():
                        prio = info.get("prio", 0)
                        state = info.get("state", 0)
                        print(f"{name:<24} {prio:>5} {state:>6}")
        except Exception as e:
            print(f"Could not retrieve task stats: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Query device information from MCUboot-enabled devices over UDP/Ethernet",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s 192.0.2.1
  %(prog)s 192.0.2.1 --port 1337
  %(prog)s 192.0.2.1 --verbose
  %(prog)s 192.0.2.1 --echo "test message"
  %(prog)s 192.0.2.1 --reset
        """
    )

    parser.add_argument(
        "host",
        help="Device IP address or hostname"
    )
    parser.add_argument(
        "-p", "--port",
        type=int,
        default=SMPClient.DEFAULT_PORT,
        help=f"UDP port (default: {SMPClient.DEFAULT_PORT})"
    )
    parser.add_argument(
        "-t", "--timeout",
        type=float,
        default=SMPClient.DEFAULT_TIMEOUT,
        help=f"Timeout in seconds (default: {SMPClient.DEFAULT_TIMEOUT})"
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Show verbose output including task and memory statistics"
    )
    parser.add_argument(
        "--echo",
        metavar="MSG",
        help="Send echo message and exit"
    )
    parser.add_argument(
        "--reset",
        action="store_true",
        help="Reset the device after displaying info"
    )
    parser.add_argument(
        "--os-info",
        action="store_true",
        help="Only show OS/application info"
    )
    parser.add_argument(
        "--image-state",
        action="store_true",
        help="Only show image state"
    )
    parser.add_argument(
        "--bootloader-info",
        action="store_true",
        help="Only show bootloader info"
    )

    args = parser.parse_args()

    try:
        with SMPClient(args.host, args.port, args.timeout) as client:
            print(f"Connecting to {args.host}:{args.port}...")

            # Handle specific commands
            if args.echo:
                response = client.echo(args.echo)
                print(f"Echo response: {response}")
                return

            if args.os_info:
                print_section("OS/Application Information")
                print(client.get_os_info("a"))
                return

            if args.image_state:
                print_section("Image State")
                state = client.get_image_state()
                for img in state.get("images", []):
                    print(f"Image {img.get('image', 0)}, Slot {img.get('slot', 0)}:")
                    print(f"  Version: {img.get('version', 'unknown')}")
                    if "hash" in img:
                        print(f"  Hash: {format_hash(img['hash'])}")
                return

            if args.bootloader_info:
                print_section("Bootloader Information")
                info = client.get_bootloader_info()
                print(f"Bootloader: {info.get('bootloader', 'Unknown')}")
                if info.get("bootloader") == "MCUboot":
                    mode_info = client.get_bootloader_info("mode")
                    mode = mode_info.get("mode", -1)
                    print(f"Mode: {MCUBOOT_MODE_NAMES.get(mode, f'Unknown ({mode})')}")
                return

            # Full device info
            print_device_info(client, verbose=args.verbose)

            # Optional reset
            if args.reset:
                print_section("Device Reset")
                print("Sending reset command...")
                try:
                    client.reset()
                    print("Reset command sent successfully")
                except Exception as e:
                    print(f"Reset failed: {e}")

    except TimeoutError as e:
        print(f"Error: {e}")
        print("Make sure the device is powered on and network-connected.")
        sys.exit(1)
    except ConnectionRefusedError:
        print(f"Error: Connection refused by {args.host}:{args.port}")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

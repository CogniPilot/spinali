#!/usr/bin/env python3
"""
Subscribe to the synapse optical flow topics and print the decoded structs.
Requires: pip install eclipse-zenoh

Payloads are raw fixed-layout little-endian structs from the synapse_fbs
v0.7.0 catalog, tagged with the value contract asserted below. Anything whose
contract does not match is reported rather than decoded, because the payload
carries no self-description that would let us notice a layout change.

Start zenohd first:
  zenohd --listen tcp/192.0.2.2:7447
"""

import argparse
import struct
import time

import zenoh

FLOW_CONTRACT = (
    "application/x-synapse-struct;type=synapse.topic.OpticalFlowData;"
    "schema=sha256-128:9d4077b392c1e5de954843933aa812b3"
)
FLOW_VEL_CONTRACT = (
    "application/x-synapse-struct;type=synapse.topic.OpticalFlowVelocityData;"
    "schema=sha256-128:031ec34678c4f89aa98d1127f0b72c05"
)

# OpticalFlowData: 88-byte fixed struct, see fbs/optical_flow.fbs. All time
# fields are nanoseconds; the trailing seven ubytes carry quality (0-255),
# distance_quality (0-255), distance_pixel_ok (0-32), mode, flags, time_status,
# and id, followed by one pad byte.
FLOW_FORMAT = "<QQQ2f3f2fII5f7Bx"

# OpticalFlowVelocityData: 32-byte fixed struct. velocity is body FLU only;
# there is no ENU velocity and no gyro or flow-rate field here.
FLOW_VEL_FORMAT = "<Q2f3f4B"

assert struct.calcsize(FLOW_FORMAT) == 88
assert struct.calcsize(FLOW_VEL_FORMAT) == 32

FLOW_FIELDS = (
    "timestamp_ns",
    "timestamp_sample_ns",
    "distance_timestamp_ns",
    "flow_rad_x",
    "flow_rad_y",
    "delta_angle_x",
    "delta_angle_y",
    "delta_angle_z",
    "distance_m",
    "distance_spread_m",
    "integration_timespan_ns",
    "error_count",
    "max_flow_rate_rad_s",
    "min_ground_distance_m",
    "max_ground_distance_m",
    "field_of_view_rad",
    "temperature_c",
    "quality",
    "distance_quality",
    "distance_pixel_ok",
    "mode",
    "flags",
    "time_status",
    "id",
)

FLOW_VEL_FIELDS = (
    "timestamp_ns",
    "velocity_flu_x",
    "velocity_flu_y",
    "distance_m",
    "roll_rad",
    "pitch_rad",
    "quality",
    "flags",
    "time_status",
    "id",
)

# TimeStatus enum (types.fbs): discipline state behind the timestamps.
TIME_STATUS_NAMES = ("LocalFreerun", "GptpSynced", "GptpHoldover")


def time_status_name(value):
    if 0 <= value < len(TIME_STATUS_NAMES):
        return TIME_STATUS_NAMES[value]
    return f"?{value}"


def decode(payload, fmt, fields):
    if len(payload) != struct.calcsize(fmt):
        return None
    return dict(zip(fields, struct.unpack(fmt, payload)))


def encoding_of(sample):
    return str(sample.encoding) if sample.encoding is not None else ""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--namespace",
        default="",
        help="deployment namespace prefixing the bare catalog keys",
    )
    parser.add_argument(
        "--every", type=int, default=10, help="print every Nth sample per topic"
    )
    args = parser.parse_args()

    prefix = f"{args.namespace}/" if args.namespace else ""
    flow_key = f"{prefix}flow"
    flow_vel_key = f"{prefix}flow_vel"

    session = zenoh.open(zenoh.Config())

    counts = {"flow": 0, "flow_vel": 0}
    warned = set()

    def check_contract(sample, expected, name):
        received = encoding_of(sample)
        if received == expected:
            return True
        if name not in warned:
            warned.add(name)
            print(f"[{name}] contract mismatch\n  expected {expected}\n  received {received}")
        return False

    def on_flow(sample):
        if not check_contract(sample, FLOW_CONTRACT, "flow"):
            return
        data = decode(sample.payload.to_bytes(), FLOW_FORMAT, FLOW_FIELDS)
        if data is None:
            return
        counts["flow"] += 1
        if counts["flow"] % args.every:
            return
        print(
            f"[FLOW] t={data['timestamp_ns']} "
            f"flow=({data['flow_rad_x']:.5f}, {data['flow_rad_y']:.5f}) rad "
            f"da=({data['delta_angle_x']:.5f}, {data['delta_angle_y']:.5f}, "
            f"{data['delta_angle_z']:.5f}) rad "
            f"dist={data['distance_m']:.3f} m spread={data['distance_spread_m']:.3f} m "
            f"q={data['quality']}/255 dq={data['distance_quality']}/255 "
            f"pix_ok={data['distance_pixel_ok']} "
            f"dt={data['integration_timespan_ns']} ns "
            f"clk={time_status_name(data['time_status'])}"
        )

    def on_flow_vel(sample):
        if not check_contract(sample, FLOW_VEL_CONTRACT, "flow_vel"):
            return
        data = decode(sample.payload.to_bytes(), FLOW_VEL_FORMAT, FLOW_VEL_FIELDS)
        if data is None:
            return
        counts["flow_vel"] += 1
        if counts["flow_vel"] % args.every:
            return
        print(
            f"[VEL]  t={data['timestamp_ns']} "
            f"flu=({data['velocity_flu_x']:.3f}, {data['velocity_flu_y']:.3f}) m/s "
            f"dist={data['distance_m']:.3f} m "
            f"rp=({data['roll_rad']:+.3f}, {data['pitch_rad']:+.3f}) rad "
            f"q={data['quality']}/255 "
            f"clk={time_status_name(data['time_status'])}"
        )

    sub_flow = session.declare_subscriber(flow_key, on_flow)
    sub_flow_vel = session.declare_subscriber(flow_vel_key, on_flow_vel)

    print(f"Listening on {flow_key} and {flow_vel_key}")
    print("Press Ctrl+C to exit\n")

    try:
        while True:
            time.sleep(1)
            if counts["flow"] + counts["flow_vel"] == 0:
                print("  (nothing received yet - is zenohd running? is ethernet up?)")
    except KeyboardInterrupt:
        pass

    sub_flow.undeclare()
    sub_flow_vel.undeclare()
    session.close()
    print(f"\nReceived {counts['flow']} flow, {counts['flow_vel']} flow_vel messages")


if __name__ == "__main__":
    main()

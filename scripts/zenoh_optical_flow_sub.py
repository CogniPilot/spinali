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
    "schema=sha256-128:e880efa8756c9c6c7938d1fbf3b03fc8"
)
FLOW_VEL_CONTRACT = (
    "application/x-synapse-struct;type=synapse.topic.OpticalFlowVelocityData;"
    "schema=sha256-128:5505d8e94ad10e80320320e3658734fa"
)

# OpticalFlowData: timestamp_us@0 flow_rad@8 delta_angle_flu_rad@16 distance_m@28
# integration_timespan_us@32 max_flow_rate_rad_s@36 min_ground_distance_m@40
# max_ground_distance_m@44 quality_pct@48, 56 bytes total.
FLOW_FORMAT = "<Q2f3ffI3fB7x"

# OpticalFlowVelocityData: timestamp_us@0 velocity_flu_m_s@8 velocity_enu_m_s@16
# flow_rate_uncompensated_rad_s@24 flow_rate_compensated_rad_s@32
# gyro_flu_rad_s@40, 56 bytes total.
FLOW_VEL_FORMAT = "<Q2f2f2f2f3f4x"

assert struct.calcsize(FLOW_FORMAT) == 56
assert struct.calcsize(FLOW_VEL_FORMAT) == 56

FLOW_FIELDS = (
    "timestamp_us",
    "flow_rad_x",
    "flow_rad_y",
    "delta_angle_x",
    "delta_angle_y",
    "delta_angle_z",
    "distance_m",
    "integration_timespan_us",
    "max_flow_rate_rad_s",
    "min_ground_distance_m",
    "max_ground_distance_m",
    "quality_pct",
)

FLOW_VEL_FIELDS = (
    "timestamp_us",
    "velocity_flu_x",
    "velocity_flu_y",
    "velocity_enu_x",
    "velocity_enu_y",
    "flow_rate_uncompensated_x",
    "flow_rate_uncompensated_y",
    "flow_rate_compensated_x",
    "flow_rate_compensated_y",
    "gyro_flu_x",
    "gyro_flu_y",
    "gyro_flu_z",
)


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
            f"[FLOW] t={data['timestamp_us']} "
            f"flow=({data['flow_rad_x']:.5f}, {data['flow_rad_y']:.5f}) rad "
            f"da=({data['delta_angle_x']:.5f}, {data['delta_angle_y']:.5f}, "
            f"{data['delta_angle_z']:.5f}) rad "
            f"dist={data['distance_m']:.3f} m "
            f"q={data['quality_pct']}% "
            f"dt={data['integration_timespan_us']} us"
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
            f"[VEL]  t={data['timestamp_us']} "
            f"flu=({data['velocity_flu_x']:.3f}, {data['velocity_flu_y']:.3f}) m/s "
            f"comp=({data['flow_rate_compensated_x']:.4f}, "
            f"{data['flow_rate_compensated_y']:.4f}) rad/s "
            f"gyro=({data['gyro_flu_x']:.4f}, {data['gyro_flu_y']:.4f}, "
            f"{data['gyro_flu_z']:.4f}) rad/s"
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

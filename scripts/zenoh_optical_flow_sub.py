#!/usr/bin/env python3
"""
Subscribe to zenoh optical flow topics and print decoded FlatBuffer data.
Requires: pip install eclipse-zenoh flatbuffers

Start zenohd first:
  zenohd --listen udp/192.0.2.2:7447
"""

import struct
import sys
import time

import zenoh


def decode_vehicle_optical_flow(payload: bytes):
    """Decode VehicleOpticalFlow FlatBuffer table.

    Layout: [root_offset:4][vtable:8][table_body: soffset:4 + struct:56]
    Struct VehicleOpticalFlowData (56 bytes):
      int64  timestamp_us
      float  pixel_flow.x, pixel_flow.y
      float  delta_angle.x, delta_angle.y, delta_angle.z
      float  distance_m
      uint32 integration_timespan_us
      uint8  quality
      3 bytes padding
      float  max_flow_rate
      float  min_ground_distance
      float  max_ground_distance
    """
    if len(payload) < 72:
        return None

    # struct data starts at offset 16 (4 root + 8 vtable + 4 soffset)
    data = payload[16:]
    fields = struct.unpack_from("<q2f3ffIB3x3f", data)

    return {
        "timestamp_us": fields[0],
        "pixel_flow_x": fields[1],
        "pixel_flow_y": fields[2],
        "delta_angle_x": fields[3],
        "delta_angle_y": fields[4],
        "delta_angle_z": fields[5],
        "distance_m": fields[6],
        "integration_timespan_us": fields[7],
        "quality": fields[8],
        "max_flow_rate": fields[9],
        "min_ground_distance": fields[10],
        "max_ground_distance": fields[11],
    }


def decode_vehicle_optical_flow_vel(payload: bytes):
    """Decode VehicleOpticalFlowVel FlatBuffer table.

    Struct VehicleOpticalFlowVelData (56 bytes):
      int64  timestamp_us
      float  vel_body.x, vel_body.y
      float  vel_ne.x, vel_ne.y
      float  flow_rate_uncompensated.x, flow_rate_uncompensated.y
      float  flow_rate_compensated.x, flow_rate_compensated.y
      float  gyro_rate.x, gyro_rate.y, gyro_rate.z
    """
    if len(payload) < 72:
        return None

    data = payload[16:]
    fields = struct.unpack_from("<q2f2f2f2f3f", data)

    return {
        "timestamp_us": fields[0],
        "vel_body_x": fields[1],
        "vel_body_y": fields[2],
        "vel_ne_x": fields[3],
        "vel_ne_y": fields[4],
        "flow_uncompensated_x": fields[5],
        "flow_uncompensated_y": fields[6],
        "flow_compensated_x": fields[7],
        "flow_compensated_y": fields[8],
        "gyro_rate_x": fields[9],
        "gyro_rate_y": fields[10],
        "gyro_rate_z": fields[11],
    }


def main():
    conf = zenoh.Config()
    session = zenoh.open(conf)

    msg_count = {"flow": 0, "vel": 0}

    def on_flow(sample):
        data = decode_vehicle_optical_flow(sample.payload.to_bytes())
        if data:
            msg_count["flow"] += 1
            if msg_count["flow"] % 10 == 0:  # print every 10th
                print(
                    f"[FLOW] t={data['timestamp_us']} "
                    f"px=({data['pixel_flow_x']:.4f}, {data['pixel_flow_y']:.4f}) "
                    f"da=({data['delta_angle_x']:.4f}, {data['delta_angle_y']:.4f}, {data['delta_angle_z']:.4f}) "
                    f"dist={data['distance_m']:.3f}m "
                    f"q={data['quality']} "
                    f"dt={data['integration_timespan_us']}us"
                )

    def on_flow_vel(sample):
        data = decode_vehicle_optical_flow_vel(sample.payload.to_bytes())
        if data:
            msg_count["vel"] += 1
            if msg_count["vel"] % 10 == 0:
                print(
                    f"[VEL]  t={data['timestamp_us']} "
                    f"body=({data['vel_body_x']:.3f}, {data['vel_body_y']:.3f}) m/s "
                    f"comp=({data['flow_compensated_x']:.4f}, {data['flow_compensated_y']:.4f}) rad/s "
                    f"gyro=({data['gyro_rate_x']:.4f}, {data['gyro_rate_y']:.4f}, {data['gyro_rate_z']:.4f})"
                )

    sub_flow = session.declare_subscriber(
        "synapse/vehicle_optical_flow", on_flow
    )
    sub_vel = session.declare_subscriber(
        "synapse/vehicle_optical_flow_vel", on_flow_vel
    )

    print("Listening on synapse/vehicle_optical_flow and synapse/vehicle_optical_flow_vel...")
    print("Press Ctrl+C to exit\n")

    try:
        while True:
            time.sleep(1)
            total = msg_count["flow"] + msg_count["vel"]
            if total == 0:
                print("  (no messages received yet - is zenohd running? is ethernet connected?)")
    except KeyboardInterrupt:
        pass

    sub_flow.undeclare()
    sub_vel.undeclare()
    session.close()
    print(f"\nReceived {msg_count['flow']} flow, {msg_count['vel']} vel messages")


if __name__ == "__main__":
    main()

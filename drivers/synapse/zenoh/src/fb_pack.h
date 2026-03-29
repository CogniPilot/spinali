/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hand-written FlatBuffer packing for VehicleOpticalFlow tables.
 * Follows the cerebri/src/topic_flatbuffer.c pattern.
 */

#ifndef FB_PACK_H
#define FB_PACK_H

#include <stddef.h>
#include <stdint.h>

#include "synapse_optical_flow_reader.h"

/*
 * VehicleOpticalFlow table layout:
 *   [root_offset:4] [vtable:8] [table_body:4+56]
 *   vtable: [vtable_size:2][object_size:2][field0_offset:2][padding:2]
 *   table_body: [vtable_soffset:4][data:56]
 *   Total: 4 + 8 + 4 + 56 = 72 bytes
 *
 * VehicleOpticalFlowVel table has same structure (data is also 56 bytes).
 */
#define FB_VEHICLE_OPTICAL_FLOW_SIZE     72U
#define FB_VEHICLE_OPTICAL_FLOW_VEL_SIZE 72U

size_t fb_pack_vehicle_optical_flow(uint8_t *buf, size_t buf_size,
				    const synapse_topic_VehicleOpticalFlowData_t *data);

size_t fb_pack_vehicle_optical_flow_vel(uint8_t *buf, size_t buf_size,
					const synapse_topic_VehicleOpticalFlowVelData_t *data);

#endif /* FB_PACK_H */

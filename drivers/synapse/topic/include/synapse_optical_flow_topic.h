/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Optical-flow topic payloads and their Zenoh value contracts, mirroring the
 * synapse_fbs schema catalog that vehicle-side consumers are built against:
 *
 *   https://github.com/CogniPilot/synapse_fbs/releases/download/v0.7.0/
 *     synapse_fbs-c.tar.gz
 *   SHA256 cf552dde506372d746e0e96bbccfd245e9f15bf9841565a4b01a50e637b83cfe
 *
 * Source of truth for the layouts and hashes below is that release:
 * fbs/optical_flow.fbs and topics.json entries 9 (OpticalFlow, key "flow")
 * and 10 (OpticalFlowVelocity, key "flow_vel"). The struct declarations are
 * field-for-field identical to the generated headers in that archive, so this
 * file can be replaced by them verbatim once the generated catalog is
 * available to this build.
 *
 * Both payloads are fixed-layout little-endian structs published as raw bytes.
 * A receiver decides whether to trust those bytes by string-comparing the
 * value contract, which pins a schema hash and nothing about the actual field
 * placement, so the offset assertions at the bottom of this file are part of
 * the contract rather than a debugging aid: they are the only thing standing
 * between a mis-ordered field and a silently misdecoded message.
 */

#ifndef SYNAPSE_OPTICAL_FLOW_TOPIC_H
#define SYNAPSE_OPTICAL_FLOW_TOPIC_H

#include <stddef.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

struct synapse_types_Vec2f {
	float x;
	float y;
};
typedef struct synapse_types_Vec2f synapse_types_Vec2f_t;

struct synapse_types_Vec3f {
	float x;
	float y;
	float z;
};
typedef struct synapse_types_Vec3f synapse_types_Vec3f_t;

/*
 * Integrated optical-flow sample.
 *
 * flow_rad and delta_angle_flu_rad are both angles about the body FLU axes
 * over integration_timespan_us, so a consumer compensates flow for rotation
 * by subtracting one from the other componentwise.
 */
struct synapse_topic_OpticalFlowData {
	uint64_t timestamp_us;
	synapse_types_Vec2f_t flow_rad;
	synapse_types_Vec3f_t delta_angle_flu_rad;
	float distance_m;
	uint32_t integration_timespan_us;
	float max_flow_rate_rad_s;
	float min_ground_distance_m;
	float max_ground_distance_m;
	uint8_t quality_pct;
};
typedef struct synapse_topic_OpticalFlowData synapse_topic_OpticalFlowData_t;

/* Velocity estimate derived from optical flow. */
struct synapse_topic_OpticalFlowVelocityData {
	uint64_t timestamp_us;
	synapse_types_Vec2f_t velocity_flu_m_s;
	synapse_types_Vec2f_t velocity_enu_m_s;
	synapse_types_Vec2f_t flow_rate_uncompensated_rad_s;
	synapse_types_Vec2f_t flow_rate_compensated_rad_s;
	synapse_types_Vec3f_t gyro_flu_rad_s;
};
typedef struct synapse_topic_OpticalFlowVelocityData synapse_topic_OpticalFlowVelocityData_t;

/*
 * Catalog keys and value contracts. The contract string is compared verbatim
 * by the receiver, so the media type, wire type and schema hash must all match
 * the catalog entry exactly.
 */
#define SYNAPSE_TOPIC_OPTICAL_FLOW_KEY "flow"
#define SYNAPSE_TOPIC_OPTICAL_FLOW_CONTRACT                                                        \
	"application/x-synapse-struct;type=synapse.topic.OpticalFlowData;"                         \
	"schema=sha256-128:e880efa8756c9c6c7938d1fbf3b03fc8"

#define SYNAPSE_TOPIC_OPTICAL_FLOW_VELOCITY_KEY "flow_vel"
#define SYNAPSE_TOPIC_OPTICAL_FLOW_VELOCITY_CONTRACT                                               \
	"application/x-synapse-struct;type=synapse.topic.OpticalFlowVelocityData;"                 \
	"schema=sha256-128:5505d8e94ad10e80320320e3658734fa"

/* Layout assertions, one per field, against the v0.7.0 catalog. */
BUILD_ASSERT(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);

BUILD_ASSERT(sizeof(synapse_types_Vec2f_t) == 8U);
BUILD_ASSERT(sizeof(synapse_types_Vec3f_t) == 12U);

BUILD_ASSERT(sizeof(synapse_topic_OpticalFlowData_t) == 56U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, timestamp_us) == 0U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, flow_rad) == 8U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, delta_angle_flu_rad) == 16U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, distance_m) == 28U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, integration_timespan_us) == 32U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, max_flow_rate_rad_s) == 36U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, min_ground_distance_m) == 40U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, max_ground_distance_m) == 44U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, quality_pct) == 48U);

BUILD_ASSERT(sizeof(synapse_topic_OpticalFlowVelocityData_t) == 56U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowVelocityData_t, timestamp_us) == 0U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowVelocityData_t, velocity_flu_m_s) == 8U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowVelocityData_t, velocity_enu_m_s) == 16U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowVelocityData_t, flow_rate_uncompensated_rad_s) ==
	     24U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowVelocityData_t, flow_rate_compensated_rad_s) == 32U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowVelocityData_t, gyro_flu_rad_s) == 40U);

#endif /* SYNAPSE_OPTICAL_FLOW_TOPIC_H */

/* vi: ts=4 sw=4 et */

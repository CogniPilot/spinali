/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Optical-flow topic payloads and their Zenoh value contracts, mirroring the
 * synapse_fbs schema that vehicle-side consumers are built against.
 *
 * Source of truth for the layouts and hashes below is the synapse_fbs schema,
 * fbs/optical_flow.fbs and fbs/types.fbs, exposed by topic-catalog entries 9
 * (OpticalFlow, key "flow") and 10 (OpticalFlowVelocity, key "flow_vel"). The
 * struct declarations reproduce those fixed-layout little-endian structs field
 * for field, so this file can be replaced by the generated struct headers
 * verbatim once the generated C catalog is available to this build.
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

#ifndef SYNAPSE_TYPES_VEC3F
#define SYNAPSE_TYPES_VEC3F
struct synapse_types_Vec3f {
	float x;
	float y;
	float z;
};
typedef struct synapse_types_Vec3f synapse_types_Vec3f_t;
#endif /* SYNAPSE_TYPES_VEC3F */

/*
 * Discipline state of the clock behind the timestamp_ns fields; mirrors
 * synapse.types.TimeStatus. Stored in the one-byte time_status fields below.
 */
#ifndef SYNAPSE_TYPES_TIME_STATUS_ENUM
#define SYNAPSE_TYPES_TIME_STATUS_ENUM
enum synapse_types_TimeStatus {
	SYNAPSE_TYPES_TIME_STATUS_LOCAL_FREERUN = 0, /* Undisciplined monotonic boot clock. */
	SYNAPSE_TYPES_TIME_STATUS_GPTP_SYNCED = 1, /* Disciplined to the gPTP grandmaster. */
	SYNAPSE_TYPES_TIME_STATUS_GPTP_HOLDOVER = 2, /* Coasting after grandmaster loss. */
};
#endif /* SYNAPSE_TYPES_TIME_STATUS_ENUM */

/*
 * Automatic scene light mode reported by the flow sensor; mirrors
 * synapse.topic.FlowLightMode. Stored in the one-byte mode field below.
 */
enum synapse_topic_FlowLightMode {
	SYNAPSE_TOPIC_FLOW_LIGHT_MODE_UNKNOWN = 0, /* Mode not reported. */
	SYNAPSE_TOPIC_FLOW_LIGHT_MODE_BRIGHT = 1, /* Bright scene, full frame rate. */
	SYNAPSE_TOPIC_FLOW_LIGHT_MODE_LOW_LIGHT = 2, /* Low-light scene. */
	SYNAPSE_TOPIC_FLOW_LIGHT_MODE_SUPER_LOW_LIGHT = 3, /* Super-low-light, reduced frame rate. */
};

/* Bitmask stored in synapse_topic_OpticalFlowData.flags. */
enum synapse_topic_OpticalFlowFlags {
	SYNAPSE_TOPIC_OPTICAL_FLOW_FLAG_FLOW_VALID = 1U << 0,
	SYNAPSE_TOPIC_OPTICAL_FLOW_FLAG_DELTA_ANGLE_VALID = 1U << 1,
	SYNAPSE_TOPIC_OPTICAL_FLOW_FLAG_DISTANCE_VALID = 1U << 2,
	SYNAPSE_TOPIC_OPTICAL_FLOW_FLAG_DISTANCE_AMBIGUOUS = 1U << 3,
};

/* Bitmask stored in synapse_topic_OpticalFlowVelocityData.flags. */
enum synapse_topic_OpticalFlowVelocityFlags {
	SYNAPSE_TOPIC_OPTICAL_FLOW_VELOCITY_FLAG_VELOCITY_VALID = 1U << 0,
	SYNAPSE_TOPIC_OPTICAL_FLOW_VELOCITY_FLAG_TILT_COMPENSATED = 1U << 1,
	SYNAPSE_TOPIC_OPTICAL_FLOW_VELOCITY_FLAG_RANGE_TRUSTED = 1U << 2,
};

/*
 * Integrated optical-flow sample.
 *
 * flow_rad and delta_angle_flu_rad are both angles about the body FLU axes
 * over integration_timespan_ns, so a consumer compensates flow for rotation
 * by subtracting one from the other componentwise. All time fields are
 * nanoseconds and share one clock domain, selected by time_status.
 */
struct synapse_topic_OpticalFlowData {
	uint64_t timestamp_ns;
	uint64_t timestamp_sample_ns;
	uint64_t distance_timestamp_ns;
	synapse_types_Vec2f_t flow_rad;
	synapse_types_Vec3f_t delta_angle_flu_rad;
	float distance_m;
	float distance_spread_m;
	uint32_t integration_timespan_ns;
	uint32_t error_count;
	float max_flow_rate_rad_s;
	float min_ground_distance_m;
	float max_ground_distance_m;
	float field_of_view_rad;
	float temperature_c;
	uint8_t quality;
	uint8_t distance_quality;
	uint8_t distance_pixel_ok;
	uint8_t mode; /* enum synapse_topic_FlowLightMode */
	uint8_t flags; /* enum synapse_topic_OpticalFlowFlags */
	uint8_t time_status; /* enum synapse_types_TimeStatus */
	uint8_t id;
	/* One trailing pad byte to 88; supplied by the struct's 8-byte alignment. */
};
typedef struct synapse_topic_OpticalFlowData synapse_topic_OpticalFlowData_t;

/*
 * Velocity estimate derived from optical flow. velocity_flu_m_s is
 * tilt-compensated from accelerometer-derived roll_rad and pitch_rad; a
 * producer without that compensation clears the TiltCompensated flag and
 * leaves roll_rad and pitch_rad at zero.
 */
struct synapse_topic_OpticalFlowVelocityData {
	uint64_t timestamp_ns;
	synapse_types_Vec2f_t velocity_flu_m_s;
	float distance_m;
	float roll_rad;
	float pitch_rad;
	uint8_t quality;
	uint8_t flags; /* enum synapse_topic_OpticalFlowVelocityFlags */
	uint8_t time_status; /* enum synapse_types_TimeStatus */
	uint8_t id;
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
	"schema=sha256-128:9d4077b392c1e5de954843933aa812b3"

#define SYNAPSE_TOPIC_OPTICAL_FLOW_VELOCITY_KEY "flow_vel"
#define SYNAPSE_TOPIC_OPTICAL_FLOW_VELOCITY_CONTRACT                                               \
	"application/x-synapse-struct;type=synapse.topic.OpticalFlowVelocityData;"                 \
	"schema=sha256-128:031ec34678c4f89aa98d1127f0b72c05"

/* Layout assertions, one per field, against the synapse_fbs schema. */
BUILD_ASSERT(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);

BUILD_ASSERT(sizeof(synapse_types_Vec2f_t) == 8U);
BUILD_ASSERT(sizeof(synapse_types_Vec3f_t) == 12U);

BUILD_ASSERT(sizeof(synapse_topic_OpticalFlowData_t) == 88U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, timestamp_ns) == 0U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, timestamp_sample_ns) == 8U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, distance_timestamp_ns) == 16U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, flow_rad) == 24U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, delta_angle_flu_rad) == 32U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, distance_m) == 44U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, distance_spread_m) == 48U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, integration_timespan_ns) == 52U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, error_count) == 56U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, max_flow_rate_rad_s) == 60U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, min_ground_distance_m) == 64U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, max_ground_distance_m) == 68U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, field_of_view_rad) == 72U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, temperature_c) == 76U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, quality) == 80U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, distance_quality) == 81U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, distance_pixel_ok) == 82U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, mode) == 83U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, flags) == 84U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, time_status) == 85U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, id) == 86U);

BUILD_ASSERT(sizeof(synapse_topic_OpticalFlowVelocityData_t) == 32U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowVelocityData_t, timestamp_ns) == 0U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowVelocityData_t, velocity_flu_m_s) == 8U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowVelocityData_t, distance_m) == 16U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowVelocityData_t, roll_rad) == 20U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowVelocityData_t, pitch_rad) == 24U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowVelocityData_t, quality) == 28U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowVelocityData_t, flags) == 29U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowVelocityData_t, time_status) == 30U);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowVelocityData_t, id) == 31U);

#endif /* SYNAPSE_OPTICAL_FLOW_TOPIC_H */

/* vi: ts=4 sw=4 et */

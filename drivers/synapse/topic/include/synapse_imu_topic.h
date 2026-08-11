/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Inertial-sample topic payload and its Zenoh value contract, mirroring the
 * synapse_fbs schema that vehicle-side consumers are built against.
 *
 * Source of truth for the layout and hash below is the synapse_fbs schema,
 * fbs/sensors.fbs and fbs/types.fbs, exposed by topic-catalog entry 5
 * (InertialSample, key "imu"). The struct declaration reproduces that
 * fixed-layout little-endian struct field for field, so this file can be
 * replaced by the generated struct header verbatim once the generated C
 * catalog is available to this build.
 *
 * The payload is a fixed-layout little-endian struct published as raw bytes.
 * A receiver decides whether to trust those bytes by string-comparing the
 * value contract, which pins a schema hash and nothing about the actual field
 * placement, so the offset assertions at the bottom of this file are part of
 * the contract rather than a debugging aid: they are the only thing standing
 * between a mis-ordered field and a silently misdecoded message.
 */

#ifndef SYNAPSE_IMU_TOPIC_H
#define SYNAPSE_IMU_TOPIC_H

#include <stddef.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

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
 * Discipline state of the clock behind the timestamp_ns field; mirrors
 * synapse.types.TimeStatus. Stored in the one-byte time_status field below.
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
 * Bitmask stored in synapse_topic_InertialSampleData.flags; mirrors
 * synapse.topic.InertialFieldFlags. A set bit marks a group updated in this
 * sample, so sensors running at different rates advertise which fields are
 * fresh.
 */
enum synapse_topic_InertialFieldFlags {
	SYNAPSE_TOPIC_INERTIAL_FIELD_FLAG_ACCEL = 1U << 0, /* Accelerometer fields were updated. */
	SYNAPSE_TOPIC_INERTIAL_FIELD_FLAG_GYRO = 1U << 1, /* Gyroscope fields were updated. */
	SYNAPSE_TOPIC_INERTIAL_FIELD_FLAG_TEMPERATURE = 1U << 2, /* Temperature was updated. */
};

/*
 * Raw accelerometer, gyroscope, and temperature sample from an IMU sensor.
 *
 * Values are sensor-native measurements in the body FLU mounting frame after
 * driver axis alignment; no estimator products belong here. The magnetometer
 * and barometer are separate sensors carried on their own topics (MagneticField
 * and AirData). timestamp_ns shares one clock domain selected by time_status,
 * and sensors updating at different rates set the matching flags bits.
 */
struct synapse_topic_InertialSampleData {
	uint64_t timestamp_ns;
	synapse_types_Vec3f_t accel_flu_m_s2;
	synapse_types_Vec3f_t gyro_flu_rad_s;
	float temperature_c;
	uint8_t flags; /* enum synapse_topic_InertialFieldFlags */
	uint8_t time_status; /* enum synapse_types_TimeStatus */
	uint8_t id;
	/* One trailing pad byte to 40; supplied by the struct's 8-byte alignment. */
};
typedef struct synapse_topic_InertialSampleData synapse_topic_InertialSample_t;

/*
 * Catalog key and value contract. The contract string is compared verbatim by
 * the receiver, so the media type, wire type and schema hash must all match
 * the catalog entry exactly.
 */
#define SYNAPSE_TOPIC_IMU_KEY "imu"
#define SYNAPSE_TOPIC_IMU_CONTRACT                                                                 \
	"application/x-synapse-struct;type=synapse.topic.InertialSampleData;"                      \
	"schema=sha256-128:d84f22749fb0af66d6869d93ca460465"

/* Layout assertions, one per field, against the synapse_fbs schema. */
BUILD_ASSERT(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);

BUILD_ASSERT(sizeof(synapse_types_Vec3f_t) == 12U);

BUILD_ASSERT(sizeof(synapse_topic_InertialSample_t) == 40U);
BUILD_ASSERT(offsetof(synapse_topic_InertialSample_t, timestamp_ns) == 0U);
BUILD_ASSERT(offsetof(synapse_topic_InertialSample_t, accel_flu_m_s2) == 8U);
BUILD_ASSERT(offsetof(synapse_topic_InertialSample_t, gyro_flu_rad_s) == 20U);
BUILD_ASSERT(offsetof(synapse_topic_InertialSample_t, temperature_c) == 32U);
BUILD_ASSERT(offsetof(synapse_topic_InertialSample_t, flags) == 36U);
BUILD_ASSERT(offsetof(synapse_topic_InertialSample_t, time_status) == 37U);
BUILD_ASSERT(offsetof(synapse_topic_InertialSample_t, id) == 38U);

#endif /* SYNAPSE_IMU_TOPIC_H */

/* vi: ts=4 sw=4 et */

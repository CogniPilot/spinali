/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * GNSS-fix topic payload and its Zenoh value contract, mirroring the
 * synapse_fbs schema that vehicle-side consumers are built against.
 *
 * Source of truth for the layout and hash below is the synapse_fbs schema,
 * fbs/sensors.fbs and fbs/types.fbs, exposed by topic-catalog entry 8
 * (GnssFix, key "gnss"). The struct declaration reproduces that fixed-layout
 * little-endian struct field for field, so this file can be replaced by the
 * generated struct header verbatim once the generated C catalog is available
 * to this build.
 *
 * The payload is a fixed-layout little-endian struct published as raw bytes.
 * A receiver decides whether to trust those bytes by string-comparing the
 * value contract, which pins a schema hash and nothing about the actual field
 * placement, so the offset assertions at the bottom of this file are part of
 * the contract rather than a debugging aid: they are the only thing standing
 * between a mis-ordered field and a silently misdecoded message.
 */

#ifndef SYNAPSE_GNSS_TOPIC_H
#define SYNAPSE_GNSS_TOPIC_H

#include <stddef.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

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
 * Receiver fix quality; mirrors synapse.types.GnssFixType. Stored in the
 * one-byte fix_type field below.
 */
enum synapse_types_GnssFixType {
	SYNAPSE_TYPES_GNSS_FIX_TYPE_NO_FIX = 0, /* No position solution. */
	SYNAPSE_TYPES_GNSS_FIX_TYPE_TIME_ONLY = 1, /* Time-only solution without position. */
	SYNAPSE_TYPES_GNSS_FIX_TYPE_FIX_2D = 2, /* Two-dimensional position fix. */
	SYNAPSE_TYPES_GNSS_FIX_TYPE_FIX_3D = 3, /* Three-dimensional position fix. */
	SYNAPSE_TYPES_GNSS_FIX_TYPE_DGNSS = 4, /* Differentially corrected fix. */
	SYNAPSE_TYPES_GNSS_FIX_TYPE_RTK_FLOAT = 5, /* RTK solution with float ambiguities. */
	SYNAPSE_TYPES_GNSS_FIX_TYPE_RTK_FIXED = 6, /* RTK solution with fixed ambiguities. */
	SYNAPSE_TYPES_GNSS_FIX_TYPE_DEAD_RECKONING = 7, /* Dead-reckoning or combined solution. */
};

/*
 * Bitmask stored in synapse_topic_GnssFixData.flags; mirrors
 * synapse.topic.GnssFixFlags. Each bit gates an optional field whose value is
 * only meaningful when the matching bit is set.
 */
enum synapse_topic_GnssFixFlags {
	SYNAPSE_TOPIC_GNSS_FIX_FLAG_TIME_VALID = 1U << 0, /* time_unix_ns is valid. */
	SYNAPSE_TOPIC_GNSS_FIX_FLAG_COURSE_VALID = 1U << 1, /* course_over_ground_cdeg is valid. */
	SYNAPSE_TOPIC_GNSS_FIX_FLAG_YAW_VALID = 1U << 2, /* yaw_cdeg is valid. */
	SYNAPSE_TOPIC_GNSS_FIX_FLAG_VELOCITY_UP_VALID = 1U << 3, /* velocity_up_cm_s is valid. */
};

/*
 * Global navigation satellite receiver fix and motion estimate.
 *
 * This is a raw-layer topic: angles keep the receiver's native convention
 * (clockwise from true north) so logs stay faithful to the hardware, and the
 * estimator converts to REP-0103 conventions for estimate-layer topics.
 * timestamp_ns shares one clock domain selected by time_status; the accuracy
 * fields saturate at their maximum rather than truncation-casting.
 */
struct synapse_topic_GnssFixData {
	uint64_t timestamp_ns;
	uint64_t time_unix_ns;
	int32_t latitude_deg_e7;
	int32_t longitude_deg_e7;
	int32_t altitude_msl_mm;
	int32_t altitude_ellipsoid_mm;
	uint16_t horizontal_accuracy_mm;
	uint16_t vertical_accuracy_mm;
	uint16_t velocity_accuracy_mm_s;
	uint16_t yaw_accuracy_cdeg;
	uint16_t hdop_centi;
	uint16_t vdop_centi;
	uint16_t ground_speed_cm_s;
	uint16_t course_over_ground_cdeg;
	uint16_t yaw_cdeg;
	int16_t velocity_up_cm_s;
	uint8_t flags; /* enum synapse_topic_GnssFixFlags */
	uint8_t fix_type; /* enum synapse_types_GnssFixType */
	uint8_t satellites_used;
	uint8_t satellites_visible;
	uint8_t time_status; /* enum synapse_types_TimeStatus */
	uint8_t id;
	/* Six trailing pad bytes to 64; supplied by the struct's 8-byte alignment. */
};
typedef struct synapse_topic_GnssFixData synapse_topic_GnssFix_t;

/*
 * Catalog key and value contract. The contract string is compared verbatim by
 * the receiver, so the media type, wire type and schema hash must all match
 * the catalog entry exactly.
 */
#define SYNAPSE_TOPIC_GNSS_KEY "gnss"
#define SYNAPSE_TOPIC_GNSS_CONTRACT                                                                \
	"application/x-synapse-struct;type=synapse.topic.GnssFixData;"                             \
	"schema=sha256-128:d84f22749fb0af66d6869d93ca460465"

/* Layout assertions, one per field, against the synapse_fbs schema. */
BUILD_ASSERT(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);

BUILD_ASSERT(sizeof(synapse_topic_GnssFix_t) == 64U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, timestamp_ns) == 0U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, time_unix_ns) == 8U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, latitude_deg_e7) == 16U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, longitude_deg_e7) == 20U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, altitude_msl_mm) == 24U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, altitude_ellipsoid_mm) == 28U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, horizontal_accuracy_mm) == 32U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, vertical_accuracy_mm) == 34U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, velocity_accuracy_mm_s) == 36U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, yaw_accuracy_cdeg) == 38U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, hdop_centi) == 40U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, vdop_centi) == 42U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, ground_speed_cm_s) == 44U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, course_over_ground_cdeg) == 46U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, yaw_cdeg) == 48U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, velocity_up_cm_s) == 50U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, flags) == 52U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, fix_type) == 53U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, satellites_used) == 54U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, satellites_visible) == 55U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, time_status) == 56U);
BUILD_ASSERT(offsetof(synapse_topic_GnssFix_t, id) == 57U);

#endif /* SYNAPSE_GNSS_TOPIC_H */

/* vi: ts=4 sw=4 et */

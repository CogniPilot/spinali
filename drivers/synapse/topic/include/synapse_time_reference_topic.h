/*
 * Copyright CogniPilot Foundation 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Time-reference topic payload and its Zenoh value contract, mirroring the
 * synapse_fbs schema that vehicle-side consumers are built against.
 *
 * Source of truth for the layout and hash below is the synapse_fbs schema,
 * fbs/state.fbs and fbs/types.fbs, exposed by topic-catalog entry 2
 * (TimeReference, key "time"). The struct declaration reproduces that
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

#ifndef SYNAPSE_TIME_REFERENCE_TOPIC_H
#define SYNAPSE_TIME_REFERENCE_TOPIC_H

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
 * Per-node clock descriptor: relates the node's timestamp_ns domain to
 * absolute time and reports the discipline state. When time_status is
 * GptpSynced the message timestamps are already in the shared TAI domain and
 * time_tai_ns equals timestamp_ns. clock_class is the IEEE 802.1AS clockClass
 * of the served time (6 GNSS-locked, 7 holdover, 248 free-running);
 * utc_offset_s is TAI minus UTC in seconds, 0 when unknown. uncertainty_ns is
 * a one-sigma bound on this node's time relative to the grandmaster and stays
 * 0 when no holdover estimator computes it. A producer publishes immediately
 * on any time_status transition in addition to its low base rate.
 */
struct synapse_topic_TimeReferenceData {
	uint64_t timestamp_ns;
	uint64_t time_tai_ns;
	uint64_t time_unix_ns;
	uint32_t uncertainty_ns;
	int16_t utc_offset_s;
	uint8_t time_status; /* enum synapse_types_TimeStatus */
	uint8_t clock_class;
	uint8_t domain;
	uint8_t id;
	/* Six trailing pad bytes to 40; supplied by the struct's 8-byte alignment. */
};
typedef struct synapse_topic_TimeReferenceData synapse_topic_TimeReferenceData_t;

/*
 * Catalog key and value contract. The contract string is compared verbatim by
 * the receiver, so the media type, wire type and schema hash must all match
 * the catalog entry exactly.
 */
#define SYNAPSE_TOPIC_TIME_REFERENCE_KEY "time"
#define SYNAPSE_TOPIC_TIME_REFERENCE_CONTRACT                                                      \
	"application/x-synapse-struct;type=synapse.topic.TimeReferenceData;"                       \
	"schema=sha256-128:b4ed060b11764eccdb3d7784384ec1d2"

/* Layout assertions, one per field, against the synapse_fbs schema. */
BUILD_ASSERT(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);

BUILD_ASSERT(sizeof(synapse_topic_TimeReferenceData_t) == 40U);
BUILD_ASSERT(offsetof(synapse_topic_TimeReferenceData_t, timestamp_ns) == 0U);
BUILD_ASSERT(offsetof(synapse_topic_TimeReferenceData_t, time_tai_ns) == 8U);
BUILD_ASSERT(offsetof(synapse_topic_TimeReferenceData_t, time_unix_ns) == 16U);
BUILD_ASSERT(offsetof(synapse_topic_TimeReferenceData_t, uncertainty_ns) == 24U);
BUILD_ASSERT(offsetof(synapse_topic_TimeReferenceData_t, utc_offset_s) == 28U);
BUILD_ASSERT(offsetof(synapse_topic_TimeReferenceData_t, time_status) == 30U);
BUILD_ASSERT(offsetof(synapse_topic_TimeReferenceData_t, clock_class) == 31U);
BUILD_ASSERT(offsetof(synapse_topic_TimeReferenceData_t, domain) == 32U);
BUILD_ASSERT(offsetof(synapse_topic_TimeReferenceData_t, id) == 33U);

#endif /* SYNAPSE_TIME_REFERENCE_TOPIC_H */

/* vi: ts=4 sw=4 et */

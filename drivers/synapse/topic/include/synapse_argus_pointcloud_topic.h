/*
 * Copyright CogniPilot Foundation 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * AFBR-S50 (argus) per-pixel point-cloud topic payload and its Zenoh value
 * contract, mirroring the synapse_fbs schema that vehicle-side consumers are
 * built against.
 *
 * Source of truth for the layout and hash below is the synapse_fbs schema,
 * fbs/argus.fbs and fbs/types.fbs, exposed by topic-catalog entry 44
 * (ArgusPointCloud, key "argus"). The schema declares the three 32-pixel
 * groups as individually named px0..px31 fields; this mirror carries each
 * group as a fixed array of the same element type, which has the identical
 * layout, so a producer can index pixels directly.
 *
 * The payload is a fixed-layout little-endian struct published as raw bytes.
 * A receiver decides whether to trust those bytes by string-comparing the
 * value contract, which pins a schema hash and nothing about the actual field
 * placement, so the offset assertions at the bottom of this file are part of
 * the contract rather than a debugging aid: they are the only thing standing
 * between a mis-ordered field and a silently misdecoded message.
 */

#ifndef SYNAPSE_ARGUS_POINTCLOUD_TOPIC_H
#define SYNAPSE_ARGUS_POINTCLOUD_TOPIC_H

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
 * Per-pixel result status bits stored in the pixel_status and
 * reference_status bytes below; mirrors synapse.topic.ArgusPixelStatusFlags
 * and the sensor SDK pixel status word. Zero marks a fully valid pixel.
 */
enum synapse_topic_ArgusPixelStatusFlags {
	SYNAPSE_TOPIC_ARGUS_PX_FLAG_OFF = 1U << 0, /* Disabled by the pixel-enable mask. */
	SYNAPSE_TOPIC_ARGUS_PX_FLAG_SATURATED = 1U << 1, /* Saturated; range unreliable. */
	SYNAPSE_TOPIC_ARGUS_PX_FLAG_BINNING_EXCLUDED = 1U << 2, /* Excluded from the 1D bin. */
	SYNAPSE_TOPIC_ARGUS_PX_FLAG_INVALID = 1U << 3, /* Pixel result is invalid. */
	SYNAPSE_TOPIC_ARGUS_PX_FLAG_PREFILTERED = 1U << 4, /* Discarded by the prefilter. */
	SYNAPSE_TOPIC_ARGUS_PX_FLAG_NO_SIGNAL = 1U << 5, /* No detectable return signal. */
	SYNAPSE_TOPIC_ARGUS_PX_FLAG_OUT_OF_SYNC = 1U << 6, /* Lost measurement sync. */
	SYNAPSE_TOPIC_ARGUS_PX_FLAG_STALLED = 1U << 7, /* Pixel processing stalled. */
};

/* Per-pixel ranges for the 32-zone field, meters; schema fields px0_m..px31_m. */
struct synapse_topic_ArgusPixelRanges32 {
	float px_m[32];
};
typedef struct synapse_topic_ArgusPixelRanges32 synapse_topic_ArgusPixelRanges32_t;

/* Per-pixel amplitudes (the intensity channel), LSB; schema fields px0_lsb..px31_lsb. */
struct synapse_topic_ArgusPixelAmplitudes32 {
	float px_lsb[32];
};
typedef struct synapse_topic_ArgusPixelAmplitudes32 synapse_topic_ArgusPixelAmplitudes32_t;

/* Per-pixel ArgusPixelStatusFlags bytes; schema fields px0..px31. */
struct synapse_topic_ArgusPixelStatus32 {
	uint8_t px[32];
};
typedef struct synapse_topic_ArgusPixelStatus32 synapse_topic_ArgusPixelStatus32_t;

/*
 * Per-pixel time-of-flight point cloud from an AFBR-S50 (argus) ranger.
 *
 * Raw layer: ranges are sensor-native meters before mounting calibration, in
 * the sensor frame; a consumer projects pixels through the sensor optics
 * model and mounting pose. Amplitude is the per-pixel intensity. The
 * reference pixel is an internal calibration aid, not scene geometry.
 * timestamp_ns shares one clock domain selected by time_status. This topic is
 * bandwidth heavy and published only behind an opt-in configuration; the
 * fused scalar range rides on the optical flow topics.
 */
struct synapse_topic_ArgusPointCloudData {
	uint64_t timestamp_ns;
	synapse_topic_ArgusPixelRanges32_t range_m;
	synapse_topic_ArgusPixelAmplitudes32_t amplitude_lsb;
	uint32_t integration_time_us;
	uint32_t pixel_enabled_mask;
	uint32_t channel_enabled_mask;
	float reference_range_m;
	float reference_amplitude_lsb;
	int16_t device_status; /* zero is OK, negative is a device error code */
	synapse_topic_ArgusPixelStatus32_t pixel_status;
	uint8_t reference_status; /* enum synapse_topic_ArgusPixelStatusFlags */
	uint8_t signal_quality; /* percentage, 0: n/a, 1: bad, 100: good */
	uint8_t time_status; /* enum synapse_types_TimeStatus */
	uint8_t id;
	/* Six trailing pad bytes to 328; supplied by the struct's 8-byte alignment. */
};
typedef struct synapse_topic_ArgusPointCloudData synapse_topic_ArgusPointCloudData_t;

/*
 * Catalog key and value contract. The contract string is compared verbatim by
 * the receiver, so the media type, wire type and schema hash must all match
 * the catalog entry exactly.
 */
#define SYNAPSE_TOPIC_ARGUS_POINTCLOUD_KEY "argus"
#define SYNAPSE_TOPIC_ARGUS_POINTCLOUD_CONTRACT                                                    \
	"application/x-synapse-struct;type=synapse.topic.ArgusPointCloudData;"                     \
	"schema=sha256-128:762a6c05c34ec4872c37449dd671eb3d"

/* Layout assertions, one per field, against the synapse_fbs schema. */
BUILD_ASSERT(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);

BUILD_ASSERT(sizeof(synapse_topic_ArgusPixelRanges32_t) == 128U);
BUILD_ASSERT(sizeof(synapse_topic_ArgusPixelAmplitudes32_t) == 128U);
BUILD_ASSERT(sizeof(synapse_topic_ArgusPixelStatus32_t) == 32U);

BUILD_ASSERT(sizeof(synapse_topic_ArgusPointCloudData_t) == 328U);
BUILD_ASSERT(offsetof(synapse_topic_ArgusPointCloudData_t, timestamp_ns) == 0U);
BUILD_ASSERT(offsetof(synapse_topic_ArgusPointCloudData_t, range_m) == 8U);
BUILD_ASSERT(offsetof(synapse_topic_ArgusPointCloudData_t, amplitude_lsb) == 136U);
BUILD_ASSERT(offsetof(synapse_topic_ArgusPointCloudData_t, integration_time_us) == 264U);
BUILD_ASSERT(offsetof(synapse_topic_ArgusPointCloudData_t, pixel_enabled_mask) == 268U);
BUILD_ASSERT(offsetof(synapse_topic_ArgusPointCloudData_t, channel_enabled_mask) == 272U);
BUILD_ASSERT(offsetof(synapse_topic_ArgusPointCloudData_t, reference_range_m) == 276U);
BUILD_ASSERT(offsetof(synapse_topic_ArgusPointCloudData_t, reference_amplitude_lsb) == 280U);
BUILD_ASSERT(offsetof(synapse_topic_ArgusPointCloudData_t, device_status) == 284U);
BUILD_ASSERT(offsetof(synapse_topic_ArgusPointCloudData_t, pixel_status) == 286U);
BUILD_ASSERT(offsetof(synapse_topic_ArgusPointCloudData_t, reference_status) == 318U);
BUILD_ASSERT(offsetof(synapse_topic_ArgusPointCloudData_t, signal_quality) == 319U);
BUILD_ASSERT(offsetof(synapse_topic_ArgusPointCloudData_t, time_status) == 320U);
BUILD_ASSERT(offsetof(synapse_topic_ArgusPointCloudData_t, id) == 321U);

#endif /* SYNAPSE_ARGUS_POINTCLOUD_TOPIC_H */

/* vi: ts=4 sw=4 et */

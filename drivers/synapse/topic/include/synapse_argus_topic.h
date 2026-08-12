/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Broadcom AFBR-S50 (argus) time-of-flight result, carried as a plain
 * in-process topic between the streaming producer and the flow-fusion
 * consumer. It is not bridged to Zenoh, so it needs no value contract;
 * monotonic timestamp_ns is the boot clock shared with the other raw sensor
 * topics. Field layout mirrors the sensor SDK result the producer fills.
 */

#ifndef SYNAPSE_ARGUS_TOPIC_H
#define SYNAPSE_ARGUS_TOPIC_H

#include <stdbool.h>
#include <stdint.h>

/* Per-pixel status bitmask. */
enum synapse_topic_ArgusPixelStatus {
	SYNAPSE_TOPIC_ARGUS_PIXEL_STATUS_OK = 0,
	SYNAPSE_TOPIC_ARGUS_PIXEL_STATUS_OFF = 1,
	SYNAPSE_TOPIC_ARGUS_PIXEL_STATUS_SAT = 2,
	SYNAPSE_TOPIC_ARGUS_PIXEL_STATUS_BIN_EXCL = 4,
	SYNAPSE_TOPIC_ARGUS_PIXEL_STATUS_INVALID = 8,
	SYNAPSE_TOPIC_ARGUS_PIXEL_STATUS_PREFILTERED = 16,
	SYNAPSE_TOPIC_ARGUS_PIXEL_STATUS_NO_SIGNAL = 32,
	SYNAPSE_TOPIC_ARGUS_PIXEL_STATUS_OUT_OF_SYNC = 64,
	SYNAPSE_TOPIC_ARGUS_PIXEL_STATUS_STALLED = 128,
};

/* One-dimensional binned range/amplitude result. */
struct synapse_topic_ArgusResultsBin {
	float range; /* meters */
	float amplitude; /* LSB */
	uint8_t signal_quality; /* percentage, 0: n/a, 1: bad, 100: good */
};

/* Auxiliary ADC channels (only populated in verbose mode). */
struct synapse_topic_ArgusResultsAux {
	float vdd;
	float temp;
	float vsub;
	float vddl;
	float iapd;
	float bgl;
	float sna;
};

/* Measurement frame configuration (only populated in verbose mode). */
struct synapse_topic_ArgusMeasureFrame {
	uint32_t integration_time;
	uint32_t px_en_mask;
	uint32_t ch_en_mask;
	uint32_t state;
	float analog_integration_depth;
	uint16_t digital_integration_depth;
	float output_power;
	uint8_t bias_current;
	uint8_t pixel_gain;
	int8_t pll_offset;
	uint8_t pll_ctrl_cur;
};

/* Per-pixel range result. */
struct synapse_topic_ArgusPixel {
	float range; /* meters, raw before calibration */
	float phase; /* units of PI */
	float amplitude; /* LSB */
	uint8_t status; /* enum synapse_topic_ArgusPixelStatus */
	int8_t range_window;
	float amplitude_raw; /* LSB */
	float uncorrelated_noise;
	float snr;
};

struct synapse_topic_ArgusResults {
	uint64_t timestamp_ns;
	int16_t status;
	bool has_frame;
	struct synapse_topic_ArgusMeasureFrame frame;
	uint32_t pixel_count;
	struct synapse_topic_ArgusPixel pixel[32];
	bool has_pixel_ref;
	struct synapse_topic_ArgusPixel pixel_ref;
	bool has_bin;
	struct synapse_topic_ArgusResultsBin bin;
	bool has_auxiliary;
	struct synapse_topic_ArgusResultsAux auxiliary;
};
typedef struct synapse_topic_ArgusResults synapse_topic_ArgusResults_t;

#endif /* SYNAPSE_ARGUS_TOPIC_H */

/* vi: ts=4 sw=4 et */

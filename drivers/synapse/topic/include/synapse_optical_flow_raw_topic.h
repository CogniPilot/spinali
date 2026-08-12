/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Raw PixArt PAA3905 optical-flow sensor sample, carried as a plain in-process
 * topic between the streaming producer and the flow-fusion consumer. It is not
 * bridged to Zenoh, so it needs no value contract; monotonic timestamp_ns is
 * the boot clock shared with the other raw sensor topics.
 */

#ifndef SYNAPSE_OPTICAL_FLOW_RAW_TOPIC_H
#define SYNAPSE_OPTICAL_FLOW_RAW_TOPIC_H

#include <stdint.h>

/* Automatic scene light mode reported by the sensor. */
enum synapse_topic_PixartPaa3905Mode {
	SYNAPSE_TOPIC_PAA3905_MODE_BRIGHT = 0,
	SYNAPSE_TOPIC_PAA3905_MODE_LOW_LIGHT = 1,
	SYNAPSE_TOPIC_PAA3905_MODE_SUPER_LOW_LIGHT = 2,
};

struct synapse_topic_PixartPaa3905 {
	uint64_t timestamp_ns;
	int16_t delta_x;
	int16_t delta_y;
	int16_t delta_z;
	uint32_t shutter;
	uint8_t mode; /* enum synapse_topic_PixartPaa3905Mode */
	uint8_t squal;
	uint8_t challenge_condition;
	uint8_t motion; /* nonzero when the frame reported surface motion */
};
typedef struct synapse_topic_PixartPaa3905 synapse_topic_PixartPaa3905_t;

#endif /* SYNAPSE_OPTICAL_FLOW_RAW_TOPIC_H */

/* vi: ts=4 sw=4 et */

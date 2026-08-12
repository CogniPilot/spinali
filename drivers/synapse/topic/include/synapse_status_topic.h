/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Compact vehicle-status topic payload carried as a plain in-process struct.
 *
 * Sensor nodes consume only the operating mode (the inertial calibration gate);
 * the field is kept so a controller publishing status can drive that gate
 * without reintroducing a serialized control message on these nodes.
 */

#ifndef SYNAPSE_STATUS_TOPIC_H
#define SYNAPSE_STATUS_TOPIC_H

#include <stdint.h>

/*
 * Vehicle operating mode. Values match the historical wire enumeration so a
 * mode published by a controller keeps the same numeric meaning.
 */
enum synapse_topic_StatusMode {
	SYNAPSE_TOPIC_STATUS_MODE_UNKNOWN = 0,
	SYNAPSE_TOPIC_STATUS_MODE_CALIBRATION = 1,
	SYNAPSE_TOPIC_STATUS_MODE_ACTUATORS = 2,
	SYNAPSE_TOPIC_STATUS_MODE_ATTITUDE_RATE = 3,
	SYNAPSE_TOPIC_STATUS_MODE_ATTITUDE = 4,
	SYNAPSE_TOPIC_STATUS_MODE_ALTITUDE = 5,
	SYNAPSE_TOPIC_STATUS_MODE_POSITION = 6,
	SYNAPSE_TOPIC_STATUS_MODE_VELOCITY = 7,
	SYNAPSE_TOPIC_STATUS_MODE_ACCELERATION = 8,
	SYNAPSE_TOPIC_STATUS_MODE_BEZIER = 9,
};

struct synapse_topic_Status {
	uint8_t mode; /* enum synapse_topic_StatusMode */
};
typedef struct synapse_topic_Status synapse_topic_Status_t;

#endif /* SYNAPSE_STATUS_TOPIC_H */

/* vi: ts=4 sw=4 et */

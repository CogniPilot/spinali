/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * C port of PX4 RingBuffer for gyro and range sample buffering.
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* ----------------------------------------------------------------
 * Gyro ring buffer
 * ---------------------------------------------------------------- */
#define GYRO_BUFFER_SIZE 32

struct gyro_sample {
	uint64_t time_us;
	float data[3];
	float dt;
};

struct gyro_ring_buffer {
	struct gyro_sample buffer[GYRO_BUFFER_SIZE];
	uint8_t head;
	uint8_t tail;
	bool first_write;
};

static inline void gyro_ring_buffer_init(struct gyro_ring_buffer *rb)
{
	memset(rb, 0, sizeof(*rb));
	rb->first_write = true;
}

static inline void gyro_ring_buffer_push(struct gyro_ring_buffer *rb, const struct gyro_sample *s)
{
	uint8_t head_new = rb->head;

	if (!rb->first_write) {
		head_new = (rb->head + 1) % GYRO_BUFFER_SIZE;
	}

	rb->buffer[head_new] = *s;
	rb->head = head_new;

	if (rb->head == rb->tail && !rb->first_write) {
		rb->tail = (rb->tail + 1) % GYRO_BUFFER_SIZE;
	} else {
		rb->first_write = false;
	}
}

static inline bool gyro_ring_buffer_pop_oldest(struct gyro_ring_buffer *rb,
					       uint64_t timestamp_oldest,
					       uint64_t timestamp_newest,
					       struct gyro_sample *sample)
{
	if (timestamp_oldest >= timestamp_newest) {
		return false;
	}

	for (uint8_t i = 0; i < GYRO_BUFFER_SIZE; i++) {
		uint8_t index = (rb->tail + i) % GYRO_BUFFER_SIZE;

		if (rb->buffer[index].time_us >= timestamp_oldest &&
		    rb->buffer[index].time_us <= timestamp_newest) {
			*sample = rb->buffer[index];

			if (index == rb->head) {
				rb->tail = rb->head;
				rb->first_write = true;
			} else {
				rb->tail = (index + 1) % GYRO_BUFFER_SIZE;
			}

			memset(&rb->buffer[index], 0, sizeof(rb->buffer[index]));
			return true;
		}
	}

	return false;
}

/* ----------------------------------------------------------------
 * Range ring buffer
 * ---------------------------------------------------------------- */
#define RANGE_BUFFER_SIZE 5

struct range_sample {
	uint64_t time_us;
	float data;
	uint8_t signal_quality; /* fused AFBR bin signal quality, 0 to 100 */
	float spread_m; /* max minus min range over PIXEL_OK pixels, meters */
	uint8_t pixel_ok; /* count of PIXEL_OK pixels, 0 to 32 */
};

struct range_ring_buffer {
	struct range_sample buffer[RANGE_BUFFER_SIZE];
	uint8_t head;
	uint8_t tail;
	bool first_write;
};

static inline void range_ring_buffer_init(struct range_ring_buffer *rb)
{
	memset(rb, 0, sizeof(*rb));
	rb->first_write = true;
}

static inline void range_ring_buffer_push(struct range_ring_buffer *rb,
					  const struct range_sample *s)
{
	uint8_t head_new = rb->head;

	if (!rb->first_write) {
		head_new = (rb->head + 1) % RANGE_BUFFER_SIZE;
	}

	rb->buffer[head_new] = *s;
	rb->head = head_new;

	if (rb->head == rb->tail && !rb->first_write) {
		rb->tail = (rb->tail + 1) % RANGE_BUFFER_SIZE;
	} else {
		rb->first_write = false;
	}
}

static inline bool range_ring_buffer_peak_first_older_than(struct range_ring_buffer *rb,
							   uint64_t timestamp,
							   struct range_sample *sample)
{
	for (uint8_t i = 0; i < RANGE_BUFFER_SIZE; i++) {
		int index = (int)rb->head - (int)i;

		if (index < 0) {
			index += RANGE_BUFFER_SIZE;
		}

		if (timestamp >= rb->buffer[index].time_us &&
		    timestamp < rb->buffer[index].time_us + 100000ULL) {
			*sample = rb->buffer[index];
			return true;
		}

		if ((uint8_t)index == rb->tail) {
			return false;
		}
	}

	return false;
}

#endif /* RING_BUFFER_H */

/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <synapse_topic_list.h>

#include "synapse_shell_print.h"

int snprintf_cat(char *buf, int n, char const *fmt, ...)
{
	if (n <= 0) {
		return n;
	}
	int result = 0;
	va_list args;
	va_start(args, fmt);
	result = vsnprintf(buf, n, fmt, args);
	va_end(args);
	return result;
}

static int snprint_timestamp(char *buf, size_t n, synapse_pb_Timestamp *m)
{
	return snprintf_cat(buf, n, "stamp: %lld.%09d\n", m->seconds, m->nanos);
}

static int snprint_vector3(char *buf, size_t n, synapse_pb_Vector3 *m)
{
	return snprintf_cat(buf, n, "  x: %10.4f  y: %10.4f  z: %10.4f\n",
			    (double)m->x, (double)m->y, (double)m->z);
}

int snprint_argus(char *buf, size_t n, synapse_pb_ArgusResults *m)
{
	return snprintf_cat(buf, n, "stat: %d, range: %10.4f m, ampl: %10.4f LSB, qual: %d\n",
			    m->status, (double)m->bin.range, (double)m->bin.amplitude,
			    m->bin.signal_quality);
}

int snprint_imu(char *buf, size_t n, synapse_pb_Imu *m)
{
	size_t offset = 0;

	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}
	if (m->has_angular_velocity) {
		offset += snprintf_cat(buf + offset, n - offset, "angular velocity [rad/s]\n");
		offset += snprint_vector3(buf + offset, n - offset, &m->angular_velocity);
	}
	if (m->has_linear_acceleration) {
		offset += snprintf_cat(buf + offset, n - offset, "linear acceleration [m/s^2]\n");
		offset += snprint_vector3(buf + offset, n - offset, &m->linear_acceleration);
	}
	return offset;
}

int snprint_pixart_paa3905(char *buf, size_t n, synapse_pb_PixartPAA3905 *m)
{
	size_t offset = 0;

	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}
	offset += snprintf_cat(buf + offset, n - offset,
			       "motion: %d mode: %d delta_x: %d delta_y: %d delta_z: %d\n"
			       "challenge_condition: %d squal: %d shutter: %d\n",
			       m->motion, m->mode, m->delta_x, m->delta_y, m->delta_z,
			       m->challenge_condition, m->squal, m->shutter);
	return offset;
}

int snprint_status(char *buf, size_t n, synapse_pb_Status *m)
{
	size_t offset = 0;

	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}
	offset += snprintf_cat(buf + offset, n - offset,
			       "arming: %d\nmode: %d\nsafety: %d\nfuel: %d\n"
			       "power: %10.2fW\nmessage: %s\n",
			       m->arming, m->mode, m->safety, m->fuel,
			       (double)m->power, m->status_message);
	return offset;
}

/* vi: ts=4 sw=4 et */

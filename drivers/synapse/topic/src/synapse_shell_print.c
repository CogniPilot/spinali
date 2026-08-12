/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdarg.h>
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

int snprint_imu(char *buf, size_t n, synapse_topic_InertialSample_t *m)
{
	size_t offset = 0;
	offset += snprintf_cat(buf + offset, n - offset, "stamp: %llu ns\n",
			       (unsigned long long)m->timestamp_ns);
	offset += snprintf_cat(buf + offset, n - offset,
			       "accel [m/s^2]  x: %10.4f y: %10.4f z: %10.4f\n",
			       (double)m->accel_flu_m_s2.x, (double)m->accel_flu_m_s2.y,
			       (double)m->accel_flu_m_s2.z);
	offset += snprintf_cat(buf + offset, n - offset,
			       "gyro  [rad/s]  x: %10.4f y: %10.4f z: %10.4f\n",
			       (double)m->gyro_flu_rad_s.x, (double)m->gyro_flu_rad_s.y,
			       (double)m->gyro_flu_rad_s.z);
	offset += snprintf_cat(buf + offset, n - offset, "temp: %8.2f C  flags: 0x%02x\n",
			       (double)m->temperature_c, m->flags);
	return offset;
}

int snprint_mag(char *buf, size_t n, synapse_topic_MagneticField_t *m)
{
	size_t offset = 0;
	offset += snprintf_cat(buf + offset, n - offset, "stamp: %llu ns\n",
			       (unsigned long long)m->timestamp_ns);
	offset += snprintf_cat(buf + offset, n - offset,
			       "mag [T]  x: %13.7f y: %13.7f z: %13.7f\n",
			       (double)m->mag_flu_tesla.x, (double)m->mag_flu_tesla.y,
			       (double)m->mag_flu_tesla.z);
	offset += snprintf_cat(buf + offset, n - offset, "temp: %8.2f C\n",
			       (double)m->temperature_c);
	return offset;
}

int snprint_gnss(char *buf, size_t n, synapse_topic_GnssFix_t *m)
{
	size_t offset = 0;
	offset += snprintf_cat(buf + offset, n - offset, "stamp: %llu ns\n",
			       (unsigned long long)m->timestamp_ns);
	offset += snprintf_cat(buf + offset, n - offset, "lat: %13.7f deg\n",
			       (double)m->latitude_deg_e7 * 1e-7);
	offset += snprintf_cat(buf + offset, n - offset, "lon: %13.7f deg\n",
			       (double)m->longitude_deg_e7 * 1e-7);
	offset += snprintf_cat(buf + offset, n - offset, "alt: %10.3f m (msl)\n",
			       (double)m->altitude_msl_mm * 1e-3);
	offset += snprintf_cat(buf + offset, n - offset, "fix: %d  sats used: %d visible: %d\n",
			       m->fix_type, m->satellites_used, m->satellites_visible);
	return offset;
}

int snprint_argus(char *buf, size_t n, synapse_topic_ArgusResults_t *m)
{
	return snprintf_cat(
		buf, n, "stamp: %llu ns  stat: %d, range: %10.4f m, ampl: %10.4f LSB, qual: %d\n",
		(unsigned long long)m->timestamp_ns, m->status, (double)m->bin.range,
		(double)m->bin.amplitude, m->bin.signal_quality);
}

int snprint_pixart_paa3905(char *buf, size_t n, synapse_topic_PixartPaa3905_t *m)
{
	return snprintf_cat(
		buf, n,
		"stamp: %llu ns  motion: %d mode: %d delta_x: %d delta_y: %d delta_z: %d\n"
		"challenge_condition: %d squal: %d shutter: %u\n",
		(unsigned long long)m->timestamp_ns, m->motion, m->mode, m->delta_x, m->delta_y,
		m->delta_z, m->challenge_condition, m->squal, m->shutter);
}

int snprint_status(char *buf, size_t n, synapse_topic_Status_t *m)
{
	return snprintf_cat(buf, n, "mode: %d\n", m->mode);
}

// vi: ts=4 sw=4 et

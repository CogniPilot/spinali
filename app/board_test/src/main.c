/*
 * Copyright (c) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

#define ICM45686_HUB DT_ALIAS(hub)
#define ICM45686_OPTICAL DT_ALIAS(optical)

int main(void)
{
	// const struct device *const dev = DEVICE_DT_GET_ONE(invensense_icm45686);
	const struct device *const dev = DEVICE_DT_GET(ICM45686_HUB);
	const struct device *const dev2 = DEVICE_DT_GET(ICM45686_OPTICAL);
	struct sensor_value acc[3], gyr[3];
	struct sensor_value acc2[3], gyr2[3];

	if (!device_is_ready(dev)) {
		printf("Device %s is not ready\n", dev->name);
		return 0;
	}

	if (!device_is_ready(dev2))
	{
		printf("Device %s is not ready\n", dev2->name);
		return 0;
	}
	
	printf("Device %p name is %s\n", dev, dev->name);
	printf("Device %p name is %s\n", dev2, dev2->name);

	// int err = sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ,
	// 		SENSOR_ATTR_SAMPLING_FREQUENCY,
	// 		&sampling_freq);

	// printf("attritube set error: %d\n", err);

	while (1) {
		/* 10ms period, 100Hz Sampling frequency */
		k_sleep(K_MSEC(100));

		sensor_sample_fetch(dev);
		sensor_sample_fetch(dev2);

		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
		sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyr);

		sensor_channel_get(dev2, SENSOR_CHAN_ACCEL_XYZ, acc2);
		sensor_channel_get(dev2, SENSOR_CHAN_GYRO_XYZ, gyr2);

		printf("%s: \t AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; "
		       "GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d;\n",
			   dev->name,
		       acc[0].val1, acc[0].val2,
		       acc[1].val1, acc[1].val2,
		       acc[2].val1, acc[2].val2,
		       gyr[0].val1, gyr[0].val2,
		       gyr[1].val1, gyr[1].val2,
		       gyr[2].val1, gyr[2].val2);

		printf("%s: AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; "
		       "GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d;\n",
			   dev2->name,
		       acc2[0].val1, acc2[0].val2,
		       acc2[1].val1, acc2[1].val2,
		       acc2[2].val1, acc2[2].val2,
		       gyr2[0].val1, gyr2[0].val2,
		       gyr2[1].val1, gyr2[1].val2,
		       gyr2[2].val1, gyr2[2].val2);
	}
	return 0;
}

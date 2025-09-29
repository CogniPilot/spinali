// Optical flow test program

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>
#include <stdio.h>

ZTEST_DMEM const struct device *const dev_icm45686 = DEVICE_DT_GET(DT_ALIAS(icm45686));

// Accelerometer ICM45686
ZTEST(icm45686_test, test_icm_rdy)
{
    zassert_true(device_is_ready(dev_icm45686), "Device was not ready");
}

ZTEST(icm45686_test, test_get_icm_data)
{
    struct sensor_value acc[3], gyr[3], die_temp[1];
    int err;

    err = sensor_sample_fetch(dev_icm45686);
    zassert_equal(err, 0, "ICM45686 could not fetch data (err %d)", err);

    err = sensor_channel_get(dev_icm45686, SENSOR_CHAN_ACCEL_XYZ, acc);
    zassert_equal(err, 0, "ICM45686 could not get accel data (err %d)", err);

	err = sensor_channel_get(dev_icm45686, SENSOR_CHAN_GYRO_XYZ, gyr);
    zassert_equal(err, 0, "ICM45686 could not get gyro data (err %d)", err);

    err = sensor_channel_get(dev_icm45686, SENSOR_CHAN_DIE_TEMP, die_temp);
    zassert_equal(err, 0, "ICM45686 could not get die temperature data (err %d)", err);

    printf("%s: \t AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; "
		       "GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d; "
               "Temp: %d.%06d; \n",
			   dev_icm45686->name,
		       acc[0].val1, acc[0].val2,
		       acc[1].val1, acc[1].val2,
		       acc[2].val1, acc[2].val2,
		       gyr[0].val1, gyr[0].val2,
		       gyr[1].val1, gyr[1].val2,
		       gyr[2].val1, gyr[2].val2,
               die_temp[0].val1, die_temp[0].val2);
}

ZTEST_SUITE(icm45686_test, NULL, NULL, NULL, NULL, NULL);

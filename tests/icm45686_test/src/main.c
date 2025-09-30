// Optical flow test program

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>
#include <stdio.h>

#define ICM45686_COMPAT invensense_icm45686

/// @brief ICM45686 device ready test
/// @param dev Device struct
static void test_icm45686_rdy(const struct device *dev)
{
    printk("Running test for device: %s\n", dev->name);
    zassert_true(device_is_ready(dev), "Device is not ready");
}

/// @brief ICM45686 fetch and get accelerometer, gyro and die temperature data
/// @param dev Device struct
static void test_icm45686_data(const struct device *dev)
{
    struct sensor_value acc[3], gyr[3], die_temp[1];
    int err;

    err = sensor_sample_fetch(dev);
    zassert_equal(err, 0, "ICM45686 could not fetch data (err %d)", err);

    err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
    zassert_equal(err, 0, "ICM45686 could not get accel data (err %d)", err);

	err = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyr);
    zassert_equal(err, 0, "ICM45686 could not get gyro data (err %d)", err);

    err = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, die_temp);
    zassert_equal(err, 0, "ICM45686 could not get die temperature data (err %d)", err);

    printf("%s: \t AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; "
		       "GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d; "
               "Temp: %d.%06d; \n",
			   dev->name,
		       acc[0].val1, acc[0].val2,
		       acc[1].val1, acc[1].val2,
		       acc[2].val1, acc[2].val2,
		       gyr[0].val1, gyr[0].val2,
		       gyr[1].val1, gyr[1].val2,
		       gyr[2].val1, gyr[2].val2,
               die_temp[0].val1, die_temp[0].val2);
}

#define ICM45686_DEVICE_INIT(n) \
    const struct device *dev_##n = DEVICE_DT_GET(n); \
    ZTEST(icm45686_##n, test_icm45686_rdy) { test_icm45686_rdy(dev_##n); } \
    ZTEST(icm45686_##n, test_icm45686_data) { test_icm45686_data(dev_##n); } \
    ZTEST_SUITE(icm45686_##n, NULL, NULL, NULL, NULL, NULL); \

// Loop through all device ids that are compatible with the invensense ICM45686
DT_FOREACH_STATUS_OKAY(ICM45686_COMPAT, ICM45686_DEVICE_INIT);

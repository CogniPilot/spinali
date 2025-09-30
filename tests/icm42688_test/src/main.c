#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>

#define ICM42688_COMPAT invensense_icm42688

/// @brief ICM42688 device ready test
/// @param dev Device struct
static void test_icm42688_rdy(const struct device *dev)
{
    printk("Running test for device: %s\n", dev->name);
    zassert_true(device_is_ready(dev), "Device is not ready");
}

/// @brief ICM42688 fetch and get accelerometer, gyro and die temperature data
/// @param dev Device struct
static void test_icm42688_data(const struct device *dev)
{
    struct sensor_value acc[3], gyr[3], die_temp[1];
    int err;

    err = sensor_sample_fetch(dev);
    zassert_equal(err, 0, "ICM42688 could not fetch data (err %d)", err);

    err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
    zassert_equal(err, 0, "ICM42688 could not get accel data (err %d)", err);

	err = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyr);
    zassert_equal(err, 0, "ICM42688 could not get gyro data (err %d)", err);

    err = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, die_temp);
    zassert_equal(err, 0, "ICM42688 could not get die temperature data (err %d)", err);

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

// Macro that allows test suites with tests to be created for each ICM42688 device within a board
#define ICM42688_DEVICE_INIT(n) \
    const struct device *dev_##n = DEVICE_DT_GET(n); \
    ZTEST(icm42688_##n, test_icm42688_rdy) { test_icm42688_rdy(dev_##n); } \
    ZTEST(icm42688_##n, test_icm42688_data) { test_icm42688_data(dev_##n); } \
    ZTEST_SUITE(icm42688_##n, NULL, NULL, NULL, NULL, NULL); \

// Loop through all device ids that are compatible with the invensense ICM42688
DT_FOREACH_STATUS_OKAY(ICM42688_COMPAT, ICM42688_DEVICE_INIT);

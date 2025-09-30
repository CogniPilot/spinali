#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>

#define BMP581_COMPAT bosch_bmp581

/// @brief BMP581 device ready test
/// @param dev Device struct
static void test_bmp581_rdy(const struct device *dev)
{
    printk("Running test for device: %s\n", dev->name);
    zassert_true(device_is_ready(dev), "Device is not ready");
}

/// @brief BMP581 fetch and get accelerometer, gyro and die temperature data
/// @param dev Device struct
static void test_bmp581_data(const struct device *dev)
{
    struct sensor_value baro_press[1], baro_temp[1];
    int err;

    err = sensor_sample_fetch(dev);
    zassert_equal(err, 0, "Barometer could not fetch data (err %d)", err);

    err = sensor_channel_get(dev, SENSOR_CHAN_PRESS, baro_press);
    zassert_equal(err, 0, "Barometer could not get pressure data (err %d)", err);

    err = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, baro_temp);
    zassert_equal(err, 0, "Barometer could not get ambient temperature data (err %d)", err);

    printf("%s: \t Pressure: %d.%06d; Ta: %d.%06d;\n",
			   dev->name,
		       baro_press[0].val1, baro_press[0].val2,
		       baro_temp[0].val1, baro_temp[0].val2);
}

// Macro that allows test suites with tests to be created for each BMP581 device within a board
#define BMP581_DEVICE_INIT(n) \
    const struct device *dev_##n = DEVICE_DT_GET(n); \
    ZTEST(bmp581_##n, test_bmp581_rdy) { test_bmp581_rdy(dev_##n); } \
    ZTEST(bmp581_##n, test_bmp581_data) { test_bmp581_data(dev_##n); } \
    ZTEST_SUITE(bmp581_##n, NULL, NULL, NULL, NULL, NULL); \

// Loop through all device ids that are compatible with the invensense BMP581
DT_FOREACH_STATUS_OKAY(BMP581_COMPAT, BMP581_DEVICE_INIT);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>

#define BMM350_COMPAT bosch_bmm350

/// @brief BMM350 device ready test
/// @param dev Device struct
static void test_bmm350_rdy(const struct device *dev)
{
    printk("Running test for device: %s\n", dev->name);
    zassert_true(device_is_ready(dev), "Device is not ready");
}

/// @brief BMM350 fetch and get magnetometric data
/// @param dev Device struct
static void test_bmm350_data(const struct device *dev)
{
    struct sensor_value mag_xyz[3];
    int err;

    err = sensor_sample_fetch(dev);
    zassert_equal(err, 0, "Magnetometer could not fetch data (err %d)", err);

    err = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, mag_xyz);
    zassert_equal(err, 0, "Magnetometer could not get XYZ data (err %d)", err);

    printf("%s: \t X: %d.%06d; Y: %d.%06d; Z: %d.%06d;\n",
			   dev->name,
		       mag_xyz[0].val1, mag_xyz[0].val2,
		       mag_xyz[1].val1, mag_xyz[1].val2,
		       mag_xyz[2].val1, mag_xyz[2].val2);
}

// Macro that allows test suites with tests to be created for each BMM350 device within a board
#define BMM350_DEVICE_INIT(n) \
    const struct device *dev_##n = DEVICE_DT_GET(n); \
    ZTEST(bmm350_##n, test_bmm350_rdy) { test_bmm350_rdy(dev_##n); } \
    ZTEST(bmm350_##n, test_bmm350_data) { test_bmm350_data(dev_##n); } \
    ZTEST_SUITE(bmm350_##n, NULL, NULL, NULL, NULL, NULL); \

// Loop through all device ids that are compatible with the bosch BMM350
DT_FOREACH_STATUS_OKAY(BMM350_COMPAT, BMM350_DEVICE_INIT);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>

#define PAA3905_COMPAT pixart_paa3905

/// @brief PAA3905 device ready test
/// @param dev Device struct
static void test_paa3905_rdy(const struct device *dev)
{
    printk("Running test for device: %s\n", dev->name);
    zassert_true(device_is_ready(dev), "Device is not ready");
}

/// @brief PAA3905 fetch and get accelerometer, gyro and die temperature data
/// @param dev Device struct
static void test_paa3905_data(const struct device *dev)
{
    ztest_test_skip();
}

// Macro that allows test suites with tests to be created for each PAA3905 device within a board
#define PAA3905_DEVICE_INIT(n) \
    const struct device *dev_##n = DEVICE_DT_GET(n); \
    ZTEST(paa3905_##n, test_paa3905_rdy) { test_paa3905_rdy(dev_##n); } \
    ZTEST(paa3905_##n, test_paa3905_data) { test_paa3905_data(dev_##n); } \
    ZTEST_SUITE(paa3905_##n, NULL, NULL, NULL, NULL, NULL); \

// Loop through all device ids that are compatible with the invensense PAA3905
DT_FOREACH_STATUS_OKAY(PAA3905_COMPAT, PAA3905_DEVICE_INIT);

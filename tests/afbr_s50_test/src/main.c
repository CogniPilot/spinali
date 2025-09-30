#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>

#define AFBR_S50_COMPAT brcm_afbr_s50

/// @brief AFBR_S50 device ready test
/// @param dev Device struct
static void test_afbr_s50_rdy(const struct device *dev)
{
    printk("Running test for device: %s\n", dev->name);
    zassert_true(device_is_ready(dev), "Device is not ready");
}

/// @brief AFBR_S50 fetch and get accelerometer, gyro and die temperature data
/// @param dev Device struct
static void test_afbr_s50_data(const struct device *dev)
{
    ztest_test_skip();
}

// Macro that allows test suites with tests to be created for each AFBR_S50 device within a board
#define AFBR_S50_DEVICE_INIT(n) \
    const struct device *dev_##n = DEVICE_DT_GET(n); \
    ZTEST(afbr_s50_##n, test_afbr_s50_rdy) { test_afbr_s50_rdy(dev_##n); } \
    ZTEST(afbr_s50_##n, test_afbr_s50_data) { test_afbr_s50_data(dev_##n); } \
    ZTEST_SUITE(afbr_s50_##n, NULL, NULL, NULL, NULL, NULL); \

// Loop through all device ids that are compatible with the invensense AFBR_S50
DT_FOREACH_STATUS_OKAY(AFBR_S50_COMPAT, AFBR_S50_DEVICE_INIT);

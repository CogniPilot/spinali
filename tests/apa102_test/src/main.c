#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/ztest.h>

#define APA102_COMPAT apa_apa102

#define GET_STRIP_NUM_PIXELS(n) \
    (DT_NODE_HAS_PROP(n, chain_length)) ? (const int STRIP_NUM_PIXELS = DT_PROP(n, chain_length)) : (const int STRIP_NUM_PIXELS = 0); \

// #if DT_NODE_HAS_PROP(DT_ALIAS(led_strip), chain_length)
// #define STRIP_NUM_PIXELS	DT_PROP(DT_ALIAS(led_strip), chain_length)
// #else
// #error Unable to determine length of LED strip
// #endif

#define DELAY_TIME K_MSEC(100)

#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

#define STRIP_NUM_PIXELS 1

static const struct led_rgb colors[] = {
	RGB(16, 0x00, 0x00), /* red */
	RGB(0x00, 16, 0x00), /* green */
	RGB(0x00, 0x00, 16), /* blue */
};

static struct led_rgb pixels[STRIP_NUM_PIXELS];

/// @brief APA102 device ready test
/// @param dev Device struct
static void test_apa102_rdy(const struct device *dev)
{
    printk("Running test for device: %s\n", dev->name);
    zassert_true(device_is_ready(dev), "Device is not ready");
}

/// @brief Test APA102 RGB colours
/// @param dev Device struct
static void test_apa102_rgb(const struct device *dev)
{
    int err;

    // Loop through RGB colors with delay
    for (size_t color = 0; color < ARRAY_SIZE(colors); color++)
    {
        for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) 
        {
            memset(&pixels, 0x00, sizeof(pixels));
            memcpy(&pixels[cursor], &colors[color], sizeof(struct led_rgb));

            err = led_strip_update_rgb(dev, pixels, STRIP_NUM_PIXELS);
            zassert_equal(err, 0, "Couldn't update led strip: %d", err);

            k_sleep(DELAY_TIME);
        }
    }

    // Turn off led strip
    memset(&pixels, 0x00, sizeof(pixels));

    err = led_strip_update_rgb(dev, pixels, STRIP_NUM_PIXELS);
    zassert_equal(err, 0, "Couldn't update led strip: %d", err);
}

// Macro that allows test suites with tests to be created for each BMM350 device within a board
#define APA102_DEVICE_INIT(n) \
    const struct device *dev_##n = DEVICE_DT_GET(n); \
    ZTEST(apa102_##n, test_apa102_rdy) { test_apa102_rdy(dev_##n); } \
    ZTEST(apa102_##n, test_apa102_rgb) { test_apa102_rgb(dev_##n); } \
    ZTEST_SUITE(apa102_##n, NULL, NULL, NULL, NULL, NULL); \

// Loop through all device ids that are compatible with the bosch BMM350
DT_FOREACH_STATUS_OKAY(APA102_COMPAT, APA102_DEVICE_INIT);

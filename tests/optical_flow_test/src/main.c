// Optical flow test program

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/can.h>
#include <zephyr/ztest.h>
#include <stdio.h>

// #include <zephyr/drivers/sensor/tdk/icm4268x/icm4268x_emul.h>
// #include <zephyr/drivers/sensor/tdk/icm4268x/icm4268x_reg.h>

ZTEST_DMEM const struct device *const dev_icm45686_base = DEVICE_DT_GET(DT_ALIAS(icm45686base));
ZTEST_DMEM const struct device *const dev_bmp581        = DEVICE_DT_GET(DT_NODELABEL(bmp581_0));
ZTEST_DMEM const struct device *const dev_bmm350        = DEVICE_DT_GET(DT_NODELABEL(bmm350_0));
ZTEST_DMEM const struct device *const dev_apa102        = DEVICE_DT_GET(DT_NODELABEL(led_strip));

ZTEST_DMEM const struct device *const dev_canbus        = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
// ZTEST_DMEM const struct device *const dev_can1          = DEVICE_DT_GET(DT_NOTELABEL(can_phy1));

// ZTEST_DMEM const struct device *const dev_enet          = DEVICE_DT_GET(DT_CHOSEN(phy));

// #ifdef CONFIG_OFLO_ADDON
ZTEST_DMEM const struct device *const dev_icm45686_opt  = DEVICE_DT_GET(DT_ALIAS(icm45686opt));
ZTEST_DMEM const struct device *const dev_icm42688      = DEVICE_DT_GET(DT_ALIAS(icm42688));
ZTEST_DMEM const struct device *const dev_paa3905       = DEVICE_DT_GET(DT_ALIAS(paa3905));
ZTEST_DMEM const struct device *const dev_afbr          = DEVICE_DT_GET(DT_ALIAS(afbr));
// #endif

#if DT_NODE_HAS_PROP(DT_ALIAS(led_strip), chain_length)
#define STRIP_NUM_PIXELS	DT_PROP(DT_ALIAS(led_strip), chain_length)
#else
#error Unable to determine length of LED strip
#endif

#define DELAY_TIME K_MSEC(100)

#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

static const struct led_rgb colors[] = {
	RGB(16, 0x00, 0x00), /* red */
	RGB(0x00, 16, 0x00), /* green */
	RGB(0x00, 0x00, 16), /* blue */
};

static struct led_rgb pixels[STRIP_NUM_PIXELS];

// Accelerometer base
ZTEST(icm45686_base, test_icm_rdy)
{
    zassert_true(device_is_ready(dev_icm45686_base), "Device was not ready");
}

ZTEST(icm45686_base, test_get_icm_data)
{
    struct sensor_value acc[3], gyr[3], die_temp[1];
    int err;

    err = sensor_sample_fetch(dev_icm45686_base);
    zassert_equal(err, 0, "ICM45686 could not fetch data (err %d)", err);

    err = sensor_channel_get(dev_icm45686_base, SENSOR_CHAN_ACCEL_XYZ, acc);
    zassert_equal(err, 0, "ICM45686 could not get accel data (err %d)", err);

	err = sensor_channel_get(dev_icm45686_base, SENSOR_CHAN_GYRO_XYZ, gyr);
    zassert_equal(err, 0, "ICM45686 could not get gyro data (err %d)", err);

    err = sensor_channel_get(dev_icm45686_base, SENSOR_CHAN_DIE_TEMP, die_temp);
    zassert_equal(err, 0, "ICM45686 could not get die temperature data (err %d)", err);

    printf("%s: \t AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; "
		       "GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d; "
               "Temp: %d.%06d; \n",
			   dev_icm45686_base->name,
		       acc[0].val1, acc[0].val2,
		       acc[1].val1, acc[1].val2,
		       acc[2].val1, acc[2].val2,
		       gyr[0].val1, gyr[0].val2,
		       gyr[1].val1, gyr[1].val2,
		       gyr[2].val1, gyr[2].val2,
               die_temp[0].val1, die_temp[0].val2);
}

ZTEST_SUITE(icm45686_base, NULL, NULL, NULL, NULL, NULL);

// Barometer
ZTEST(bmp581, test_baro_rdy)
{
    // zassert_true(device_is_ready(dev_bmp581), "Device was not ready");
}

ZTEST(bmp581, test_get_baro_data)
{
    // struct sensor_value baro_press[1], baro_temp[1];
    // int err;

    // err = sensor_sample_fetch(dev_bmp581);
    // zassert_equal(err, 0, "Barometer could not fetch data (err %d)", err);

    // err = sensor_channel_get(dev_bmp581, SENSOR_CHAN_PRESS, baro_press);
    // zassert_equal(err, 0, "Barometer could not get pressure data (err %d)", err);

    // err = sensor_channel_get(dev_bmp581, SENSOR_CHAN_AMBIENT_TEMP, baro_temp);
    // zassert_equal(err, 0, "Barometer could not get ambient temperature data (err %d)", err);

    // printf("%s: \t Pressure: %d.%06d; Ta: %d.%06d;\n",
	// 		   dev_bmp581->name,
	// 	       baro_press[0].val1, baro_press[0].val2,
	// 	       baro_temp[0].val1, baro_temp[0].val2);
}

ZTEST_SUITE(bmp581, NULL, NULL, NULL, NULL, NULL);

// Magnetometer
ZTEST(bmm350, test_mag_rdy)
{
    zassert_true(device_is_ready(dev_bmm350), "Device was not ready");
}

ZTEST(bmm350, test_get_mag_data)
{
    struct sensor_value mag_xyz[3];
    int err;

    err = sensor_sample_fetch(dev_bmm350);
    zassert_equal(err, 0, "Magnetometer could not fetch data (err %d)", err);

    err = sensor_channel_get(dev_bmm350, SENSOR_CHAN_MAGN_XYZ, mag_xyz);
    zassert_equal(err, 0, "Magnetometer could not get XYZ data (err %d)", err);

    printf("%s: \t X: %d.%06d; Y: %d.%06d; Z: %d.%06d;\n",
			   dev_bmm350->name,
		       mag_xyz[0].val1, mag_xyz[0].val2,
		       mag_xyz[1].val1, mag_xyz[1].val2,
		       mag_xyz[2].val1, mag_xyz[2].val2);
}

ZTEST_SUITE(bmm350, NULL, NULL, NULL, NULL, NULL);

// CAN
ZTEST(canbus, test_can_rdy)
{
    zassert_true(device_is_ready(dev_canbus), "Device was not ready");
}



ZTEST_SUITE(canbus, NULL, NULL, NULL, NULL, NULL);

// ENET


// Led strip
ZTEST(apa102, test_led_rdy)
{
    zassert_true(device_is_ready(dev_apa102), "Device was not ready");
}

ZTEST(apa102, test_led_rgb)
{
	int err;

    // Loop through RGB colors with delay
    for (size_t color = 0; color < ARRAY_SIZE(colors); color++)
    {
        for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) 
        {
            memset(&pixels, 0x00, sizeof(pixels));
            memcpy(&pixels[cursor], &colors[color], sizeof(struct led_rgb));

            err = led_strip_update_rgb(dev_apa102, pixels, STRIP_NUM_PIXELS);
            zassert_equal(err, 0, "Couldn't update led strip: %d", err);

            k_sleep(DELAY_TIME);
        }
    }

    // Turn off led strip
    memset(&pixels, 0x00, sizeof(pixels));

    err = led_strip_update_rgb(dev_apa102, pixels, STRIP_NUM_PIXELS);
    zassert_equal(err, 0, "Couldn't update led strip: %d", err);
}

ZTEST_SUITE(apa102, NULL, NULL, NULL, NULL, NULL);

// #define CONFIG_OFLO_ADDON

// ICM45686 optical flow
ZTEST(icm45686_opt, test_icm45686_opt_rdy)
{
    Z_TEST_SKIP_IFNDEF(CONFIG_OFLO_ADDON);

    zassert_true(device_is_ready(dev_icm45686_opt));
}

ZTEST(icm45686_opt, test_icm45686_opt_get_data)
{
    Z_TEST_SKIP_IFNDEF(CONFIG_OFLO_ADDON);

    struct sensor_value acc[3], gyr[3], die_temp[1];
    int err;

    err = sensor_sample_fetch(dev_icm45686_opt);
    zassert_equal(err, 0, "ICM45686 could not fetch data (err %d)", err);

    err = sensor_channel_get(dev_icm45686_opt, SENSOR_CHAN_ACCEL_XYZ, acc);
    zassert_equal(err, 0, "ICM45686 could not get accel data (err %d)", err);

	err = sensor_channel_get(dev_icm45686_opt, SENSOR_CHAN_GYRO_XYZ, gyr);
    zassert_equal(err, 0, "ICM45686 could not get gyro data (err %d)", err);

    err = sensor_channel_get(dev_icm45686_opt, SENSOR_CHAN_DIE_TEMP, die_temp);
    zassert_equal(err, 0, "ICM45686 could not get die temperature data (err %d)", err);

    printf("%s: \t AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; "
		       "GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d; "
               "Temp: %d.%06d; \n",
			   dev_icm45686_opt->name,
		       acc[0].val1, acc[0].val2,
		       acc[1].val1, acc[1].val2,
		       acc[2].val1, acc[2].val2,
		       gyr[0].val1, gyr[0].val2,
		       gyr[1].val1, gyr[1].val2,
		       gyr[2].val1, gyr[2].val2,
               die_temp[0].val1, die_temp[0].val2);
}

ZTEST_SUITE(icm45686_opt, NULL, NULL, NULL, NULL, NULL);

// ICM42688 optical flow
ZTEST(icm42688_opt, test_icm42688_opt_rdy)
{
    Z_TEST_SKIP_IFNDEF(CONFIG_OFLO_ADDON);

    zassert_true(device_is_ready(dev_icm42688));
}

ZTEST(icm42688_opt, test_icm42688_opt_get_data)
{
    Z_TEST_SKIP_IFNDEF(CONFIG_OFLO_ADDON);

    struct sensor_value acc[3], gyr[3], die_temp[1];
    int err;

    err = sensor_sample_fetch(dev_icm42688);
    zassert_equal(err, 0, "ICM45686 could not fetch data (err %d)", err);

    err = sensor_channel_get(dev_icm42688, SENSOR_CHAN_ACCEL_XYZ, acc);
    zassert_equal(err, 0, "ICM45686 could not get accel data (err %d)", err);

	err = sensor_channel_get(dev_icm42688, SENSOR_CHAN_GYRO_XYZ, gyr);
    zassert_equal(err, 0, "ICM45686 could not get gyro data (err %d)", err);

    err = sensor_channel_get(dev_icm42688, SENSOR_CHAN_DIE_TEMP, die_temp);
    zassert_equal(err, 0, "ICM45686 could not get die temperature data (err %d)", err);

    printf("%s: \t AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; "
		       "GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d; "
               "Temp: %d.%06d; \n",
			   dev_icm42688->name,
		       acc[0].val1, acc[0].val2,
		       acc[1].val1, acc[1].val2,
		       acc[2].val1, acc[2].val2,
		       gyr[0].val1, gyr[0].val2,
		       gyr[1].val1, gyr[1].val2,
		       gyr[2].val1, gyr[2].val2,
               die_temp[0].val1, die_temp[0].val2);

    // for (int i=0; i < 3; i++)
    // {
    //     ug[i] = sensor_ms2_to_ug(&acc[i]);
    // }

    printf("%s: \t UGX: %d; UGY: %d; UGZ: %d;\n",
			   dev_icm42688->name,
		       sensor_ms2_to_ug(&acc[0]),
		       sensor_ms2_to_ug(&acc[1]),
		       sensor_ms2_to_ug(&acc[2]));
}

static void test_fetch_accel_with_range(int16_t accel_range_g, 
                    const int16_t accel_percent[3])
{
	struct sensor_value values[3];
	int32_t expect_ug;
	int32_t actual_ug;
	// uint8_t register_buffer[6];

	// /* Se the INT_STATUS register to show we have data */
	// register_buffer[0] = BIT_DATA_RDY_INT;
	// icm4268x_emul_set_reg(dev_icm42688, REG_INT_STATUS, register_buffer, 1);

	/* Set accel range */
	sensor_g_to_ms2(accel_range_g, &values[0]);
	zassert_ok(sensor_attr_set(dev_icm42688, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE,
				   &values[0]));

	// /* Set the accel data accel_percent * accel_range_g */
	// for (int i = 0; i < 3; ++i) {
	// 	register_buffer[i * 2] = (accel_percent[i] >> 8) & GENMASK(7, 0);
	// 	register_buffer[i * 2 + 1] = accel_percent[i] & GENMASK(7, 0);
	// }
	// icm4268x_emul_set_reg(dev_icm42688, REG_ACCEL_DATA_X1, register_buffer, 6);

	/* Fetch the data */
	zassert_ok(sensor_sample_fetch(dev_icm42688));
	zassert_ok(sensor_channel_get(dev_icm42688, SENSOR_CHAN_ACCEL_XYZ, values));

	/* Assert the data is within 0.005g (0.05m/s2) */
	actual_ug = sensor_ms2_to_ug(&values[0]);
	expect_ug = (int32_t)(accel_percent[0] * INT64_C(1000000) * accel_range_g / INT16_MAX);
	zassert_within(expect_ug, actual_ug, INT32_C(5000),
		       "Expected %" PRIi32 " ug, got X=%" PRIi32 " ug", expect_ug, actual_ug);

	actual_ug = sensor_ms2_to_ug(&values[1]);
	expect_ug = (int32_t)(accel_percent[1] * INT64_C(1000000) * accel_range_g / INT16_MAX);
	zassert_within(expect_ug, actual_ug, INT32_C(5000),
		       "Expected %" PRIi32 " ug, got X=%" PRIi32 " ug", expect_ug, actual_ug);

	actual_ug = sensor_ms2_to_ug(&values[2]);
	expect_ug = (int32_t)(accel_percent[2] * INT64_C(1000000) * accel_range_g / INT16_MAX);
	zassert_within(expect_ug, actual_ug, INT32_C(5000),
		       "Expected %" PRIi32 " ug, got X=%" PRIi32 " ug", expect_ug, actual_ug);
}

ZTEST(icm42688_opt, test_icm42688_opt_fetch_accel)
{
	/* Use (0.25, -0.33.., 0.91) * range for testing accel values */
	const int16_t accel_percent[3] = {
		INT16_MAX / INT16_C(4),
		INT16_MIN / INT16_C(3),
		(int16_t)((INT16_MAX * INT32_C(91)) / INT32_C(100)),
	};

	test_fetch_accel_with_range(2, accel_percent);
	test_fetch_accel_with_range(4, accel_percent);
	test_fetch_accel_with_range(8, accel_percent);
	test_fetch_accel_with_range(16, accel_percent);
}

ZTEST_SUITE(icm42688_opt, NULL, NULL, NULL, NULL, NULL);

// Optical flow
ZTEST(paa3905, test_paa3905_rdy)
{
    Z_TEST_SKIP_IFNDEF(CONFIG_OFLO_ADDON);

    zassert_true(device_is_ready(dev_paa3905), "PAA3905 was not ready");
}

ZTEST_SUITE(paa3905, NULL, NULL, NULL, NULL, NULL);

// Time of flight
ZTEST(afbr, test_afbr_rdy)
{
    Z_TEST_SKIP_IFNDEF(CONFIG_OFLO_ADDON);

    zassert_true(device_is_ready(dev_afbr), "Device was not ready");
}

ZTEST_SUITE(afbr, NULL, NULL, NULL, NULL, NULL);

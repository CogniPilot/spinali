// Optical flow test program

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>
#include <stdio.h>

// #define ICM45686_HUB DT_ALIAS(hub)
// #define ICM45686_OPTICAL DT_ALIAS(optical)

#define MAGNETOMETER DT_ALIAS(mag)

ZTEST_DMEM const struct device *const dev_icm45686_base = DEVICE_DT_GET(DT_ALIAS(icm45686base));
// ZTEST_DMEM const struct device *const dev_magneto = DEVICE_DT_GET(MAGNETOMETER);
ZTEST_DMEM const struct device *const dev_bmp581        = DEVICE_DT_GET(DT_NODELABEL(bmp581_0));
ZTEST_DMEM const struct device *const dev_bmm350        = DEVICE_DT_GET(DT_NODELABEL(bmm350_0));

#ifdef CONFIG_OFLO_ADDON
ZTEST_DMEM const struct device *const dev_icm45686_opt  = DEVICE_DT_GET(DT_ALIAS(icm45686opt));
ZTEST_DMEM const struct device *const dev_icm42688      = DEVICE_DT_GET(DT_ALIAS(icm42688));
ZTEST_DMEM const struct device *const dev_paa3905       = DEVICE_DT_GET(DT_ALIAS(paa3905));
ZTEST_DMEM const struct device *const dev_afbr          = DEVICE_DT_GET(DT_ALIAS(afbr));
#endif

ZTEST(icm45686_base, test_icm_rdy)
{
    zassert_true(device_is_ready(dev_icm45686_base), "Device was not ready");
}

ZTEST(icm45686_base, test_get_icm_data)
{
    struct sensor_value acc[3], gyr[3];
    int err;

    err = sensor_sample_fetch(dev_icm45686_base);
    zassert_equal(err, 0, "ICM45686 could not fetch data (err %d)", err);

    err = sensor_channel_get(dev_icm45686_base, SENSOR_CHAN_ACCEL_XYZ, acc);
    zassert_equal(err, 0, "ICM45686 could not get accel data (err %d)", err);

	err = sensor_channel_get(dev_icm45686_base, SENSOR_CHAN_GYRO_XYZ, gyr);
    zassert_equal(err, 0, "ICM45686 could not get gyro data (err %d)", err);

    printf("%s: \t AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; "
		       "GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d;\n",
			   dev_icm45686_base->name,
		       acc[0].val1, acc[0].val2,
		       acc[1].val1, acc[1].val2,
		       acc[2].val1, acc[2].val2,
		       gyr[0].val1, gyr[0].val2,
		       gyr[1].val1, gyr[1].val2,
		       gyr[2].val1, gyr[2].val2);
}

ZTEST_SUITE(icm45686_base, NULL, NULL, NULL, NULL, NULL);

ZTEST(bmp581, test_baro_rdy)
{
    zassert_true(device_is_ready(dev_bmp581), "Device was not ready");
}

ZTEST_SUITE(bmp581, NULL, NULL, NULL, NULL, NULL);

ZTEST(bmm350, test_mag_rdy)
{
    zassert_true(device_is_ready(dev_bmm350), "Device was not ready");
}

ZTEST_SUITE(bmm350, NULL, NULL, NULL, NULL, NULL);

#ifdef CONFIG_OFLO_ADDON
ZTEST(paa3905, test_paa3905_rdy)
{
    zassert_true(device_is_ready(dev_paa3905), "PAA3905 was not ready");
}

ZTEST_SUITE(paa3905, NULL, NULL, NULL, NULL, NULL);

ZTEST(afbr, test_afbr_rdy)
{
    zassert_true(device_is_ready(dev_afbr), "Device was not ready");
}

ZTEST_SUITE(afbr, NULL, NULL, NULL, NULL, NULL);
#endif

/*
 * Copyright (c) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/drivers/sensor/afbr_s50.h>

#define AFBR_S50 DT_NODELABEL(afbr_s50)
// #define TEMP_CHANNEL {SENSOR_CHAN_AMBIENT_TEMP, 0}

const struct device *const dev = DEVICE_DT_GET(AFBR_S50);

// SENSOR_DT_READ_IODEV(temp_iodev, AFBR_S50, TEMP_CHANNEL);
// RTIO_DEFINE(afbr_rtio, 1, 1);

struct rtio sensor_read_rtio;

/* Create a single common config for one-shot reading */
// static struct sensor_chan_spec iodev_sensor_shell_channels[SENSOR_CHAN_ALL];
// static struct sensor_read_config iodev_sensor_shell_read_config = {
// 	.sensor = NULL,
// 	.is_streaming = false,
// 	.channels = iodev_sensor_shell_channels,
// 	.count = 0,
// 	.max = ARRAY_SIZE(iodev_sensor_shell_channels),
// };
// RTIO_IODEV_DEFINE(iodev_sensor_shell_read, &__sensor_iodev_api, &iodev_sensor_shell_read_config);

// /* Create the RTIO context to service the reading */
// RTIO_DEFINE_WITH_MEMPOOL(sensor_read_rtio, 8, 8, 32, 64, 4);

int main(void)
{
    int rc;
	// struct sensor_value val[1];
	// struct sensor_shell_processing_context *ctx = userdata;
	const struct sensor_decoder_api *decoder;
	uint8_t decoded_buffer[128];
	struct {
		uint64_t base_timestamp_ns;
		int count;
		uint64_t timestamp_delta;
		int64_t values[3];
		int8_t shift;
	} accumulator_buffer;

	uint8_t *buf;
	uint32_t buf_len = 32;
	// struct sensor_q31_data temp_data = {0};
	// struct sensor_decode_context temp_decoder = SENSOR_DECODE_CONTEXT_INIT(SENSOR_DECODER_DT_GET(AFBR_S50), buf, SENSOR_CHAN_AFBR_S50_PIXELS, 0);

	if (!device_is_ready(dev)) {
		printf("Device %s is not ready\n", dev->name);
		return 0;
	}

	printf("Device %p name is %s\n", dev, dev->name);

	while (1) {
		/* Blocking read */
		// rc = sensor_read(dev, &temp_ctx, buf, sizeof(buf));
		// err = sensor_sample_fetch(dev);

		// if (err != 0) {
		// 	printk("sensor_read() failed %d\n", err);
		// }

		// void *userdata = NULL;
		// uint8_t *buf = NULL;
		// uint32_t buf_len = 0;
		int rc;

		/* Wait for a CQE */
		struct rtio_cqe *cqe = rtio_cqe_consume_block(&sensor_read_rtio);

		/* Cache the data from the CQE */
		rc = cqe->result;
		// userdata = cqe->userdata;
		rtio_cqe_get_mempool_buffer(&sensor_read_rtio, cqe, &buf, &buf_len);

		/* Release the CQE */
		rtio_cqe_release(&sensor_read_rtio, cqe);

		rc = sensor_get_decoder(dev, &decoder);
		if (rc != 0) {
			printk("Sensor get decoder failed: %d\n", rc);
		}

		for (struct sensor_chan_spec ch = {0, 0}; ch.chan_type < SENSOR_CHAN_ALL; ch.chan_type++) {
			uint32_t fit = 0;
			size_t base_size;
			size_t frame_size;
			uint16_t frame_count;

			rc = decoder->get_size_info(ch, &base_size, &frame_size);
			if (rc != 0) {
				printk("Sensor get decoder size failed\n");
			}

			while (decoder->get_frame_count(buf, ch, &frame_count) == 0) {
				fit = 0;
				memset(&accumulator_buffer, 0, sizeof(accumulator_buffer));

				while (decoder->decode(buf, ch, &fit, 1, decoded_buffer) > 0) {
					struct sensor_q31_data *data =
						(struct sensor_q31_data *)decoded_buffer;

					if (accumulator_buffer.count == 0) {
						accumulator_buffer.base_timestamp_ns =
							data->header.base_timestamp_ns;
					}
					accumulator_buffer.count++;
					accumulator_buffer.shift = data->shift;
					accumulator_buffer.timestamp_delta += data->readings[0].timestamp_delta;
					accumulator_buffer.values[0] += data->readings[0].value;
				}

				struct sensor_q31_data *data =
					(struct sensor_q31_data *)decoded_buffer;

				data->header.base_timestamp_ns =
					accumulator_buffer.base_timestamp_ns;
				data->header.reading_count = 1;
				data->shift = accumulator_buffer.shift;
				data->readings[0].timestamp_delta =
					(uint32_t)(accumulator_buffer.timestamp_delta /
						   accumulator_buffer.count);
				data->readings[0].value = (q31_t)(accumulator_buffer.values[0] /
								  accumulator_buffer.count);

				printk("channel type=%d(%s) index=%d shift=%d num_samples=%d\n"
					   "value=%" PRIsensor_q31_data,
					   ch.chan_type,
					   "test",
					   ch.chan_idx,
					   data->shift, accumulator_buffer.count,
					   PRIsensor_q31_data_arg(*data, 0));

				++ch.chan_idx;
			}
		}

		/* Release the memory */
		rtio_release_buffer(&sensor_read_rtio, buf, buf_len);

		// printf("value: %f\n", sensor_value_to_float(&val[0]));
		// /* Decode the data into a single q31 */
		// sensor_decode(&temp_decoder, &temp_data, 1);

		// printk("Temperature " PRIsensor_q31_data "\n",
		//        PRIsensor_q31_data_arg(temp_data, 0));

		// k_msleep(1);
	}
	return 0;
}

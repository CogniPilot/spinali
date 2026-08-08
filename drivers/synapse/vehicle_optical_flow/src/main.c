/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * PX4-style Vehicle Optical Flow processing module.
 * Fuses PAA3905, ICM45686, and AFBR-S50 sensor data.
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#include "integrator_coning.h"
#include "ring_buffer.h"

LOG_MODULE_REGISTER(vehicle_optical_flow, CONFIG_SPINALI_VEHICLE_OPTICAL_FLOW_LOG_LEVEL);

#define MY_STACK_SIZE 4096
#define MY_PRIORITY   3

#define VOF_SCALE     (CONFIG_VOF_SCALE * 0.01f)
#define VOF_MINHGT    (CONFIG_VOF_MINHGT * 0.01f)
#define VOF_MAXHGT    (CONFIG_VOF_MAXHGT * 0.01f)
#define VOF_MAXR      (CONFIG_VOF_MAXR * 0.01f)
#define VOF_SENS      (CONFIG_VOF_SENS * 0.01f)

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	struct zros_node node;
	/* subscriptions */
	struct zros_sub sub_optical_flow_raw;
	struct zros_sub sub_imu;
	struct zros_sub sub_argus;
	/* subscription data */
	synapse_pb_PixartPAA3905 optical_flow_raw;
	synapse_pb_Imu imu;
	synapse_pb_ArgusResults argus;
	/* publications */
	struct zros_pub pub_optical_flow;
	struct zros_pub pub_optical_flow_vel;
	/* publication data */
	synapse_topic_OpticalFlowData_t optical_flow;
	synapse_topic_OpticalFlowVelocityData_t optical_flow_vel;
	/* processing state */
	struct integrator_coning gyro_integrator;
	struct gyro_ring_buffer gyro_buffer;
	struct range_ring_buffer range_buffer;
	/* accumulation */
	float flow_integral[2];
	float delta_angle[3];
	uint32_t integration_timespan_us;
	float distance_sum;
	uint8_t distance_sum_count;
	uint16_t quality_sum;
	uint8_t accumulated_count;
	int64_t flow_timestamp_sample_last_us;
	int64_t gyro_timestamp_sample_last_us;
	/* thread */
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
};

static struct context g_ctx = {
	.node = {},
	.sub_optical_flow_raw = {},
	.sub_imu = {},
	.sub_argus = {},
	.optical_flow_raw = {},
	.imu = {},
	.argus = {},
	.pub_optical_flow = {},
	.pub_optical_flow_vel = {},
	.optical_flow = {},
	.optical_flow_vel = {},
	.gyro_integrator = {},
	.gyro_buffer = {},
	.range_buffer = {},
	.flow_integral = {0.0f, 0.0f},
	.delta_angle = {0.0f, 0.0f, 0.0f},
	.integration_timespan_us = 0,
	.distance_sum = NAN,
	.distance_sum_count = 0,
	.quality_sum = 0,
	.accumulated_count = 0,
	.flow_timestamp_sample_last_us = 0,
	.gyro_timestamp_sample_last_us = 0,
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
};

static int64_t timestamp_from_pb(const synapse_pb_Timestamp *ts)
{
	return ts->seconds * 1000000LL + ts->nanos / 1000LL;
}

static void clear_accumulated_data(struct context *ctx)
{
	ctx->flow_integral[0] = 0.0f;
	ctx->flow_integral[1] = 0.0f;
	ctx->integration_timespan_us = 0;
	ctx->delta_angle[0] = 0.0f;
	ctx->delta_angle[1] = 0.0f;
	ctx->delta_angle[2] = 0.0f;
	ctx->distance_sum = NAN;
	ctx->distance_sum_count = 0;
	ctx->quality_sum = 0;
	ctx->accumulated_count = 0;
	integrator_coning_reset(&ctx->gyro_integrator);
}

static void update_gyro_buffer(struct context *ctx)
{
	const synapse_pb_Imu *imu = &ctx->imu;

	if (!imu->has_stamp || !imu->has_angular_velocity) {
		return;
	}

	int64_t timestamp_us = timestamp_from_pb(&imu->stamp);
	float dt_s = (timestamp_us - ctx->gyro_timestamp_sample_last_us) * 1e-6f;

	ctx->gyro_timestamp_sample_last_us = timestamp_us;

	if (dt_s <= 0.0f || dt_s > 0.1f) {
		return;
	}

	struct gyro_sample sample = {
		.time_us = (uint64_t)timestamp_us,
		.data = {
			(float)imu->angular_velocity.x,
			(float)imu->angular_velocity.y,
			(float)imu->angular_velocity.z,
		},
		.dt = dt_s,
	};

	gyro_ring_buffer_push(&ctx->gyro_buffer, &sample);
}

static void update_range_buffer(struct context *ctx)
{
	const synapse_pb_ArgusResults *argus = &ctx->argus;

	if (!argus->has_stamp || !argus->has_bin) {
		return;
	}

	/* use bin result (1D pixel binning algorithm output) */
	if (argus->bin.signal_quality < 10) {
		return;
	}

	float range_m = argus->bin.range;

	if (!isfinite(range_m) || range_m < VOF_MINHGT || range_m > VOF_MAXHGT) {
		return;
	}

	int64_t timestamp_us = timestamp_from_pb(&argus->stamp);

	struct range_sample sample = {
		.time_us = (uint64_t)timestamp_us,
		.data = range_m,
	};

	range_ring_buffer_push(&ctx->range_buffer, &sample);
}

static void apply_rotation(float *x, float *y, int rot_deg)
{
	float tmp;

	switch (rot_deg) {
	case 90:
		tmp = *x;
		*x = -*y;
		*y = tmp;
		break;
	case 180:
		*x = -*x;
		*y = -*y;
		break;
	case 270:
		tmp = *x;
		*x = *y;
		*y = -tmp;
		break;
	default:
		/* 0 degrees, no rotation */
		break;
	}
}

static void process_optical_flow(struct context *ctx)
{
	const synapse_pb_PixartPAA3905 *flow = &ctx->optical_flow_raw;

	if (!flow->has_stamp) {
		return;
	}

	int64_t timestamp_us = timestamp_from_pb(&flow->stamp);

	/*
	 * Convert raw PAA3905 counts to radians.
	 * The PAA3905 delta_x/delta_y are in raw counts.
	 * VOF_SENS is the sensitivity in counts/radian.
	 */
	float pixel_flow_x = 0.0f;
	float pixel_flow_y = 0.0f;

	if (VOF_SENS > 0.0f) {
		pixel_flow_x = (float)flow->delta_x / VOF_SENS;
		pixel_flow_y = (float)flow->delta_y / VOF_SENS;
	}

	/*
	 * Estimate integration timespan from sensor data rate (~126Hz).
	 * PAA3905 doesn't provide integration_timespan directly,
	 * so we compute from timestamp deltas.
	 */
	uint32_t integration_timespan_us = 0;

	if (ctx->flow_timestamp_sample_last_us > 0) {
		int64_t dt = timestamp_us - ctx->flow_timestamp_sample_last_us;

		if (dt > 0 && dt < 100000LL) {
			integration_timespan_us = (uint32_t)dt;
		}
	}

	/* gap detection: clear if timestamp gap > 2x integration_timespan */
	if (ctx->accumulated_count > 0 && integration_timespan_us > 0) {
		uint64_t gap = (uint64_t)(timestamp_us - ctx->flow_timestamp_sample_last_us);

		if (gap > (uint64_t)integration_timespan_us * 2) {
			clear_accumulated_data(ctx);
		}
	}

	/* quality transition detection */
	uint8_t quality = flow->squal;

	if (ctx->accumulated_count > 0 && quality > 0 && ctx->quality_sum == 0) {
		clear_accumulated_data(ctx);
	}

	/* integrate gyro from ring buffer over flow time window */
	if (integration_timespan_us > 0) {
		uint64_t ts_oldest = (uint64_t)(timestamp_us - (int64_t)integration_timespan_us);
		uint64_t ts_newest = (uint64_t)timestamp_us;
		struct gyro_sample gyro_sample;

		while (gyro_ring_buffer_pop_oldest(&ctx->gyro_buffer, ts_oldest, ts_newest,
						   &gyro_sample)) {
			integrator_coning_put(&ctx->gyro_integrator, gyro_sample.data, gyro_sample.dt);

			float min_interval_s = (float)integration_timespan_us * 1e-6f * 0.99f;

			if (integrator_coning_integral_dt(&ctx->gyro_integrator) > min_interval_s) {
				break;
			}
		}

		float delta_angle[3];
		uint32_t delta_angle_dt;

		if (integrator_coning_reset_get(&ctx->gyro_integrator, delta_angle,
						&delta_angle_dt)) {
			ctx->delta_angle[0] += delta_angle[0];
			ctx->delta_angle[1] += delta_angle[1];
			ctx->delta_angle[2] += delta_angle[2];
		} else {
			integrator_coning_reset(&ctx->gyro_integrator);
		}
	}

	/* match distance from range buffer */
	struct range_sample range_sample;

	if (range_ring_buffer_peak_first_older_than(&ctx->range_buffer, (uint64_t)timestamp_us,
						    &range_sample)) {
		if (!isfinite(ctx->distance_sum)) {
			ctx->distance_sum = range_sample.data;
			ctx->distance_sum_count = 1;
		} else {
			ctx->distance_sum += range_sample.data;
			ctx->distance_sum_count++;
		}
	}

	/* accumulate */
	ctx->flow_timestamp_sample_last_us = timestamp_us;
	ctx->flow_integral[0] += pixel_flow_x;
	ctx->flow_integral[1] += pixel_flow_y;
	ctx->integration_timespan_us += integration_timespan_us;
	ctx->quality_sum += quality;
	ctx->accumulated_count++;

	/* rate limiting */
	bool publish = true;

	if (CONFIG_VOF_RATE > 0) {
		float interval_us = 1e6f / (float)CONFIG_VOF_RATE;

		if ((float)ctx->integration_timespan_us < interval_us) {
			publish = false;
		}
	}

	if (!publish || ctx->accumulated_count == 0) {
		return;
	}

	/* --- publish optical_flow --- */
	synapse_topic_OpticalFlowData_t *vof = &ctx->optical_flow;

	memset(vof, 0, sizeof(*vof));

	vof->timestamp_us = (uint64_t)timestamp_us;

	/* apply scale */
	float scaled_flow[2] = {
		ctx->flow_integral[0] * VOF_SCALE,
		ctx->flow_integral[1] * VOF_SCALE,
	};

	/* apply rotation */
	apply_rotation(&scaled_flow[0], &scaled_flow[1], CONFIG_VOF_ROT);

	vof->flow_rad.x = scaled_flow[0];
	vof->flow_rad.y = scaled_flow[1];
	vof->delta_angle_flu_rad.x = ctx->delta_angle[0];
	vof->delta_angle_flu_rad.y = ctx->delta_angle[1];
	vof->delta_angle_flu_rad.z = ctx->delta_angle[2];
	vof->integration_timespan_us = ctx->integration_timespan_us;
	vof->quality_pct = (uint8_t)(ctx->quality_sum / ctx->accumulated_count);

	if (ctx->distance_sum_count > 0 && isfinite(ctx->distance_sum)) {
		vof->distance_m = ctx->distance_sum / (float)ctx->distance_sum_count;
	} else {
		vof->distance_m = NAN;
	}

	vof->max_flow_rate_rad_s = VOF_MAXR;
	vof->min_ground_distance_m = VOF_MINHGT;
	vof->max_ground_distance_m = VOF_MAXHGT;

	zros_pub_update(&ctx->pub_optical_flow);

	/* --- publish optical_flow_vel if distance available --- */
	if (ctx->distance_sum_count > 0 && isfinite(ctx->distance_sum)) {
		float range = ctx->distance_sum / (float)ctx->distance_sum_count;
		float flow_dt = 1e-6f * (float)ctx->integration_timespan_us;

		synapse_topic_OpticalFlowVelocityData_t *vel = &ctx->optical_flow_vel;

		memset(vel, 0, sizeof(*vel));
		vel->timestamp_us = (uint64_t)timestamp_us;

		if (flow_dt > 1e-6f) {
			/*
			 * PX4 sign convention: EKF assumes positive LOS rate is
			 * produced by a RH rotation of the image about the sensor axis.
			 */
			float flow_xy_rad[2] = {-scaled_flow[0], -scaled_flow[1]};
			float gyro_rate_integral[3] = {
				-ctx->delta_angle[0],
				-ctx->delta_angle[1],
				-ctx->delta_angle[2],
			};

			float flow_compensated[2] = {
				flow_xy_rad[0] - gyro_rate_integral[0],
				flow_xy_rad[1] - gyro_rate_integral[1],
			};

			/* velocity in body frame */
			vel->velocity_flu_m_s.x = -range * flow_compensated[1] / flow_dt;
			vel->velocity_flu_m_s.y = range * flow_compensated[0] / flow_dt;

			/* ENU frame velocity - requires attitude, set NAN for now */
			vel->velocity_enu_m_s.x = NAN;
			vel->velocity_enu_m_s.y = NAN;

			/* flow rates */
			vel->flow_rate_uncompensated_rad_s.x = flow_xy_rad[0] / flow_dt;
			vel->flow_rate_uncompensated_rad_s.y = flow_xy_rad[1] / flow_dt;
			vel->flow_rate_compensated_rad_s.x = flow_compensated[0] / flow_dt;
			vel->flow_rate_compensated_rad_s.y = flow_compensated[1] / flow_dt;

			/* gyro rate */
			vel->gyro_flu_rad_s.x = gyro_rate_integral[0] / flow_dt;
			vel->gyro_flu_rad_s.y = gyro_rate_integral[1] / flow_dt;
			vel->gyro_flu_rad_s.z = gyro_rate_integral[2] / flow_dt;
		}

		zros_pub_update(&ctx->pub_optical_flow_vel);
	}

	clear_accumulated_data(ctx);
}

static int vof_init(struct context *ctx)
{
	int ret = 0;

	zros_node_init(&ctx->node, "vehicle_optical_flow");

	ret = zros_sub_init(&ctx->sub_optical_flow_raw, &ctx->node,
			    &topic_optical_flow_raw, &ctx->optical_flow_raw, 126);
	if (ret < 0) {
		LOG_ERR("init sub optical_flow_raw failed: %d", ret);
		return ret;
	}

	ret = zros_sub_init(&ctx->sub_imu, &ctx->node, &topic_imu, &ctx->imu, 800);
	if (ret < 0) {
		LOG_ERR("init sub imu failed: %d", ret);
		return ret;
	}

	ret = zros_sub_init(&ctx->sub_argus, &ctx->node, &topic_argus, &ctx->argus, 100);
	if (ret < 0) {
		LOG_ERR("init sub argus failed: %d", ret);
		return ret;
	}

	ret = zros_pub_init(&ctx->pub_optical_flow, &ctx->node, &topic_optical_flow,
			    &ctx->optical_flow);
	if (ret < 0) {
		LOG_ERR("init pub optical_flow failed: %d", ret);
		return ret;
	}

	ret = zros_pub_init(&ctx->pub_optical_flow_vel, &ctx->node, &topic_optical_flow_vel,
			    &ctx->optical_flow_vel);
	if (ret < 0) {
		LOG_ERR("init pub optical_flow_vel failed: %d", ret);
		return ret;
	}

	integrator_coning_init(&ctx->gyro_integrator);
	gyro_ring_buffer_init(&ctx->gyro_buffer);
	range_ring_buffer_init(&ctx->range_buffer);
	clear_accumulated_data(ctx);

	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
	return 0;
}

static int vof_fini(struct context *ctx)
{
	zros_sub_fini(&ctx->sub_optical_flow_raw);
	zros_sub_fini(&ctx->sub_imu);
	zros_sub_fini(&ctx->sub_argus);
	zros_pub_fini(&ctx->pub_optical_flow);
	zros_pub_fini(&ctx->pub_optical_flow_vel);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
	return 0;
}

static void vof_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	int ret = vof_init(ctx);

	if (ret < 0) {
		LOG_ERR("init failed: %d", ret);
		return;
	}

	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
		struct k_poll_event events[] = {
			*zros_sub_get_event(&ctx->sub_optical_flow_raw),
			*zros_sub_get_event(&ctx->sub_imu),
			*zros_sub_get_event(&ctx->sub_argus),
		};

		int rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));

		if (rc != 0) {
			LOG_DBG("poll timeout");
		}

		/* buffer gyro samples */
		if (zros_sub_update(&ctx->sub_imu) == 0) {
			update_gyro_buffer(ctx);
		}

		/* buffer range samples */
		if (zros_sub_update(&ctx->sub_argus) == 0) {
			update_range_buffer(ctx);
		}

		/* process optical flow */
		if (zros_sub_update(&ctx->sub_optical_flow_raw) == 0) {
			process_optical_flow(ctx);
		}
	}

	vof_fini(ctx);
}

static int start(struct context *ctx)
{
	k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size, vof_run,
				      ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);

	k_thread_name_set(tid, "vehicle_optical_flow");
	k_thread_start(tid);
	return 0;
}

static int vof_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
{
	ARG_UNUSED(argc);
	struct context *ctx = data;

	if (strcmp(argv[0], "start") == 0) {
		if (k_sem_count_get(&ctx->running) == 0) {
			shell_print(sh, "already running");
		} else {
			start(ctx);
		}
	} else if (strcmp(argv[0], "stop") == 0) {
		if (k_sem_count_get(&ctx->running) == 0) {
			k_sem_give(&ctx->running);
		} else {
			shell_print(sh, "not running");
		}
	} else if (strcmp(argv[0], "status") == 0) {
		shell_print(sh, "running: %d", (int)(k_sem_count_get(&ctx->running) == 0));
	}

	return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_vof, vof_cmd_handler,
			     (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"),
			     (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(vehicle_optical_flow, &sub_vof, "vehicle optical flow commands", NULL);

static int vof_sys_init(void)
{
	return start(&g_ctx);
}

SYS_INIT(vof_sys_init, APPLICATION, 91);

/* vi: ts=4 sw=4 et */

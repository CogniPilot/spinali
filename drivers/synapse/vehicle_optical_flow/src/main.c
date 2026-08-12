/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Optical flow fusion: accumulates PAA3905 image motion over an integration
 * window, compensates it with the ICM45686 rotation across the same window,
 * and pairs it with an AFBR-S50 range to produce a planar body velocity.
 *
 * Everything published from here is in the body FLU frame. Angles about the
 * body axes are used throughout, so the flow the sensor saw and the rotation
 * the vehicle performed are the same kind of quantity and subtract directly.
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <../drivers/sensor/pixart/paa3905/paa3905_reg.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_time_status.h>
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
#define VOF_MAXSPREAD (CONFIG_VOF_MAXSPREAD * 0.01f)
#define VOF_MAXFRAME_US (CONFIG_VOF_MAXFRAME * 1000U)

/*
 * Accelerometer tilt estimate. The imu accel stream is body specific force in
 * m/s^2, so at rest its magnitude sits near gravity. VOF_TILT_TAU_S is the
 * complementary-filter time constant: the estimate follows the gyro over spans
 * shorter than it and settles onto the accelerometer over longer ones. The
 * accelerometer is trusted for the correction only while its magnitude stays
 * within the band around gravity, which rejects the specific force of
 * maneuvers that would otherwise be read as tilt.
 */
#define VOF_GRAVITY_M_S2 9.80665f
#define VOF_TILT_TAU_S   0.5f
#define VOF_TILT_ACC_LO  (0.5f * VOF_GRAVITY_M_S2)
#define VOF_TILT_ACC_HI  (1.5f * VOF_GRAVITY_M_S2)

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	struct zros_node node;
	/* subscriptions */
	struct zros_sub sub_optical_flow_raw;
	struct zros_sub sub_imu;
	struct zros_sub sub_argus;
	/* subscription data */
	synapse_topic_PixartPaa3905_t optical_flow_raw;
	synapse_topic_InertialSample_t imu;
	synapse_topic_ArgusResults_t argus;
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
	/* accumulation, body FLU */
	float flow_rad[2];
	float delta_angle_flu[3];
	uint32_t integration_timespan_us;
	float distance_sum;
	uint8_t distance_sum_count;
	uint16_t quality_pct_sum;
	uint8_t accumulated_count;
	uint8_t quality_pct_last;
	bool delta_angle_valid;
	int64_t range_timestamp_last_us;
	uint8_t distance_quality_last;
	float distance_spread_last;
	uint8_t distance_pixel_ok_last;
	int64_t flow_timestamp_sample_last_us;
	int64_t gyro_timestamp_sample_last_us;
	/* accelerometer tilt estimate, body FLU relative to level, radians */
	float tilt_roll_rad;
	float tilt_pitch_rad;
	bool tilt_valid;
	/* cumulative health, not cleared between windows */
	uint32_t error_count;
	/*
	 * gPTP discipline latch: set the first time a grandmaster is seen and
	 * never cleared, so a later loss is published as holdover rather than as
	 * never-synchronized.
	 */
	bool time_ever_synced;
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
	.flow_rad = {0.0f, 0.0f},
	.delta_angle_flu = {0.0f, 0.0f, 0.0f},
	.integration_timespan_us = 0,
	.distance_sum = NAN,
	.distance_sum_count = 0,
	.quality_pct_sum = 0,
	.accumulated_count = 0,
	.quality_pct_last = 0,
	.delta_angle_valid = false,
	.range_timestamp_last_us = 0,
	.distance_quality_last = 0,
	.distance_spread_last = 0.0f,
	.distance_pixel_ok_last = 0,
	.flow_timestamp_sample_last_us = 0,
	.gyro_timestamp_sample_last_us = 0,
	.tilt_roll_rad = 0.0f,
	.tilt_pitch_rad = 0.0f,
	.tilt_valid = false,
	.error_count = 0,
	.time_ever_synced = false,
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
};

static void clear_accumulated_data(struct context *ctx)
{
	ctx->flow_rad[0] = 0.0f;
	ctx->flow_rad[1] = 0.0f;
	ctx->integration_timespan_us = 0;
	ctx->delta_angle_flu[0] = 0.0f;
	ctx->delta_angle_flu[1] = 0.0f;
	ctx->delta_angle_flu[2] = 0.0f;
	ctx->distance_sum = NAN;
	ctx->distance_sum_count = 0;
	ctx->quality_pct_sum = 0;
	ctx->accumulated_count = 0;
	ctx->delta_angle_valid = false;
	ctx->range_timestamp_last_us = 0;
	ctx->distance_quality_last = 0;
	ctx->distance_spread_last = 0.0f;
	ctx->distance_pixel_ok_last = 0;
	integrator_coning_reset(&ctx->gyro_integrator);
}

/*
 * Both sensors sit on the same carrier and share the in-plane mapping the
 * flight controller applies to the ICM45686:
 *
 *   body x (forward) <-  sensor y
 *   body y (left)    <- -sensor x
 *   body z (up)      <-  sensor z
 *
 * Applying it at ingest keeps every accumulated quantity in body FLU, which
 * is what makes the gyro rotation subtractable from the measured flow.
 */
static void sensor_xy_to_flu(float sensor_x, float sensor_y, float *flu_x, float *flu_y)
{
	*flu_x = sensor_y;
	*flu_y = -sensor_x;
}

/*
 * Estimate the body FLU roll and pitch relative to the level frame from the
 * accelerometer, blended with the gyro. The same imu sample carries the accel
 * in the same raw chip axes as the gyro, so it maps into body FLU the same way.
 *
 * At rest the accelerometer reads the reaction to gravity, +z in body FLU when
 * level, so atan2 of its components recovers the tilt directly:
 *
 *   roll  = atan2(a_y, a_z)
 *   pitch = atan2(-a_x, hypot(a_y, a_z))
 *
 * A complementary blend integrates the gyro over each interval and eases the
 * result back toward the accelerometer, so brief specific-force transients do
 * not throw the tilt while a steady gravity vector still disciplines the gyro
 * drift. The gyro-only branches keep the estimate coasting when the accel is
 * absent or off-magnitude, once a trustworthy gravity vector has seeded it.
 * gyro_roll_rate and gyro_pitch_rate are the body FLU rates about +x and +y.
 */
static void update_tilt_estimate(struct context *ctx, const synapse_topic_InertialSample_t *imu,
				 float gyro_roll_rate, float gyro_pitch_rate, float dt_s)
{
	float roll_pred = ctx->tilt_roll_rad + gyro_roll_rate * dt_s;
	float pitch_pred = ctx->tilt_pitch_rad + gyro_pitch_rate * dt_s;

	if (!(imu->flags & SYNAPSE_TOPIC_INERTIAL_FIELD_FLAG_ACCEL)) {
		if (ctx->tilt_valid) {
			ctx->tilt_roll_rad = roll_pred;
			ctx->tilt_pitch_rad = pitch_pred;
		}
		return;
	}

	float a_flu_x = imu->accel_flu_m_s2.x;
	float a_flu_y = imu->accel_flu_m_s2.y;
	float a_flu_z = imu->accel_flu_m_s2.z;
	float a_mag = sqrtf(a_flu_x * a_flu_x + a_flu_y * a_flu_y + a_flu_z * a_flu_z);

	bool acc_usable =
		isfinite(a_mag) && a_mag >= VOF_TILT_ACC_LO && a_mag <= VOF_TILT_ACC_HI;

	if (!acc_usable) {
		/* off-magnitude specific force: coast on the gyro once seeded */
		if (ctx->tilt_valid) {
			ctx->tilt_roll_rad = roll_pred;
			ctx->tilt_pitch_rad = pitch_pred;
		}
		return;
	}

	float roll_acc = atan2f(a_flu_y, a_flu_z);
	float pitch_acc = atan2f(-a_flu_x, hypotf(a_flu_y, a_flu_z));

	if (!isfinite(roll_acc) || !isfinite(pitch_acc)) {
		return;
	}

	if (!ctx->tilt_valid) {
		/* first trustworthy gravity vector seeds the estimate */
		ctx->tilt_roll_rad = roll_acc;
		ctx->tilt_pitch_rad = pitch_acc;
		ctx->tilt_valid = true;
		return;
	}

	float alpha = VOF_TILT_TAU_S / (VOF_TILT_TAU_S + dt_s);
	float roll = alpha * roll_pred + (1.0f - alpha) * roll_acc;
	float pitch = alpha * pitch_pred + (1.0f - alpha) * pitch_acc;

	if (isfinite(roll) && isfinite(pitch)) {
		ctx->tilt_roll_rad = roll;
		ctx->tilt_pitch_rad = pitch;
	}
}

static void update_gyro_buffer(struct context *ctx)
{
	const synapse_topic_InertialSample_t *imu = &ctx->imu;

	if (!(imu->flags & SYNAPSE_TOPIC_INERTIAL_FIELD_FLAG_GYRO)) {
		return;
	}

	int64_t timestamp_us = (int64_t)(imu->timestamp_ns / 1000ULL);
	float dt_s = (timestamp_us - ctx->gyro_timestamp_sample_last_us) * 1e-6f;

	ctx->gyro_timestamp_sample_last_us = timestamp_us;

	if (dt_s <= 0.0f || dt_s > 0.1f) {
		return;
	}

	struct gyro_sample sample = {
		.time_us = (uint64_t)timestamp_us,
		.dt = dt_s,
	};

	sample.data[0] = imu->gyro_flu_rad_s.x;
	sample.data[1] = imu->gyro_flu_rad_s.y;
	sample.data[2] = imu->gyro_flu_rad_s.z;

	gyro_ring_buffer_push(&ctx->gyro_buffer, &sample);

	/* fold the same sample into the accelerometer tilt estimate */
	update_tilt_estimate(ctx, imu, sample.data[0], sample.data[1], dt_s);
}

static void update_range_buffer(struct context *ctx)
{
	const synapse_topic_ArgusResults_t *argus = &ctx->argus;

	if (!argus->has_bin) {
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

	int64_t timestamp_us = (int64_t)(argus->timestamp_ns / 1000ULL);

	/*
	 * Summarize the per-pixel footprint that produced this range. The AFBR
	 * reports a range per pixel; the spread between the closest and farthest
	 * pixels that read PIXEL_OK tells a consumer whether the fused range
	 * describes one surface or straddles several. The pixel array is only
	 * present when the argus stream runs in pointcloud or verbose mode; with
	 * pixel_count zero both summaries stay at their neutral values.
	 */
	float spread_m = 0.0f;
	uint8_t pixel_ok = 0;
	float min_r = 0.0f;
	float max_r = 0.0f;

	for (size_t i = 0; i < argus->pixel_count && i < 32U; i++) {
		if (argus->pixel[i].status != SYNAPSE_TOPIC_ARGUS_PIXEL_STATUS_OK) {
			continue;
		}

		float pixel_range = argus->pixel[i].range;

		if (!isfinite(pixel_range)) {
			continue;
		}

		if (pixel_ok == 0) {
			min_r = pixel_range;
			max_r = pixel_range;
		} else if (pixel_range < min_r) {
			min_r = pixel_range;
		} else if (pixel_range > max_r) {
			max_r = pixel_range;
		}

		pixel_ok++;
	}

	if (pixel_ok > 0) {
		spread_m = max_r - min_r;
	}

	struct range_sample sample = {
		.time_us = (uint64_t)timestamp_us,
		.data = range_m,
		.signal_quality = (uint8_t)argus->bin.signal_quality,
		.spread_m = spread_m,
		.pixel_ok = pixel_ok,
	};

	range_ring_buffer_push(&ctx->range_buffer, &sample);
}

/*
 * Trim for a flow sensor mounted rotated about its boresight relative to the
 * IMU. It acts on the in-plane flow angles only: the gyro is already in body
 * axes by the time it reaches the integrator, so rotating it here as well
 * would double-count the mounting.
 */
static void apply_yaw_rotation(float *x, float *y, int rot_deg)
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

/*
 * SQUAL is a raw feature count, meaningful only against the minimum for the
 * observation mode the frame was captured in, below which the driver treats
 * the frame as invalid. Scale the usable span onto the 0 to 100 the schema
 * asks for, and report a frame at or below its minimum as zero quality.
 */
static uint8_t squal_to_quality_pct(uint8_t squal, uint8_t mode)
{
	uint8_t squal_min;

	switch (mode) {
	case OBSERVATION_MODE_BRIGHT:
		squal_min = SQUAL_MIN_BRIGHT;
		break;
	case OBSERVATION_MODE_LOW_LIGHT:
		squal_min = SQUAL_MIN_LOW_LIGHT;
		break;
	case OBSERVATION_MODE_SUPER_LOW_LIGHT:
		squal_min = SQUAL_MIN_SUPER_LOW_LIGHT;
		break;
	default:
		return 0U;
	}

	if (squal <= squal_min) {
		return 0U;
	}

	return (uint8_t)(((uint32_t)(squal - squal_min) * 100U) / (UINT8_MAX - squal_min));
}

/* Rescale a 0-to-100 percentage onto the 0-to-255 range the wire schema uses. */
static uint8_t quality_pct_to_u8(uint8_t pct)
{
	if (pct >= 100U) {
		return UINT8_MAX;
	}

	return (uint8_t)(((uint32_t)pct * UINT8_MAX + 50U) / 100U);
}

/* Map the PAA3905 observation mode onto the wire FlowLightMode enum. */
static uint8_t flow_light_mode_from_pb(uint8_t pb_mode)
{
	switch (pb_mode) {
	case SYNAPSE_TOPIC_PAA3905_MODE_BRIGHT:
		return SYNAPSE_TOPIC_FLOW_LIGHT_MODE_BRIGHT;
	case SYNAPSE_TOPIC_PAA3905_MODE_LOW_LIGHT:
		return SYNAPSE_TOPIC_FLOW_LIGHT_MODE_LOW_LIGHT;
	case SYNAPSE_TOPIC_PAA3905_MODE_SUPER_LOW_LIGHT:
		return SYNAPSE_TOPIC_FLOW_LIGHT_MODE_SUPER_LOW_LIGHT;
	default:
		return SYNAPSE_TOPIC_FLOW_LIGHT_MODE_UNKNOWN;
	}
}

static void process_optical_flow(struct context *ctx)
{
	const synapse_topic_PixartPaa3905_t *flow = &ctx->optical_flow_raw;

	int64_t timestamp_us = (int64_t)(flow->timestamp_ns / 1000ULL);

	/*
	 * The PAA3905 reports no exposure window, so the frame period stands in
	 * for it: the counts in this report were accumulated since the previous
	 * one. That only holds while frames keep arriving. A frame following a
	 * stall covers an unknown span, and a frame far longer than the ones
	 * already accumulated means the stream broke mid-window, so in both
	 * cases the partial accumulation is dropped rather than divided by a
	 * denominator that no longer describes it.
	 */
	int64_t dt = (ctx->flow_timestamp_sample_last_us > 0)
			     ? timestamp_us - ctx->flow_timestamp_sample_last_us
			     : 0;
	bool dt_usable = (dt > 0) && (dt <= (int64_t)VOF_MAXFRAME_US);

	if (ctx->accumulated_count > 0) {
		uint32_t mean_us = ctx->integration_timespan_us / ctx->accumulated_count;

		if (!dt_usable || (mean_us > 0U && (uint64_t)dt > (uint64_t)mean_us * 2U)) {
			LOG_DBG("flow gap %lld us, dropping %u frames", (long long)dt,
				ctx->accumulated_count);
			ctx->error_count++;
			clear_accumulated_data(ctx);
		}
	}

	ctx->flow_timestamp_sample_last_us = timestamp_us;

	if (!dt_usable) {
		return;
	}

	uint32_t integration_timespan_us = (uint32_t)dt;

	uint8_t quality_pct = squal_to_quality_pct(flow->squal, (uint8_t)flow->mode);

	/* surface reacquired: counts spanning the blind interval are not ours */
	if (ctx->accumulated_count > 0 && quality_pct > 0U && ctx->quality_pct_last == 0U) {
		clear_accumulated_data(ctx);
	}

	ctx->quality_pct_last = quality_pct;

	/*
	 * delta_x/delta_y are the displacement of the sensor over the surface
	 * along the sensor axes, in counts. VOF_SENS is counts per radian of
	 * subtended angle, so the quotient is the angle the boresight swept
	 * relative to the surface, independently of height.
	 */
	float sweep[2] = {0.0f, 0.0f};

	if (VOF_SENS > 0.0f) {
		sensor_xy_to_flu((float)flow->delta_x / VOF_SENS, (float)flow->delta_y / VOF_SENS,
				 &sweep[0], &sweep[1]);
		apply_yaw_rotation(&sweep[0], &sweep[1], CONFIG_VOF_ROT);
	}

	/*
	 * Restate the sweep as rotations about the body axes, which is what
	 * flow_rad means and what makes it comparable with delta_angle_flu_rad.
	 * Sweeping the boresight forward is a positive rotation about +y;
	 * sweeping it left is a negative rotation about +x.
	 */
	float flow_rad[2] = {
		-sweep[1] * VOF_SCALE,
		sweep[0] * VOF_SCALE,
	};

	/* integrate gyro from ring buffer over flow time window */
	uint64_t ts_oldest = (uint64_t)(timestamp_us - (int64_t)integration_timespan_us);
	uint64_t ts_newest = (uint64_t)timestamp_us;
	struct gyro_sample gyro_sample;

	while (gyro_ring_buffer_pop_oldest(&ctx->gyro_buffer, ts_oldest, ts_newest, &gyro_sample)) {
		integrator_coning_put(&ctx->gyro_integrator, gyro_sample.data, gyro_sample.dt);

		float min_interval_s = (float)integration_timespan_us * 1e-6f * 0.99f;

		if (integrator_coning_integral_dt(&ctx->gyro_integrator) > min_interval_s) {
			break;
		}
	}

	float delta_angle_flu[3];
	uint32_t delta_angle_dt;

	if (integrator_coning_reset_get(&ctx->gyro_integrator, delta_angle_flu, &delta_angle_dt)) {
		ctx->delta_angle_flu[0] += delta_angle_flu[0];
		ctx->delta_angle_flu[1] += delta_angle_flu[1];
		ctx->delta_angle_flu[2] += delta_angle_flu[2];
		ctx->delta_angle_valid = true;
	} else {
		integrator_coning_reset(&ctx->gyro_integrator);
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

		/* track the range sample folded into distance_m (most recent match) */
		ctx->range_timestamp_last_us = (int64_t)range_sample.time_us;
		ctx->distance_quality_last = range_sample.signal_quality;
		ctx->distance_spread_last = range_sample.spread_m;
		ctx->distance_pixel_ok_last = range_sample.pixel_ok;
	}

	/* accumulate */
	ctx->flow_rad[0] += flow_rad[0];
	ctx->flow_rad[1] += flow_rad[1];
	ctx->integration_timespan_us += integration_timespan_us;
	ctx->quality_pct_sum += quality_pct;
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

	/*
	 * All wire time fields are nanoseconds. The source stamps are the
	 * microsecond boot clock, so scale by 1000. When this node is disciplined
	 * as a gPTP slave, time_offset_ns carries those boot-clock stamps onto the
	 * shared grandmaster timescale; before the first lock it is zero and the
	 * stamps stay node-local monotonic boot time. time_status names the domain
	 * the stamps are on. The same offset and status are applied to the velocity
	 * message below, so both messages agree.
	 */
	int64_t time_offset_ns = 0;
	uint8_t time_status = synapse_time_status_resolve(&ctx->time_ever_synced, &time_offset_ns);

	vof->timestamp_ns = (uint64_t)((int64_t)timestamp_us * 1000LL + time_offset_ns);

	/* the window runs back from timestamp_us over integration_timespan_us */
	int64_t sample_center_us = timestamp_us - (int64_t)ctx->integration_timespan_us / 2;

	vof->timestamp_sample_ns = (uint64_t)(sample_center_us * 1000LL + time_offset_ns);

	vof->flow_rad.x = ctx->flow_rad[0];
	vof->flow_rad.y = ctx->flow_rad[1];
	vof->delta_angle_flu_rad.x = ctx->delta_angle_flu[0];
	vof->delta_angle_flu_rad.y = ctx->delta_angle_flu[1];
	vof->delta_angle_flu_rad.z = ctx->delta_angle_flu[2];

	vof->integration_timespan_ns =
		(uint32_t)((uint64_t)ctx->integration_timespan_us * 1000ULL);

	uint8_t quality_pct_mean = (uint8_t)(ctx->quality_pct_sum / ctx->accumulated_count);

	vof->quality = quality_pct_to_u8(quality_pct_mean);

	bool distance_valid = (ctx->distance_sum_count > 0 && isfinite(ctx->distance_sum));
	float distance_mean = 0.0f;

	if (distance_valid) {
		distance_mean = ctx->distance_sum / (float)ctx->distance_sum_count;
		vof->distance_m = distance_mean;
		vof->distance_timestamp_ns =
			(uint64_t)(ctx->range_timestamp_last_us * 1000LL + time_offset_ns);
		vof->distance_quality = quality_pct_to_u8(ctx->distance_quality_last);

		/*
		 * Per-pixel AFBR footprint of the matched range: the spread and
		 * the PIXEL_OK count carried alongside it through the range ring
		 * buffer. With the argus stream in bin-only mode there are no
		 * pixels, so both stay zero.
		 */
		vof->distance_spread_m = ctx->distance_spread_last;
		vof->distance_pixel_ok = ctx->distance_pixel_ok_last;
	}

	vof->max_flow_rate_rad_s = VOF_MAXR;
	vof->min_ground_distance_m = VOF_MINHGT;
	vof->max_ground_distance_m = VOF_MAXHGT;

	/* no field-of-view config or devicetree constant is exposed to this node */
	vof->field_of_view_rad = 0.0f;

	/*
	 * No Celsius-calibrated temperature source is available: the PAA3905
	 * reports none and the AFBR auxiliary temperature is a raw ADC channel,
	 * not degrees Celsius. NAN marks it unavailable rather than implying 0 C.
	 */
	vof->temperature_c = NAN;

	vof->mode = flow_light_mode_from_pb((uint8_t)flow->mode);
	vof->error_count = ctx->error_count;

	uint8_t flow_flags = 0;

	if (quality_pct_mean > 0U) {
		flow_flags |= SYNAPSE_TOPIC_OPTICAL_FLOW_FLAG_FLOW_VALID;
	}
	if (ctx->delta_angle_valid) {
		flow_flags |= SYNAPSE_TOPIC_OPTICAL_FLOW_FLAG_DELTA_ANGLE_VALID;
	}
	if (distance_valid) {
		flow_flags |= SYNAPSE_TOPIC_OPTICAL_FLOW_FLAG_DISTANCE_VALID;
	}
	/*
	 * A per-pixel spread beyond VOF_MAXSPREAD means the footprint straddles
	 * more than one surface, so the fused distance is ambiguous. This needs
	 * at least one PIXEL_OK reading; with no pixels the spread is zero and
	 * the flag stays clear.
	 */
	if (distance_valid && ctx->distance_pixel_ok_last > 0 &&
	    ctx->distance_spread_last > VOF_MAXSPREAD) {
		flow_flags |= SYNAPSE_TOPIC_OPTICAL_FLOW_FLAG_DISTANCE_AMBIGUOUS;
	}
	vof->flags = flow_flags;

	vof->time_status = time_status;

	/* single flow-sensor instance on this node */
	vof->id = 0U;

	zros_pub_update(&ctx->pub_optical_flow);

	/*
	 * --- publish optical_flow_vel, only for a window that supports one ---
	 *
	 * Without a range there is nothing to scale the flow by, and without a
	 * span there is nothing to divide it by. Staying silent says that; a
	 * message of zeroes would instead say the vehicle is stationary.
	 */
	if (distance_valid && ctx->integration_timespan_us > 0U) {
		float range = distance_mean;
		float flow_dt = 1e-6f * (float)ctx->integration_timespan_us;

		synapse_topic_OpticalFlowVelocityData_t *vel = &ctx->optical_flow_vel;

		/*
		 * Both terms are rotations about the same body axes over the
		 * same window, so what the vehicle rotated subtracts directly
		 * from what the sensor saw.
		 */
		float compensated[2] = {
			ctx->flow_rad[0] - ctx->delta_angle_flu[0],
			ctx->flow_rad[1] - ctx->delta_angle_flu[1],
		};

		memset(vel, 0, sizeof(*vel));
		vel->timestamp_ns = (uint64_t)((int64_t)timestamp_us * 1000LL + time_offset_ns);

		/*
		 * v_perp is the ground velocity perpendicular to the boresight:
		 * the rotation-compensated flow rate scaled by the measured
		 * range. This is slope-safe because it uses the range the sensor
		 * actually saw, not a range*cos(tilt) height that would assume
		 * flat ground. In body FLU, forward travel sweeps the boresight
		 * about +y and leftward travel sweeps it about -x, so in the body
		 * plane:
		 */
		float v_perp_fwd = range * compensated[1] / flow_dt;
		float v_perp_left = -range * compensated[0] / flow_dt;

		/*
		 * Tilt compensation. The accelerometer-derived roll (r) and pitch
		 * (p) give the body attitude relative to the level frame. v_perp
		 * lies in the body plane, so its body-z component is zero;
		 * rotating it into the level frame with R = Ry(p) Rx(r) and
		 * keeping the horizontal part gives the ground velocity in the
		 * level FLU frame:
		 *
		 *   v_level_x = cos(p) v_fwd + sin(p) sin(r) v_left
		 *   v_level_y =                cos(r) v_left
		 *
		 * The discarded z row is the unobservable along-boresight
		 * component. With a valid tilt this replaces the level-only
		 * mapping; otherwise the body-plane v_perp is published as is and
		 * TiltCompensated stays clear.
		 */
		bool tilt_ok = ctx->tilt_valid && isfinite(ctx->tilt_roll_rad) &&
			       isfinite(ctx->tilt_pitch_rad);
		float vel_fwd = v_perp_fwd;
		float vel_left = v_perp_left;
		float roll_out = 0.0f;
		float pitch_out = 0.0f;
		bool tilt_applied = false;

		if (tilt_ok) {
			float r = ctx->tilt_roll_rad;
			float p = ctx->tilt_pitch_rad;
			float cr = cosf(r);
			float sr = sinf(r);
			float cp = cosf(p);
			float sp = sinf(p);
			float lvl_fwd = cp * v_perp_fwd + sp * sr * v_perp_left;
			float lvl_left = cr * v_perp_left;

			if (isfinite(lvl_fwd) && isfinite(lvl_left)) {
				vel_fwd = lvl_fwd;
				vel_left = lvl_left;
				roll_out = r;
				pitch_out = p;
				tilt_applied = true;
			}
		}

		vel->velocity_flu_m_s.x = vel_fwd;
		vel->velocity_flu_m_s.y = vel_left;

		vel->distance_m = range;

		/*
		 * roll_rad and pitch_rad report the attitude the velocity was
		 * corrected for; both stay zero when no valid tilt was applied,
		 * matching the cleared TiltCompensated flag.
		 */
		vel->roll_rad = roll_out;
		vel->pitch_rad = pitch_out;

		/* combined confidence is limited by the weaker of flow and range */
		uint8_t range_quality = quality_pct_to_u8(ctx->distance_quality_last);

		vel->quality = (vof->quality < range_quality) ? vof->quality : range_quality;

		uint8_t vel_flags = SYNAPSE_TOPIC_OPTICAL_FLOW_VELOCITY_FLAG_VELOCITY_VALID;

		/* set only when a valid roll/pitch actually rotated the velocity */
		if (tilt_applied) {
			vel_flags |= SYNAPSE_TOPIC_OPTICAL_FLOW_VELOCITY_FLAG_TILT_COMPENSATED;
		}

		/* the buffered range already passed the ground-distance gate */
		if (range >= VOF_MINHGT && range <= VOF_MAXHGT) {
			vel_flags |= SYNAPSE_TOPIC_OPTICAL_FLOW_VELOCITY_FLAG_RANGE_TRUSTED;
		}
		vel->flags = vel_flags;

		vel->time_status = time_status;
		vel->id = 0U;

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

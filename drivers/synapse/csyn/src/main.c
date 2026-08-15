/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * csyn transport bridge for the synapse topics.
 *
 * This bridge shuttles fixed-layout samples between the in-process zros bus and
 * the csyn topic store. Outbound topics are subscribed on zros and copied into
 * the csyn store; the csyn zenoh transport then publishes them tagged with the
 * value contract resolved from the pinned synapse_fbs catalog. Inbound RTCM3
 * corrections arrive in the csyn store from the transport and are republished
 * on the zros rtcm3 topic for the GNSS forwarder.
 *
 * The bridge declares each carried topic with CSYN_TOPIC_DEFINE against its
 * catalog key, so payload type, encoding, schema hash, and fixed-layout size
 * are checked at csyn init. It never includes csyn's per-type reader headers:
 * the wire contract is owned by csyn and the catalog, not by this driver.
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <csyn/csyn.h>

#include <synapse_topic_list.h>

LOG_MODULE_REGISTER(synapse_csyn, CONFIG_SPINALI_SYNAPSE_CSYN_LOG_LEVEL);

#define MY_STACK_SIZE 4096
#define MY_PRIORITY   5

/*
 * Outbound topics are rate-limited per topic in tx_table, not by one shared cap:
 * GNSS, magnetometer, and time have very different useful rates, and the mesh
 * transport shares one best-effort TX path, so a uniform high cap floods it.
 */
#define BRIDGE_POLL_MS     100

/*
 * Outbound topic presence. A topic is carried when its source exists (the GNSS
 * and gPTP-time Kconfig, or the inertial and magnetometer devicetree stream
 * aliases) AND its per-topic mesh rate Kconfig is above zero. Setting a topic's
 * rate to 0 turns that topic off, so the mesh publish set is entirely conf
 * driven. Each expands to a literal 0/1 usable in the preprocessor and in C.
 */
#if defined(CONFIG_ZROS_SENSE_GNSS) && (CONFIG_SPINALI_SYNAPSE_CSYN_GNSS_RATE_HZ != 0)
#define BRIDGE_TX_GNSS 1
#else
#define BRIDGE_TX_GNSS 0
#endif
#if defined(CONFIG_NET_GPTP) && (CONFIG_SPINALI_SYNAPSE_CSYN_TIME_RATE_HZ != 0)
#define BRIDGE_TX_TIME 1
#else
#define BRIDGE_TX_TIME 0
#endif
/*
 * Raw single-sample inertial is off by default (its rate Kconfig defaults to 0):
 * at the sensor ODR it is one packet per sample, which saturates the shared
 * best-effort multicast TX path and starves the transport control plane. Prefer
 * the batched InertialBatch path for high-rate inertial.
 */
#if DT_NODE_EXISTS(DT_ALIAS(imu_stream_0)) && (CONFIG_SPINALI_SYNAPSE_CSYN_IMU_RATE_HZ != 0)
#define BRIDGE_TX_IMU 1
#else
#define BRIDGE_TX_IMU 0
#endif
#if DT_NODE_EXISTS(DT_ALIAS(mag_stream_0)) && (CONFIG_SPINALI_SYNAPSE_CSYN_MAG_RATE_HZ != 0)
#define BRIDGE_TX_MAG 1
#else
#define BRIDGE_TX_MAG 0
#endif
/*
 * Optical flow is a motion-driven sensor whose ODR reaches the PAA3905 frame
 * rate (126 Hz) and must never be capped below it: dropped frames lose the
 * motion integration the estimator fuses. Its cadence is owned upstream by the
 * vehicle_optical_flow driver (VOF_RATE); the bridge rate follows the shared
 * convention (0 off, -1 unlimited, >0 Hz cap) and defaults to -1 so every
 * sample the driver publishes reaches the mesh.
 */
#if defined(CONFIG_SPINALI_VEHICLE_OPTICAL_FLOW) &&                                                 \
	(CONFIG_SPINALI_SYNAPSE_CSYN_OPTICAL_FLOW_RATE_HZ != 0)
#define BRIDGE_TX_OPTICAL_FLOW 1
#else
#define BRIDGE_TX_OPTICAL_FLOW 0
#endif
#if defined(CONFIG_SPINALI_VEHICLE_OPTICAL_FLOW) &&                                                 \
	(CONFIG_SPINALI_SYNAPSE_CSYN_OPTICAL_FLOW_VEL_RATE_HZ != 0)
#define BRIDGE_TX_OPTICAL_FLOW_VEL 1
#else
#define BRIDGE_TX_OPTICAL_FLOW_VEL 0
#endif
#define BRIDGE_TX_COUNT                                                                            \
	(BRIDGE_TX_GNSS + BRIDGE_TX_TIME + BRIDGE_TX_IMU + BRIDGE_TX_MAG +                          \
	 BRIDGE_TX_OPTICAL_FLOW + BRIDGE_TX_OPTICAL_FLOW_VEL)

#if defined(CONFIG_ZROS_SENSE_RTCM3_SUB)
#define BRIDGE_RX_RTCM3 1
#else
#define BRIDGE_RX_RTCM3 0
#endif

/*
 * One csyn topic per carried wire type. The bare catalog key resolves through
 * the synapse_fbs catalog at csyn init; the reserved store slot size is the
 * in-process fixed-layout struct and must equal the catalog payload_size.
 */
#if BRIDGE_TX_GNSS
CSYN_TOPIC_DEFINE(csyn_gnss, SYNAPSE_TOPIC_GNSS_KEY, CSYN_DIR_TX, sizeof(synapse_topic_GnssFix_t));
#endif
#if BRIDGE_TX_IMU
CSYN_TOPIC_DEFINE(csyn_imu, SYNAPSE_TOPIC_IMU_KEY, CSYN_DIR_TX,
		  sizeof(synapse_topic_InertialSample_t));
#endif
#if BRIDGE_TX_MAG
CSYN_TOPIC_DEFINE(csyn_mag, SYNAPSE_TOPIC_MAG_KEY, CSYN_DIR_TX,
		  sizeof(synapse_topic_MagneticField_t));
#endif
#if BRIDGE_TX_TIME
CSYN_TOPIC_DEFINE(csyn_time, SYNAPSE_TOPIC_TIME_REFERENCE_KEY, CSYN_DIR_TX,
		  sizeof(synapse_topic_TimeReferenceData_t));
#endif
#if BRIDGE_TX_OPTICAL_FLOW
CSYN_TOPIC_DEFINE(csyn_optical_flow, SYNAPSE_TOPIC_OPTICAL_FLOW_KEY, CSYN_DIR_TX,
		  sizeof(synapse_topic_OpticalFlowData_t));
#endif
#if BRIDGE_TX_OPTICAL_FLOW_VEL
CSYN_TOPIC_DEFINE(csyn_optical_flow_vel, SYNAPSE_TOPIC_OPTICAL_FLOW_VELOCITY_KEY, CSYN_DIR_TX,
		  sizeof(synapse_topic_OpticalFlowVelocityData_t));
#endif
#if BRIDGE_RX_RTCM3
CSYN_TOPIC_DEFINE(csyn_rtcm3, "rtcm3", CSYN_DIR_RX, sizeof(synapse_topic_Rtcm3_t));
#endif

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	struct zros_node node;
	/* outbound sample buffers, one per subscribed zros topic */
#if BRIDGE_TX_GNSS
	synapse_topic_GnssFix_t gnss;
#endif
#if BRIDGE_TX_IMU
	synapse_topic_InertialSample_t imu;
#endif
#if BRIDGE_TX_MAG
	synapse_topic_MagneticField_t mag;
#endif
#if BRIDGE_TX_TIME
	synapse_topic_TimeReferenceData_t time_ref;
#endif
#if BRIDGE_TX_OPTICAL_FLOW
	synapse_topic_OpticalFlowData_t optical_flow;
#endif
#if BRIDGE_TX_OPTICAL_FLOW_VEL
	synapse_topic_OpticalFlowVelocityData_t optical_flow_vel;
#endif
#if BRIDGE_RX_RTCM3
	/* inbound RTCM3 correction bytes, republished on topic_rtcm3 */
	synapse_topic_Rtcm3_t rtcm3;
	struct zros_pub pub_rtcm3;
	uint32_t rtcm3_generation;
#endif
	struct k_sem running;
	struct k_thread thread_data;
};

static struct context g_ctx = {
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
};

/*
 * One row per outbound topic: the zros topic it is subscribed on, the csyn
 * store topic its samples are copied into, and the fixed-layout buffer and
 * size. Publisher declaration and the run loop iterate this table.
 */
struct tx_binding {
	struct zros_topic *ztopic;
	struct csyn_topic *ctopic;
	void *buffer;
	size_t size;
	int32_t rate_hz; /* per-topic mesh publish rate: 0 off, -1 unlimited, >0 Hz cap */
};

#if BRIDGE_TX_COUNT > 0
static const struct tx_binding tx_table[] = {
#if BRIDGE_TX_GNSS
	/* GNSS fix, rate from SPINALI_SYNAPSE_CSYN_GNSS_RATE_HZ. */
	{&topic_nav_sat_fix, &csyn_gnss, &g_ctx.gnss, sizeof(g_ctx.gnss),
	 CONFIG_SPINALI_SYNAPSE_CSYN_GNSS_RATE_HZ},
#endif
#if BRIDGE_TX_IMU
	{&topic_imu, &csyn_imu, &g_ctx.imu, sizeof(g_ctx.imu),
	 CONFIG_SPINALI_SYNAPSE_CSYN_IMU_RATE_HZ},
#endif
#if BRIDGE_TX_MAG
	/* Magnetometer (RM3100, 150 Hz) downsampled for the mesh, rate from
	 * SPINALI_SYNAPSE_CSYN_MAG_RATE_HZ. */
	{&topic_mag, &csyn_mag, &g_ctx.mag, sizeof(g_ctx.mag),
	 CONFIG_SPINALI_SYNAPSE_CSYN_MAG_RATE_HZ},
#endif
#if BRIDGE_TX_TIME
	/* Time reference, rate from SPINALI_SYNAPSE_CSYN_TIME_RATE_HZ. */
	{&topic_time_reference, &csyn_time, &g_ctx.time_ref, sizeof(g_ctx.time_ref),
	 CONFIG_SPINALI_SYNAPSE_CSYN_TIME_RATE_HZ},
#endif
#if BRIDGE_TX_OPTICAL_FLOW
	/* Processed optical flow (body-FLU integrated flow with fused range) from
	 * the vehicle_optical_flow driver, rate from
	 * SPINALI_SYNAPSE_CSYN_OPTICAL_FLOW_RATE_HZ. */
	{&topic_optical_flow, &csyn_optical_flow, &g_ctx.optical_flow, sizeof(g_ctx.optical_flow),
	 CONFIG_SPINALI_SYNAPSE_CSYN_OPTICAL_FLOW_RATE_HZ},
#endif
#if BRIDGE_TX_OPTICAL_FLOW_VEL
	/* Tilt-compensated optical-flow velocity, rate from
	 * SPINALI_SYNAPSE_CSYN_OPTICAL_FLOW_VEL_RATE_HZ. */
	{&topic_optical_flow_vel, &csyn_optical_flow_vel, &g_ctx.optical_flow_vel,
	 sizeof(g_ctx.optical_flow_vel), CONFIG_SPINALI_SYNAPSE_CSYN_OPTICAL_FLOW_VEL_RATE_HZ},
#endif
};
BUILD_ASSERT(ARRAY_SIZE(tx_table) == BRIDGE_TX_COUNT);
static struct zros_sub tx_subs[BRIDGE_TX_COUNT];
#endif /* BRIDGE_TX_COUNT > 0 */

static int bridge_init(struct context *ctx)
{
	zros_node_init(&ctx->node, "csyn_bridge");

#if BRIDGE_TX_COUNT > 0
	for (size_t i = 0; i < ARRAY_SIZE(tx_table); i++) {
		int ret = zros_sub_init(&tx_subs[i], &ctx->node, tx_table[i].ztopic,
					tx_table[i].buffer, tx_table[i].rate_hz);
		if (ret < 0) {
			LOG_ERR("init sub %zu failed: %d", i, ret);
			return ret;
		}
	}
#endif

#if BRIDGE_RX_RTCM3
	int ret = zros_pub_init(&ctx->pub_rtcm3, &ctx->node, &topic_rtcm3, &ctx->rtcm3);
	if (ret < 0) {
		LOG_ERR("init pub rtcm3 failed: %d", ret);
		return ret;
	}
#endif

	/* Claim the run token so the loop stays live until a stop gives it back. */
	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
	return 0;
}

static void bridge_fini(struct context *ctx)
{
#if BRIDGE_RX_RTCM3
	zros_pub_fini(&ctx->pub_rtcm3);
#endif
#if BRIDGE_TX_COUNT > 0
	for (size_t i = 0; i < ARRAY_SIZE(tx_table); i++) {
		zros_sub_fini(&tx_subs[i]);
	}
#endif
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
}

#if BRIDGE_RX_RTCM3
/*
 * Move any freshly received RTCM3 sample from the csyn store to the zros rtcm3
 * topic. The csyn zenoh transport writes the store from its receive path, so
 * this bridge only forwards on a generation change.
 */
static void bridge_pump_rtcm3(struct context *ctx)
{
	uint32_t generation = csyn_topic_generation(&csyn_rtcm3);
	size_t len;

	if (generation == 0U || generation == ctx->rtcm3_generation) {
		return;
	}
	if (csyn_topic_copy(&csyn_rtcm3, &ctx->rtcm3, sizeof(ctx->rtcm3), &len, NULL)) {
		zros_pub_update(&ctx->pub_rtcm3);
		ctx->rtcm3_generation = generation;
	}
}
#endif

static void bridge_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	if (bridge_init(ctx) < 0) {
		bridge_fini(ctx);
		return;
	}

	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
#if BRIDGE_TX_COUNT > 0
		struct k_poll_event events[ARRAY_SIZE(tx_table)];

		for (size_t i = 0; i < ARRAY_SIZE(tx_table); i++) {
			events[i] = *zros_sub_get_event(&tx_subs[i]);
		}
		(void)k_poll(events, ARRAY_SIZE(events), K_MSEC(BRIDGE_POLL_MS));

		for (size_t i = 0; i < ARRAY_SIZE(tx_table); i++) {
			if (zros_sub_update(&tx_subs[i]) == 0) {
				(void)csyn_topic_publish(tx_table[i].ctopic, tx_table[i].buffer,
							 tx_table[i].size);
			}
		}
#else
		k_sleep(K_MSEC(BRIDGE_POLL_MS));
#endif

#if BRIDGE_RX_RTCM3
		bridge_pump_rtcm3(ctx);
#endif
	}

	bridge_fini(ctx);
}

static int start(struct context *ctx)
{
	k_tid_t tid = k_thread_create(&ctx->thread_data, g_my_stack_area,
				      K_THREAD_STACK_SIZEOF(g_my_stack_area), bridge_run, ctx, NULL,
				      NULL, MY_PRIORITY, 0, K_FOREVER);

	k_thread_name_set(tid, "csyn_bridge");
	k_thread_start(tid);
	return 0;
}

static int csyn_bridge_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_csyn_bridge, csyn_bridge_cmd_handler, (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"), (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(csyn_bridge, &sub_csyn_bridge, "csyn synapse bridge commands", NULL);

static int csyn_bridge_sys_init(void)
{
	return start(&g_ctx);
}

SYS_INIT(csyn_bridge_sys_init, APPLICATION, 92);

/* vi: ts=4 sw=4 et */

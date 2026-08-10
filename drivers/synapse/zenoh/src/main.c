/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zenoh publisher for the synapse optical flow topics.
 *
 * Payloads go out as raw fixed-layout structs, tagged with the catalog value
 * contract for their type. A receiver matches that contract string before it
 * decodes anything, so key, media type, wire type and schema hash all have to
 * agree with the catalog entry the receiver was built against.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <zenoh-pico.h>

#include <synapse_topic_list.h>

LOG_MODULE_REGISTER(synapse_zenoh, CONFIG_SPINALI_SYNAPSE_ZENOH_LOG_LEVEL);

#define MY_STACK_SIZE 8192
#define MY_PRIORITY   5

#define KEYEXPR_MAX 96

/*
 * Inbound RTCM3 corrections. Only a node that forwards corrections to a GNSS
 * receiver (CONFIG_ZROS_SENSE_RTCM3_SUB) wants this path, and it needs the
 * zenoh subscription feature compiled in. The wire payload is the raw
 * variable-length RTCM3 byte stream (no fixed-layout struct, no value
 * contract), so the driver republishes it verbatim on topic_rtcm3.
 */
#if defined(CONFIG_ZROS_SENSE_RTCM3_SUB) && Z_FEATURE_SUBSCRIPTION == 1
#define SYNAPSE_ZENOH_RTCM3_INBOUND 1
#define SYNAPSE_TOPIC_RTCM3_KEY     "rtcm3"
#else
#define SYNAPSE_ZENOH_RTCM3_INBOUND 0
#endif

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	struct zros_node node;
	/* topic data, pointed to by the topic table rows below */
	synapse_topic_OpticalFlowData_t flow;
	synapse_topic_OpticalFlowVelocityData_t flow_vel;
	synapse_topic_GnssFix_t gnss;
	synapse_topic_InertialSample_t imu;
#if SYNAPSE_ZENOH_RTCM3_INBOUND
	/* inbound RTCM3 correction bytes, republished on topic_rtcm3 */
	synapse_pb_Rtcm3 rtcm3;
#endif
	/* zenoh */
	z_owned_session_t session;
#if SYNAPSE_ZENOH_RTCM3_INBOUND
	z_owned_subscriber_t rtcm3_reader;
	struct zros_pub pub_rtcm3;
#endif
	/* thread */
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
};

static struct context g_ctx = {
	.node = {},
	.flow = {},
	.flow_vel = {},
	.gnss = {},
	.imu = {},
#if SYNAPSE_ZENOH_RTCM3_INBOUND
	.rtcm3 = {},
#endif
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
};

/*
 * One row per published topic. A row binds a subscribed zros topic to the
 * fixed-layout struct buffer its samples land in, and to the Zenoh key and
 * value contract those bytes go out under. Adding a topic is a matter of
 * adding a row; the subscriber init, publisher declaration, poll set, run
 * loop and teardown all iterate this table.
 */
struct topic_binding {
	struct zros_topic *topic;
	void *buffer;
	size_t size;
	const char *key;
	const char *contract;
};

static const struct topic_binding topic_table[] = {
	{
		.topic = &topic_optical_flow,
		.buffer = &g_ctx.flow,
		.size = sizeof(g_ctx.flow),
		.key = SYNAPSE_TOPIC_OPTICAL_FLOW_KEY,
		.contract = SYNAPSE_TOPIC_OPTICAL_FLOW_CONTRACT,
	},
	{
		.topic = &topic_optical_flow_vel,
		.buffer = &g_ctx.flow_vel,
		.size = sizeof(g_ctx.flow_vel),
		.key = SYNAPSE_TOPIC_OPTICAL_FLOW_VELOCITY_KEY,
		.contract = SYNAPSE_TOPIC_OPTICAL_FLOW_VELOCITY_CONTRACT,
	},
	{
		.topic = &topic_nav_sat_fix,
		.buffer = &g_ctx.gnss,
		.size = sizeof(g_ctx.gnss),
		.key = SYNAPSE_TOPIC_GNSS_KEY,
		.contract = SYNAPSE_TOPIC_GNSS_CONTRACT,
	},
	{
		.topic = &topic_imu,
		.buffer = &g_ctx.imu,
		.size = sizeof(g_ctx.imu),
		.key = SYNAPSE_TOPIC_IMU_KEY,
		.contract = SYNAPSE_TOPIC_IMU_CONTRACT,
	},
};

/* Parallel per-row state, indexed the same way as topic_table. */
static struct zros_sub subs[ARRAY_SIZE(topic_table)];
static z_owned_publisher_t publishers[ARRAY_SIZE(topic_table)];

static int zenoh_session_init(struct context *ctx)
{
#if defined(CONFIG_SPINALI_SYNAPSE_ZENOH_MODE_PEER)
	const char *mode = "peer";
#else
	const char *mode = "client";
#endif
	const char *locator = CONFIG_ZENOH_LOCATOR;
	z_owned_config_t config;
	int ret = 0;

	LOG_INF("Opening zenoh session (%s) via %s ...", mode, locator);


	do {
		z_config_default(&config);
		zp_config_insert(z_loan_mut(config), Z_CONFIG_MODE_KEY, mode);
		zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY, locator);

		if (ret != 0) {
			LOG_WRN("Unable to open session (ret=%d), retrying...", ret);
			k_sleep(K_SECONDS(5));
		}
	} while ((ret = z_open(&ctx->session, z_move(config), NULL)) < 0);

#if Z_FEATURE_MULTI_THREAD == 1
	if (zp_start_read_task(z_loan_mut(ctx->session), NULL) < 0 ||
	    zp_start_lease_task(z_loan_mut(ctx->session), NULL) < 0) {
		LOG_ERR("Unable to start read and lease tasks");
		z_drop(z_move(ctx->session));
		return -EINVAL;
	}
#endif

	LOG_INF("Zenoh session opened");
	return 0;
}

/*
 * A bare catalog key is scoped by the deployment namespace; a key that already
 * carries a path is used verbatim. This mirrors how the receiving side derives
 * its own key expressions, which is what makes the two ends meet.
 */
static int topic_keyexpr(const char *key, char *out, size_t out_size)
{
	int len;

	if (CONFIG_SPINALI_SYNAPSE_ZENOH_NAMESPACE[0] != '\0' && strchr(key, '/') == NULL) {
		len = snprintf(out, out_size, "%s/%s", CONFIG_SPINALI_SYNAPSE_ZENOH_NAMESPACE, key);
	} else {
		len = snprintf(out, out_size, "%s", key);
	}

	return (len > 0 && (size_t)len < out_size) ? 0 : -EINVAL;
}

static int declare_topic_publisher(struct context *ctx, z_owned_publisher_t *pub, const char *key,
				   const char *contract)
{
	char keyexpr[KEYEXPR_MAX];
	z_publisher_options_t options;
	z_owned_encoding_t encoding;
	z_view_keyexpr_t ke;
	int ret;

	ret = topic_keyexpr(key, keyexpr, sizeof(keyexpr));
	if (ret < 0) {
		LOG_ERR("Key expression too long for %s", key);
		return ret;
	}

	ret = z_view_keyexpr_from_str(&ke, keyexpr);
	if (ret < 0) {
		LOG_ERR("Invalid key expression %s", keyexpr);
		return ret;
	}

	ret = z_encoding_from_str(&encoding, contract);
	if (ret < 0) {
		LOG_ERR("Invalid value contract for %s", keyexpr);
		return ret;
	}

	z_publisher_options_default(&options);
	options.encoding = z_move(encoding);

	ret = z_declare_publisher(z_loan(ctx->session), pub, z_loan(ke), &options);
	if (ret < 0) {
		LOG_ERR("Unable to declare publisher %s", keyexpr);
		return ret;
	}

	LOG_INF("Zenoh publisher %s", keyexpr);
	return 0;
}

static int zenoh_publishers_init(struct context *ctx)
{
	for (size_t i = 0; i < ARRAY_SIZE(topic_table); i++) {
		int ret = declare_topic_publisher(ctx, &publishers[i], topic_table[i].key,
						  topic_table[i].contract);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static void publish_struct(const z_loaned_publisher_t *pub, const void *data, size_t size)
{
	z_owned_bytes_t payload;

	if (z_bytes_copy_from_buf(&payload, data, size) < 0) {
		return;
	}

	/* Encoding comes from the publisher declaration. */
	z_publisher_put(pub, z_move(payload), NULL);
}

#if SYNAPSE_ZENOH_RTCM3_INBOUND
/*
 * Copy the received RTCM3 bytes into the topic buffer and republish them on
 * topic_rtcm3, where the RTCM3 forwarder picks them up and hands them to the
 * GNSS receiver. The payload is opaque, so a frame longer than the buffer or a
 * short read is dropped rather than truncated. Runs in the zenoh read task
 * (multi-thread) or inside zp_spin_once (single-thread); zros_topic_publish is
 * thread-safe under both.
 */
static void rtcm3_recv_handler(z_loaned_sample_t *sample, void *arg)
{
	struct context *ctx = arg;
	const z_loaned_bytes_t *payload = z_sample_payload(sample);
	size_t len = z_bytes_len(payload);
	z_bytes_reader_t reader;

	if (len == 0 || len > sizeof(ctx->rtcm3.data.bytes)) {
		LOG_WRN("dropping rtcm3 frame of %zu bytes", len);
		return;
	}

	reader = z_bytes_get_reader(payload);
	if (z_bytes_reader_read(&reader, ctx->rtcm3.data.bytes, len) != len) {
		LOG_WRN("short read on rtcm3 frame");
		return;
	}

	ctx->rtcm3.data.size = len;
	zros_pub_update(&ctx->pub_rtcm3);
}

static int zenoh_rtcm3_sub_init(struct context *ctx)
{
	char keyexpr[KEYEXPR_MAX];
	z_owned_closure_sample_t closure;
	z_view_keyexpr_t ke;
	int ret;

	ret = zros_pub_init(&ctx->pub_rtcm3, &ctx->node, &topic_rtcm3, &ctx->rtcm3);
	if (ret < 0) {
		LOG_ERR("init pub rtcm3 failed: %d", ret);
		return ret;
	}

	ret = topic_keyexpr(SYNAPSE_TOPIC_RTCM3_KEY, keyexpr, sizeof(keyexpr));
	if (ret < 0) {
		LOG_ERR("Key expression too long for %s", SYNAPSE_TOPIC_RTCM3_KEY);
		return ret;
	}

	ret = z_view_keyexpr_from_str(&ke, keyexpr);
	if (ret < 0) {
		LOG_ERR("Invalid key expression %s", keyexpr);
		return ret;
	}

	z_closure_sample(&closure, rtcm3_recv_handler, NULL, ctx);

	ret = z_declare_subscriber(z_loan(ctx->session), &ctx->rtcm3_reader, z_loan(ke),
				   z_move(closure), NULL);
	if (ret < 0) {
		LOG_ERR("Unable to declare subscriber %s", keyexpr);
		return ret;
	}

	LOG_INF("Zenoh subscriber %s", keyexpr);
	return 0;
}
#endif /* SYNAPSE_ZENOH_RTCM3_INBOUND */

static int zenoh_init(struct context *ctx)
{
	int ret = 0;

	zros_node_init(&ctx->node, "zenoh");

	for (size_t i = 0; i < ARRAY_SIZE(topic_table); i++) {
		ret = zros_sub_init(&subs[i], &ctx->node, topic_table[i].topic,
				    topic_table[i].buffer, 70);
		if (ret < 0) {
			LOG_ERR("init sub %s failed: %d", topic_table[i].key, ret);
			return ret;
		}
	}

	ret = zenoh_session_init(ctx);
	if (ret < 0) {
		return ret;
	}

	ret = zenoh_publishers_init(ctx);
	if (ret < 0) {
		return ret;
	}

#if SYNAPSE_ZENOH_RTCM3_INBOUND
	ret = zenoh_rtcm3_sub_init(ctx);
	if (ret < 0) {
		return ret;
	}
#endif

	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
	return 0;
}

static int zenoh_fini(struct context *ctx)
{
#if SYNAPSE_ZENOH_RTCM3_INBOUND
	z_undeclare_subscriber(z_move(ctx->rtcm3_reader));
	zros_pub_fini(&ctx->pub_rtcm3);
#endif
	for (size_t i = 0; i < ARRAY_SIZE(topic_table); i++) {
		z_undeclare_publisher(z_move(publishers[i]));
	}
	for (size_t i = 0; i < ARRAY_SIZE(topic_table); i++) {
		zros_sub_fini(&subs[i]);
	}
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
	return 0;
}

static void zenoh_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	int ret = zenoh_init(ctx);

	if (ret < 0) {
		LOG_ERR("init failed: %d", ret);
		return;
	}

	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
		struct k_poll_event events[ARRAY_SIZE(topic_table)];

		for (size_t i = 0; i < ARRAY_SIZE(topic_table); i++) {
			events[i] = *zros_sub_get_event(&subs[i]);
		}

		int rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));

		if (rc != 0) {
			LOG_DBG("poll timeout");
		}

		for (size_t i = 0; i < ARRAY_SIZE(topic_table); i++) {
			if (zros_sub_update(&subs[i]) == 0) {
				publish_struct(z_loan(publishers[i]), topic_table[i].buffer,
					       topic_table[i].size);
			}
		}

#if Z_FEATURE_MULTI_THREAD == 0
		/* Also services the inbound rtcm3 subscriber callback, if any.
		 * In multi-thread builds the read task does this instead.
		 */
		zp_spin_once(z_loan(ctx->session));
#endif
	}

	zenoh_fini(ctx);
}

static int start(struct context *ctx)
{
	k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				      zenoh_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);

	k_thread_name_set(tid, "zenoh");
	k_thread_start(tid);
	return 0;
}

static int zenoh_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_zenoh, zenoh_cmd_handler,
			     (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"),
			     (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(zenoh, &sub_zenoh, "zenoh commands", NULL);

static int zenoh_sys_init(void)
{
	return start(&g_ctx);
}

SYS_INIT(zenoh_sys_init, APPLICATION, 92);

/* vi: ts=4 sw=4 et */

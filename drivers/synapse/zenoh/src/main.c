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
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <zenoh-pico.h>

#include <synapse_topic_list.h>

LOG_MODULE_REGISTER(synapse_zenoh, CONFIG_SPINALI_SYNAPSE_ZENOH_LOG_LEVEL);

#define MY_STACK_SIZE 8192
#define MY_PRIORITY   5

#define KEYEXPR_MAX 96

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	struct zros_node node;
	/* subscriptions */
	struct zros_sub sub_optical_flow;
	struct zros_sub sub_optical_flow_vel;
	/* topic data */
	synapse_topic_OpticalFlowData_t flow;
	synapse_topic_OpticalFlowVelocityData_t flow_vel;
	/* zenoh */
	z_owned_session_t session;
	z_owned_publisher_t pub_flow;
	z_owned_publisher_t pub_flow_vel;
	/* thread */
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
};

static struct context g_ctx = {
	.node = {},
	.sub_optical_flow = {},
	.sub_optical_flow_vel = {},
	.flow = {},
	.flow_vel = {},
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
};

static int zenoh_session_init(struct context *ctx)
{
	const char *mode = "client";
	const char *locator = CONFIG_ZENOH_LOCATOR;
	z_owned_config_t config;
	int ret = 0;

	LOG_INF("Opening zenoh session to %s ...", locator);


	do {
		z_config_default(&config);
		zp_config_insert(z_loan_mut(config), Z_CONFIG_MODE_KEY, mode);
		zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY, locator);

		if (ret != 0) {
			LOG_WRN("Unable to open session (ret=%d), retrying...", ret);
			k_sleep(K_SECONDS(5));
		}
	} while ((ret = z_open(&ctx->session, z_move(config), NULL)) < 0);

	if (zp_start_read_task(z_loan_mut(ctx->session), NULL) < 0 ||
	    zp_start_lease_task(z_loan_mut(ctx->session), NULL) < 0) {
		LOG_ERR("Unable to start read and lease tasks");
		z_drop(z_move(ctx->session));
		return -EINVAL;
	}

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
	int ret;

	ret = declare_topic_publisher(ctx, &ctx->pub_flow, SYNAPSE_TOPIC_OPTICAL_FLOW_KEY,
				      SYNAPSE_TOPIC_OPTICAL_FLOW_CONTRACT);
	if (ret < 0) {
		return ret;
	}

	return declare_topic_publisher(ctx, &ctx->pub_flow_vel,
				       SYNAPSE_TOPIC_OPTICAL_FLOW_VELOCITY_KEY,
				       SYNAPSE_TOPIC_OPTICAL_FLOW_VELOCITY_CONTRACT);
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

static int zenoh_init(struct context *ctx)
{
	int ret = 0;

	zros_node_init(&ctx->node, "zenoh");

	ret = zros_sub_init(&ctx->sub_optical_flow, &ctx->node, &topic_optical_flow, &ctx->flow, 70);
	if (ret < 0) {
		LOG_ERR("init sub optical_flow failed: %d", ret);
		return ret;
	}

	ret = zros_sub_init(&ctx->sub_optical_flow_vel, &ctx->node, &topic_optical_flow_vel,
			    &ctx->flow_vel, 70);
	if (ret < 0) {
		LOG_ERR("init sub optical_flow_vel failed: %d", ret);
		return ret;
	}

	ret = zenoh_session_init(ctx);
	if (ret < 0) {
		return ret;
	}

	ret = zenoh_publishers_init(ctx);
	if (ret < 0) {
		return ret;
	}

	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
	return 0;
}

static int zenoh_fini(struct context *ctx)
{
	z_undeclare_publisher(z_move(ctx->pub_flow));
	z_undeclare_publisher(z_move(ctx->pub_flow_vel));
	zros_sub_fini(&ctx->sub_optical_flow);
	zros_sub_fini(&ctx->sub_optical_flow_vel);
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
		struct k_poll_event events[] = {
			*zros_sub_get_event(&ctx->sub_optical_flow),
			*zros_sub_get_event(&ctx->sub_optical_flow_vel),
		};

		int rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));

		if (rc != 0) {
			LOG_DBG("poll timeout");
		}

		if (zros_sub_update(&ctx->sub_optical_flow) == 0) {
			publish_struct(z_loan(ctx->pub_flow), &ctx->flow, sizeof(ctx->flow));
		}

		if (zros_sub_update(&ctx->sub_optical_flow_vel) == 0) {
			publish_struct(z_loan(ctx->pub_flow_vel), &ctx->flow_vel,
				       sizeof(ctx->flow_vel));
		}
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

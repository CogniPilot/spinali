/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zenoh publisher for processed optical flow data using FlatBuffers.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <zenoh-pico.h>

#include <synapse_topic_list.h>

#include "fb_pack.h"

LOG_MODULE_REGISTER(synapse_zenoh, CONFIG_SPINALI_SYNAPSE_ZENOH_LOG_LEVEL);

#define MY_STACK_SIZE 8192
#define MY_PRIORITY   5

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	struct zros_node node;
	/* subscriptions */
	struct zros_sub sub_vehicle_optical_flow;
	struct zros_sub sub_vehicle_optical_flow_vel;
	/* topic data */
	synapse_topic_VehicleOpticalFlowData_t flow;
	synapse_topic_VehicleOpticalFlowVelData_t flow_vel;
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
	.sub_vehicle_optical_flow = {},
	.sub_vehicle_optical_flow_vel = {},
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

static int zenoh_publishers_init(struct context *ctx)
{
	z_view_keyexpr_t ke;
	int ret;

	ret = z_view_keyexpr_from_str(&ke, "synapse/vehicle_optical_flow");
	if (ret < 0) {
		LOG_ERR("Invalid key expression for optical flow");
		return ret;
	}

	ret = z_declare_publisher(z_loan(ctx->session), &ctx->pub_flow, z_loan(ke), NULL);
	if (ret < 0) {
		LOG_ERR("Unable to declare flow publisher");
		return ret;
	}

	ret = z_view_keyexpr_from_str(&ke, "synapse/vehicle_optical_flow_vel");
	if (ret < 0) {
		LOG_ERR("Invalid key expression for optical flow vel");
		return ret;
	}

	ret = z_declare_publisher(z_loan(ctx->session), &ctx->pub_flow_vel, z_loan(ke), NULL);
	if (ret < 0) {
		LOG_ERR("Unable to declare flow vel publisher");
		return ret;
	}

	LOG_INF("Zenoh publishers declared");
	return 0;
}

static void publish_flow(struct context *ctx)
{
	uint8_t buf[FB_VEHICLE_OPTICAL_FLOW_SIZE];
	size_t len = fb_pack_vehicle_optical_flow(buf, sizeof(buf), &ctx->flow);

	if (len > 0) {
		z_owned_bytes_t payload;

		z_bytes_copy_from_buf(&payload, buf, len);
		z_publisher_put(z_loan(ctx->pub_flow), z_move(payload), NULL);
	}
}

static void publish_flow_vel(struct context *ctx)
{
	uint8_t buf[FB_VEHICLE_OPTICAL_FLOW_VEL_SIZE];
	size_t len = fb_pack_vehicle_optical_flow_vel(buf, sizeof(buf), &ctx->flow_vel);

	if (len > 0) {
		z_owned_bytes_t payload;

		z_bytes_copy_from_buf(&payload, buf, len);
		z_publisher_put(z_loan(ctx->pub_flow_vel), z_move(payload), NULL);
	}
}

static int zenoh_init(struct context *ctx)
{
	int ret = 0;

	zros_node_init(&ctx->node, "zenoh");

	ret = zros_sub_init(&ctx->sub_vehicle_optical_flow, &ctx->node,
			    &topic_vehicle_optical_flow, &ctx->flow, 70);
	if (ret < 0) {
		LOG_ERR("init sub vehicle_optical_flow failed: %d", ret);
		return ret;
	}

	ret = zros_sub_init(&ctx->sub_vehicle_optical_flow_vel, &ctx->node,
			    &topic_vehicle_optical_flow_vel, &ctx->flow_vel, 70);
	if (ret < 0) {
		LOG_ERR("init sub vehicle_optical_flow_vel failed: %d", ret);
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
	zros_sub_fini(&ctx->sub_vehicle_optical_flow);
	zros_sub_fini(&ctx->sub_vehicle_optical_flow_vel);
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
			*zros_sub_get_event(&ctx->sub_vehicle_optical_flow),
			*zros_sub_get_event(&ctx->sub_vehicle_optical_flow_vel),
		};

		int rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));

		if (rc != 0) {
			LOG_DBG("poll timeout");
		}

		if (zros_sub_update(&ctx->sub_vehicle_optical_flow) == 0) {
			publish_flow(ctx);
		}

		if (zros_sub_update(&ctx->sub_vehicle_optical_flow_vel) == 0) {
			publish_flow_vel(ctx);
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

/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>

#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <pb_encode.h>

#include <synapse_topic_list.h>

#define MY_STACK_SIZE 8192
#define MY_PRIORITY   1
#define TX_BUF_SIZE   8192

LOG_MODULE_REGISTER(zenoh, LOG_LEVEL_DBG);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	// zros node handle
	struct zros_node node;
	// subscriptions
	struct zros_sub sub_imu;
	// topic data
	synapse_pb_Frame tx_frame;
	synapse_pb_Imu imu;
	// connections
	// TODO Zenoh
	// status
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
};

static struct context g_ctx = {
	.node = {},
	.sub_imu = {},
	.imu = {},
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
};

static void send_frame(struct context *ctx, pb_size_t which_msg)
{
	synapse_pb_Frame *frame = &ctx->tx_frame;
	frame->which_msg = which_msg;
	if (which_msg == synapse_pb_Frame_imu_tag) {
		frame->msg.imu = ctx->imu;
	} else if (which_msg == synapse_pb_Frame_clock_offset_tag) {
		int64_t ticks = k_uptime_ticks();
		int64_t sec = ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
		int32_t nanosec = (ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 /
				  CONFIG_SYS_CLOCK_TICKS_PER_SEC;
		frame->msg.clock_offset.has_stamp = false;
		frame->msg.clock_offset.has_offset = true;
		frame->msg.clock_offset.offset.seconds = sec;
		frame->msg.clock_offset.offset.nanos = nanosec;
	}
	static uint8_t tx_buf[TX_BUF_SIZE];
	pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
	if (!pb_encode_ex(&stream, synapse_pb_Frame_fields, frame, PB_ENCODE_DELIMITED)) {
		LOG_ERR("encoding failed: %s", PB_GET_ERROR(&stream));
	} else {
		//udp_tx_send(&ctx->udp, tx_buf, stream.bytes_written);
		LOG_ERR("TODO SEND Zenoh size %d", stream.bytes_written);
		//TODO Zenoh
	}
}

static int zenoh_init(struct context *ctx)
{
	int ret = 0;
	// initialize node
	zros_node_init(&ctx->node, "zenoh");

	// initialize node subscriptions
	ret = zros_sub_init(&ctx->sub_imu, &ctx->node, &topic_imu, &ctx->imu, 15);
	if (ret < 0) {
		LOG_ERR("init imu failed: %d", ret);
		return ret;
	}

	// initialize Zenoh

	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
	return ret;
};

static int zenoh_fini(struct context *ctx)
{
	int ret = 0;
	// TODO CLose zenoh

	// close subscriptions
	zros_sub_fini(&ctx->sub_imu);
	zros_node_fini(&ctx->node);

	k_sem_give(&ctx->running);
	LOG_INF("fini");
	return ret;
};

static void zenoh_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	int ret = 0;

	// constructor
	ret = zenoh_init(ctx);
	if (ret < 0) {
		LOG_ERR("init failed: %d", ret);
		return;
	}

	int64_t ticks_last_uptime = 0;

	// subscribe to topics

	// while running
	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
		int64_t now = k_uptime_ticks();

		struct k_poll_event events[] = {
			*zros_sub_get_event(&ctx->sub_imu),
		};

		int rc = 0;
		rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
		if (rc != 0) {
			LOG_DBG("poll timeout");
		}

		if (zros_sub_update_available(&ctx->sub_imu)) {
			zros_sub_update(&ctx->sub_imu);
			send_frame(ctx, synapse_pb_Frame_imu_tag);
		}

		if (now - ticks_last_uptime > CONFIG_SYS_CLOCK_TICKS_PER_SEC) {
			send_frame(ctx, synapse_pb_Frame_clock_offset_tag);
			ticks_last_uptime = now;
		}
	}

	// deconstructor
	ret = zenoh_fini(ctx);
	if (ret < 0) {
		LOG_ERR("fini failed: %d", ret);
	}
};

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
		if (k_sem_count_get(&g_ctx.running) == 0) {
			shell_print(sh, "already running");
		} else {
			start(ctx);
		}
	} else if (strcmp(argv[0], "stop") == 0) {
		if (k_sem_count_get(&g_ctx.running) == 0) {
			k_sem_give(&g_ctx.running);
		} else {
			shell_print(sh, "not running");
		}
	} else if (strcmp(argv[0], "status") == 0) {
		shell_print(sh, "running: %d", (int)k_sem_count_get(&g_ctx.running) == 0);
	}
	return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_zenoh, zenoh_cmd_handler, (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"), (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(zenoh, &sub_zenoh, "zenoh commands", NULL);

static int zenoh_sys_init(void)
{
	return start(&g_ctx);
};

SYS_INIT(zenoh_sys_init, APPLICATION, 0);

// vi: ts=4 sw=4 et

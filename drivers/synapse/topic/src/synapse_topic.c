/*
 * Copyright (c) 2023 CogniPilot Foundation <cogni@cognipilot.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <math.h>
#include <stdio.h>
#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/private/zros_topic_struct.h>
#include <zros/zros_broker.h>
#include <zros/zros_common.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>
#include <zros/zros_topic.h>

LOG_MODULE_REGISTER(zros_topic);

#include "synapse_shell_print.h"

#define TOPIC_QUEUE_STACK_SIZE 8192
#define TOPIC_QUEUE_PRIORITY   10

K_THREAD_STACK_DEFINE(topic_queue_stack_area, TOPIC_QUEUE_STACK_SIZE);

struct k_work_q g_topic_work_q;
struct k_poll_signal signal_quit;

typedef int msg_handler_t(const struct shell *sh, struct zros_topic *topic, void *msg,
			  snprint_t *echo);
void topic_work_handler(struct k_work *work);

typedef struct context_t {
	struct k_work work_item;
	const struct shell *sh;
	struct zros_topic *topic;
	msg_handler_t *handler;
	struct k_mutex lock;
} context_t;

static context_t g_ctx = {.work_item = Z_WORK_INITIALIZER(topic_work_handler),
			  .sh = NULL,
			  .topic = NULL,
			  .handler = NULL,
			  .lock = Z_MUTEX_INITIALIZER(g_ctx.lock)};

#define TOPIC_DICTIONARY()                                                                         \
	(argus, &topic_argus, "argus"),                                                            \
	(imu0, &topic_imu0, "imu0"),                                                              \
	(optical_flow_raw, &topic_optical_flow_raw, "optical_flow_raw"),                           \
	(status, &topic_status, "status"),                                                         \
	(vehicle_optical_flow, &topic_vehicle_optical_flow, "vehicle_optical_flow"),                \
	(vehicle_optical_flow_vel, &topic_vehicle_optical_flow_vel, "vehicle_optical_flow_vel")

static void shell_callback(const struct shell *sh, uint8_t *data, size_t len)
{
	k_poll_signal_raise(&signal_quit, 1);
}

static int topic_count_hz(const struct shell *sh, struct zros_topic *topic, void *msg,
			  snprint_t *echo)
{
	struct zros_sub sub;
	struct zros_node node;
	zros_node_init(&node, "sub hz");
	zros_sub_init(&sub, &node, topic, msg, 1000);
	struct k_poll_event events[1] = {
		*zros_sub_get_event(&sub),
	};

	int64_t ticks_start = k_uptime_ticks();
	int64_t elapsed_ticks = 0;
	const int64_t ticks_sample = 5 * CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	int64_t ticks_remaining = ticks_sample;

	static const int max_msg = 11;
	int64_t msg_tick[max_msg];
	float sample_sec[max_msg - 1];

	int msg_count = 0;

	while (ticks_remaining > 0.1f * CONFIG_SYS_CLOCK_TICKS_PER_SEC && msg_count < max_msg) {
		int rc = 0;
		rc = k_poll(events, ARRAY_SIZE(events),
			    K_MSEC(1e3f * ticks_remaining / CONFIG_SYS_CLOCK_TICKS_PER_SEC));
		if (rc != 0) {
			char name[20];
			zros_topic_get_name(topic, name, sizeof(name));
			LOG_WRN("%s not published.", name);
		}

		elapsed_ticks = k_uptime_ticks() - ticks_start;
		ticks_remaining = ticks_sample - elapsed_ticks;

		if (zros_sub_update_available(&sub)) {
			rc = zros_sub_update(&sub);
			if (rc == 0) {
				msg_tick[msg_count] = k_uptime_ticks();
				msg_count++;
			} else {
				LOG_ERR("sub update failed");
			}
		}
	}

	float mean = 0.0f;
	float min = 0.0f;
	float max = 0.0f;

	shell_print(sh, "sample   delta");
	for (int i = 0; i < msg_count - 1; i++) {
		sample_sec[i] = (float)(msg_tick[i + 1] - msg_tick[i]) /
				(float)CONFIG_SYS_CLOCK_TICKS_PER_SEC;
		if (i == 0) {
			min = sample_sec[i];
			max = sample_sec[i];
		} else {
			if (sample_sec[i] < min) {
				min = sample_sec[i];
			} else if (sample_sec[i] > max) {
				max = sample_sec[i];
			}
		}
		mean += sample_sec[i];
		shell_print(sh, "  %d  %10.6fs", i, (double)sample_sec[i]);
	}

	mean /= (float)(msg_count - 1);

	float std = 0.0f;

	for (int i = 0; i < msg_count - 1; i++) {
		float v = sample_sec[i] - mean;
		std += v * v;
	}
	std = sqrtf(std / (float)(msg_count - 1));

	zros_sub_fini(&sub);
	zros_node_fini(&node);

	shell_print(sh,
		    "average rate: %8.3f Hz\n"
		    "min: %10.6fs, max: %10.6fs, std: %10.6fs, window: %d",
		    (double)(1.0f / mean), (double)min, (double)max, (double)std, msg_count);
	return ZROS_OK;
}

static int topic_echo(const struct shell *sh, struct zros_topic *topic, void *msg, snprint_t *echo)
{
	static char buf[2048] = {};
	struct zros_sub sub;
	struct zros_node node;
	zros_node_init(&node, "sub echo");
	zros_sub_init(&sub, &node, topic, msg, 10);

	k_poll_signal_init(&signal_quit);

	struct k_poll_event events[2] = {*zros_sub_get_event(&sub),
					 K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
								  K_POLL_MODE_NOTIFY_ONLY,
								  &signal_quit)};
	int rc = 0;

	shell_print(sh, "press any key to exit");
	shell_set_bypass(sh, shell_callback);

	while (true) {
		rc = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
		if (rc != 0) {
			shell_print(sh, "not published");
			break;
		} else {
			int quit_signaled, result;
			k_poll_signal_check(&signal_quit, &quit_signaled, &result);
			if (quit_signaled) {
				break;
			}
			if (!zros_sub_update_available(&sub)) {
				break;
			} else {
				zros_sub_update(&sub);
				echo(buf, sizeof(buf), msg);
				shell_print(sh, "%s", buf);
			}
		}
	}
	zros_sub_fini(&sub);
	zros_node_fini(&node);
	shell_set_bypass(sh, NULL);
	return ZROS_OK;
}

void topic_work_handler(struct k_work *work)
{
	context_t *ctx = CONTAINER_OF(work, context_t, work_item);

	ZROS_RC(k_mutex_lock(&ctx->lock, K_MSEC(1000)), LOG_ERR("topic handler busy\n"); return);

	const struct shell *sh = ctx->sh;
	struct zros_topic *topic = ctx->topic;
	msg_handler_t *handler = ctx->handler;

	if (topic == &topic_argus) {
		synapse_pb_ArgusResults msg = {};
		handler(sh, topic, &msg, (snprint_t *)&snprint_argus);
	} else if (topic == &topic_imu0) {
		synapse_pb_Imu msg = {};
		handler(sh, topic, &msg, (snprint_t *)&snprint_imu);
	} else if (topic == &topic_optical_flow_raw) {
		synapse_pb_PixartPAA3905 msg = {};
		handler(sh, topic, &msg, (snprint_t *)&snprint_pixart_paa3905);
	} else if (topic == &topic_status) {
		synapse_pb_Status msg = {};
		handler(sh, topic, &msg, (snprint_t *)&snprint_status);
	} else {
		char name[20];
		zros_topic_get_name(topic, name, sizeof(name));
		shell_print(sh, "%s: echo not implemented", name);
	}

	k_mutex_unlock(&ctx->lock);
	shell_print(sh, "");
}

static int cmd_zros_topic_hz(const struct shell *sh, size_t argc, char **argv, void *data)
{
	struct zros_topic *topic = (struct zros_topic *)data;
	g_ctx.sh = sh;
	g_ctx.handler = &topic_count_hz;
	g_ctx.topic = topic;
	return k_work_submit_to_queue(&g_topic_work_q, &g_ctx.work_item);
}

static int cmd_zros_topic_echo(const struct shell *sh, size_t argc, char **argv, void *data)
{
	struct zros_topic *topic = (struct zros_topic *)data;
	g_ctx.sh = sh;
	g_ctx.handler = &topic_echo;
	g_ctx.topic = topic;
	return k_work_submit_to_queue(&g_topic_work_q, &g_ctx.work_item);
}

void topic_print_iterator(const struct zros_topic *topic, void *data)
{
	const struct shell *sh = (const struct shell *)data;
	char name[50];
	zros_topic_get_name(topic, name, sizeof(name));
	shell_print(sh, "%s", name);
}

static int cmd_zros_topic_list(const struct shell *sh, size_t argc, char **argv)
{
	zros_broker_iterate_topic(topic_print_iterator, (void *)sh);
	return ZROS_OK;
}

void pub_print_iterator(const struct zros_pub *pub, void *data)
{
	const struct shell *sh = (const struct shell *)data;
	char name[50];
	zros_node_get_name(pub->_node, name, sizeof(name));
	shell_print(sh, "\t%s", name);
}

void sub_print_iterator(const struct zros_sub *sub, void *data)
{
	const struct shell *sh = (const struct shell *)data;
	char name[50];
	zros_node_get_name(sub->_node, name, sizeof(name));
	shell_print(sh, "\t%s", name);
}

static int cmd_zros_topic_info(const struct shell *sh, size_t argc, char **argv, void *data)
{
	struct zros_topic *topic = (struct zros_topic *)data;
	shell_print(sh, "pubs");
	zros_topic_iterate_pub(topic, pub_print_iterator, (void *)sh);
	shell_print(sh, "subs");
	zros_topic_iterate_sub(topic, sub_print_iterator, (void *)sh);
	return ZROS_OK;
}

void node_print_iterator(const struct zros_node *node, void *data)
{
	const struct shell *sh = (const struct shell *)data;
	char name[50];
	zros_node_get_name(node, name, sizeof(name));
	shell_print(sh, "%s", name);
}

static int cmd_zros_node_list(const struct shell *sh, size_t argc, char **argv)
{
	zros_broker_iterate_nodes(node_print_iterator, (void *)sh);
	return ZROS_OK;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_zros_topic_echo, cmd_zros_topic_echo, TOPIC_DICTIONARY());
SHELL_SUBCMD_DICT_SET_CREATE(sub_zros_topic_hz, cmd_zros_topic_hz, TOPIC_DICTIONARY());
SHELL_SUBCMD_DICT_SET_CREATE(sub_zros_topic_info, cmd_zros_topic_info, TOPIC_DICTIONARY());

SHELL_STATIC_SUBCMD_SET_CREATE(sub_zros_topic,
			       SHELL_CMD(echo, &sub_zros_topic_echo, "Echo topic.", NULL),
			       SHELL_CMD(hz, &sub_zros_topic_hz, "Check topic pub rate.", NULL),
			       SHELL_CMD(info, &sub_zros_topic_info, "Topic pubs and subs.", NULL),
			       SHELL_CMD(list, NULL, "List topics.", cmd_zros_topic_list),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_zros_node,
			       SHELL_CMD(list, NULL, "List nodes.", cmd_zros_node_list),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_zros, SHELL_CMD(topic, &sub_zros_topic, "Topic commands.", NULL),
			       SHELL_CMD(node, &sub_zros_node, "Node commands.", NULL),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(zros, &sub_zros, "ZROS Commands", NULL);

static int init_topic_queue(void)
{
	k_work_queue_init(&g_topic_work_q);
	struct k_work_queue_config topic_work_cfg = {.name = "synapse_topic_q", .no_yield = false};
	k_work_queue_start(&g_topic_work_q, topic_queue_stack_area,
			   K_THREAD_STACK_SIZEOF(topic_queue_stack_area), TOPIC_QUEUE_PRIORITY,
			   &topic_work_cfg);
	return 0;
};

SYS_INIT(init_topic_queue, POST_KERNEL, 0);

/* vi: ts=4 sw=4 et: */

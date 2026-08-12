/*
 * Copyright CogniPilot Foundation 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Per-node clock descriptor producer for the TimeReference topic.
 *
 * Publishes the relation between this node's timestamp_ns domain and absolute
 * time: one boot-clock sample carried onto the gPTP slave PHC timescale
 * through the shared time-status resolver, the clock quality the elected
 * grandmaster serves over Announce, and the TAI-to-UTC offset. Consumers use
 * it to place a stream's timestamps in absolute time and to judge how far
 * this node's clock can be trusted.
 *
 * Publication follows the catalog transition policy: a sample goes out
 * immediately on any time_status transition, and at a low base rate in
 * between so late joiners and loggers see the mapping without waiting for a
 * transition. The poll interval bounds transition detection latency.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/gptp.h>
#include <zephyr/shell/shell.h>

#include "ethernet/gptp/gptp_messages.h"
#include "ethernet/gptp/gptp_data_set.h"

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>

#include <synapse_time_status.h>
#include <synapse_topic_list.h>

LOG_MODULE_REGISTER(time_reference, LOG_LEVEL_INF);

#define MY_STACK_SIZE 2048
#define MY_PRIORITY   6

/* Transition detection latency bound, and the low base publish rate. */
#define TIME_REF_POLL_PERIOD_MS 100
#define TIME_REF_BASE_PERIOD_MS 1000

/* IEEE 802.1AS clockClass values published when not forwarding the served
 * grandmaster class: a coasting slave is its own holdover clock, and an
 * undisciplined node is a free-running one.
 */
#define TIME_REF_CLOCK_CLASS_HOLDOVER 7U
#define TIME_REF_CLOCK_CLASS_FREERUN  248U

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	struct zros_node node;
	/* publications */
	struct zros_pub pub_time_reference;
	/* publication data */
	synapse_topic_TimeReferenceData_t time_reference;
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
	.pub_time_reference = {},
	.time_reference = {},
	.time_ever_synced = false,
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
};

/*
 * Clock quality of the served time, learned over gPTP. The grandmaster's
 * announced clockClass sits in the elected grandmaster priority vector: on a
 * slave that is the vector received over Announce, and on the grandmaster
 * itself it is the local quality the PPS servo maintains through
 * gptp_update_gm_quality. The single byte is read without locking, the same
 * way the gPTP shell reads these data sets.
 */
static uint8_t served_clock_class(void)
{
	return GPTP_GLOBAL_DS()->gm_priority.root_system_id.clk_quality.clock_class;
}

/*
 * TAI-to-UTC offset of the served time, 0 while unknown. Validity follows the
 * announced time-property flags, which the port-role update copies from the
 * grandmaster's Announce on a slave and from the local system values on the
 * grandmaster itself. The offset value is taken from the slave port's received
 * Announce when one exists, since the global copy tracks only the local system
 * offset, and falls back to that system offset when this node is the
 * grandmaster and no slave port is present.
 */
static int16_t served_utc_offset_s(void)
{
	struct gptp_global_ds *global_ds = GPTP_GLOBAL_DS();
	int port;

	if ((global_ds->global_flags.octets[1] & GPTP_FLAG_CUR_UTC_OFF_VALID) == 0U) {
		return 0;
	}

	for (port = GPTP_PORT_START; port <= GPTP_PORT_END; port++) {
		if (global_ds->selected_role[port] == GPTP_PORT_SLAVE) {
			return GPTP_PORT_BMCA_DATA(port)->ann_current_utc_offset;
		}
	}

	return global_ds->sys_current_utc_offset;
}

/*
 * Sample the node clock and fill the descriptor.
 *
 * timestamp_ns is the node timebase, the same value a sensor producer would
 * stamp at this instant: monotonic boot time shifted by the boot-to-PHC
 * offset while on the shared timescale, bare boot time in freerun. On the
 * shared timescale that value is the TAI epoch itself, so time_tai_ns equals
 * timestamp_ns; in freerun both absolute fields stay 0, there is no absolute
 * claim to make. time_unix_ns is derived from TAI with the leap-second
 * offset, and stays 0 while that offset is unknown. uncertainty_ns stays 0:
 * no holdover estimator computes it yet.
 */
static void time_reference_sample(struct context *ctx)
{
	synapse_topic_TimeReferenceData_t *ref = &ctx->time_reference;
	int64_t time_offset_ns = 0;
	uint8_t time_status = synapse_time_status_resolve(&ctx->time_ever_synced, &time_offset_ns);
	int64_t boot_now_ns = k_ticks_to_ns_floor64(k_uptime_ticks());

	ref->timestamp_ns = (uint64_t)(boot_now_ns + time_offset_ns);

	if (time_status != SYNAPSE_TYPES_TIME_STATUS_LOCAL_FREERUN) {
		ref->time_tai_ns = ref->timestamp_ns;
		ref->utc_offset_s = served_utc_offset_s();
		ref->time_unix_ns =
			(ref->utc_offset_s != 0)
				? ref->time_tai_ns - (uint64_t)((int64_t)ref->utc_offset_s *
								1000000000LL)
				: 0U;
		/* while synced the served grandmaster class is forwarded; a
		 * coasting slave describes its own clock instead */
		ref->clock_class = (time_status == SYNAPSE_TYPES_TIME_STATUS_GPTP_SYNCED)
					   ? served_clock_class()
					   : TIME_REF_CLOCK_CLASS_HOLDOVER;
	} else {
		ref->time_tai_ns = 0U;
		ref->time_unix_ns = 0U;
		ref->utc_offset_s = 0;
		ref->clock_class = TIME_REF_CLOCK_CLASS_FREERUN;
	}

	ref->uncertainty_ns = 0U;
	ref->time_status = time_status;
	ref->domain = 0U; /* the 802.1AS stack serves gPTP domain 0 */
	ref->id = 0U;
}

static int time_reference_init(struct context *ctx)
{
	int ret;

	zros_node_init(&ctx->node, "time_reference");

	ret = zros_pub_init(&ctx->pub_time_reference, &ctx->node, &topic_time_reference,
			    &ctx->time_reference);
	if (ret < 0) {
		LOG_ERR("init pub time_reference failed: %d", ret);
		return ret;
	}

	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
	return 0;
}

static int time_reference_fini(struct context *ctx)
{
	zros_pub_fini(&ctx->pub_time_reference);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
	return 0;
}

static void time_reference_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	int ret = time_reference_init(ctx);

	if (ret < 0) {
		LOG_ERR("init failed: %d", ret);
		return;
	}

	/* sentinel: the first poll always publishes */
	uint8_t last_status = UINT8_MAX;
	int64_t last_pub_ms = 0;

	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
		time_reference_sample(ctx);

		int64_t now_ms = k_uptime_get();

		if (ctx->time_reference.time_status != last_status ||
		    (now_ms - last_pub_ms) >= TIME_REF_BASE_PERIOD_MS) {
			zros_pub_update(&ctx->pub_time_reference);
			if (ctx->time_reference.time_status != last_status) {
				LOG_INF("time_status %d -> %d, clock_class %d", last_status,
					ctx->time_reference.time_status,
					ctx->time_reference.clock_class);
			}
			last_status = ctx->time_reference.time_status;
			last_pub_ms = now_ms;
		}

		k_msleep(TIME_REF_POLL_PERIOD_MS);
	}

	time_reference_fini(ctx);
}

static int start(struct context *ctx)
{
	k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				      time_reference_run, ctx, NULL, NULL, MY_PRIORITY, 0,
				      K_FOREVER);

	k_thread_name_set(tid, "time_reference");
	k_thread_start(tid);
	return 0;
}

static int time_reference_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_time_reference, time_reference_cmd_handler,
			     (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"),
			     (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(time_reference, &sub_time_reference, "time reference commands", NULL);

static int time_reference_sys_init(void)
{
	return start(&g_ctx);
}

SYS_INIT(time_reference_sys_init, APPLICATION, 91);

/* vi: ts=4 sw=4 et */

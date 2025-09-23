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

#include <zenoh-pico.h>

#include <pb_encode.h>

#include <synapse_topic_list.h>
#include <dds_serializer.h>

#include <synapse_msgs.h>

#define CDR_SAFETY_MARGIN 12

#define MY_STACK_SIZE 8192
#define MY_PRIORITY   1
#define TX_BUF_SIZE   8192

#define NET_MODE_SIZE    sizeof("client")
#define NET_LOCATOR_SIZE 64
#define KEYEXPR_RIHS01_SIZE                                                                        \
	sizeof("RIHS01_XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
#define KEYEXPR_MSG_NAME      "synapse_msgs::msg::dds_::"
#define KEYEXPR_MSG_NAME_SIZE sizeof(KEYEXPR_MSG_NAME)
#define TOPIC_INFO_SIZE       (96)
#define MAX_LINE_SIZE         (2 * TOPIC_INFO_SIZE)
#define KEYEXPR_SIZE          (MAX_LINE_SIZE + KEYEXPR_MSG_NAME_SIZE + KEYEXPR_RIHS01_SIZE + 128)

/* Derived from ROS2 rmw
 * https://github.com/ros2/rmw/blob/e6addf2411b8ee8a2ac43d691533b8c05ae8f1b6/rmw/include/rmw/types.h#L44
 */
#define RMW_GID_STORAGE_SIZE 16u

/* See rmw_zenoh design.md for more information
 * https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md#publishers */
#define RMW_ATTACHEMENT_SIZE (8u + 8u + 1u + RMW_GID_STORAGE_SIZE)

typedef struct __attribute__((__packed__)) RmwAttachment {
	int64_t sequence_number;
	int64_t time;
	uint8_t rmw_gid_size;
	uint8_t rmw_gid[RMW_GID_STORAGE_SIZE];
} RmwAttachment;

RmwAttachment _attachment = {0, 0, RMW_GID_STORAGE_SIZE};

LOG_MODULE_REGISTER(zenoh, LOG_LEVEL_DBG);

// FIXME proper GUID
static uint8_t zenoh_guid[16];
static const uint16_t domain_id = 7;

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
	z_owned_session_t s;
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

struct synapse_zenoh_publisher;

typedef void (*synapse_pub_conv_cb_t)(struct synapse_zenoh_publisher *zp, struct context *ctx,
				      pb_size_t which_msg);

typedef struct synapse_zenoh_publisher {
	const char *topic_name;
	synapse_msg *msg_type;
	const pb_size_t pb_tag;
	synapse_pub_conv_cb_t callback;
	z_owned_publisher_t pub;
} synapse_zenoh_publisher;

void imu_convert_and_publish(synapse_zenoh_publisher *zp, struct context *ctx, pb_size_t which_msg)
{
	synapse_pb_Frame *frame = &ctx->tx_frame;
	synapse_msgs_msg_Imu imu_data;
	uint8_t buf[sizeof(ros2_header) + sizeof(synapse_msgs_msg_Imu) + CDR_SAFETY_MARGIN];

	memcpy(buf, ros2_header, sizeof(ros2_header));

	strcpy(imu_data.header.frame_id, frame->msg.imu.frame_id);
	int64_t ticks = k_uptime_ticks();
	imu_data.header.stamp.sec = ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	imu_data.header.stamp.nanosec =
		(ticks - imu_data.header.stamp.sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 /
		CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	imu_data.x = frame->msg.imu.orientation.x;
	imu_data.y = frame->msg.imu.orientation.y;
	imu_data.z = frame->msg.imu.orientation.z;

	// THIS CODE NEEDS AN HELPER

	dds_ostream_t os;
	os.m_buffer = &buf[4];
	os.m_index = 0;
	os.m_size =
		(uint32_t)sizeof(ros2_header) + sizeof(synapse_msgs_msg_Imu) + CDR_SAFETY_MARGIN;
	os.m_xcdr_version = DDSI_RTPS_CDR_ENC_VERSION_1;

	z_publisher_put_options_t options;
	z_publisher_put_options_default(&options);

	_attachment.sequence_number++;
	_attachment.time = 0; // FIXME

	z_owned_bytes_t z_attachment;
	z_bytes_from_static_buf(&z_attachment, (const uint8_t *)&_attachment, RMW_ATTACHEMENT_SIZE);

	options.attachment = z_move(z_attachment);

	if (dds_stream_write(&os, &dds_allocator, (const char *)&imu_data,
			     synapse_msgs_msg_imu.desc->ops.ops)) {
		z_owned_bytes_t payload;
		z_bytes_copy_from_buf(&payload, buf, os.m_size);
		z_publisher_put(z_loan(zp->pub), z_move(payload), &options);
	}
}

synapse_zenoh_publisher publishers[] = {
	{"/imu", &synapse_msgs_msg_imu, synapse_pb_Frame_imu_tag, imu_convert_and_publish},
};

#define NUM_PUBLISHERS (sizeof(publishers) / sizeof(publishers[0]))

static void send_frame(struct context *ctx, pb_size_t which_msg)
{
	for (int i = 0; i < NUM_PUBLISHERS; i++) {
		if (publishers[i].pb_tag == which_msg) {
			publishers[i].callback(&publishers[i], ctx, which_msg);
		}
	}

	// CHEAT
	publishers[0].callback(&publishers[0], ctx, which_msg);
}

static int generate_rmw_zenoh_node_liveliness_keyexpr(const z_id_t *id, char *keyexpr)
{
	return snprintf(
		keyexpr, KEYEXPR_SIZE,
		"@ros2_lv/0/%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x/0/0/"
		"NN/%%/%%/"
		"spinali_%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
		id->id[0], id->id[1], id->id[2], id->id[3], id->id[4], id->id[5], id->id[6],
		id->id[7], id->id[8], id->id[9], id->id[10], id->id[11], id->id[12], id->id[13],
		id->id[14], id->id[15], zenoh_guid[0], zenoh_guid[1], zenoh_guid[2], zenoh_guid[3],
		zenoh_guid[4], zenoh_guid[5], zenoh_guid[6], zenoh_guid[7], zenoh_guid[8],
		zenoh_guid[9], zenoh_guid[10], zenoh_guid[11], zenoh_guid[12], zenoh_guid[13],
		zenoh_guid[14], zenoh_guid[15]);
}

int generate_rmw_zenoh_topic_keyexpr(const char *topic, const uint8_t *rihs_hash,
				     char *type_camel_case, char *keyexpr)
{
	return snprintf(keyexpr, KEYEXPR_SIZE,
			"%" PRId32 "%s/%s_/RIHS01_"
			"%02x%02x%02x%02x%02x%02x%02x%02x"
			"%02x%02x%02x%02x%02x%02x%02x%02x"
			"%02x%02x%02x%02x%02x%02x%02x%02x"
			"%02x%02x%02x%02x%02x%02x%02x%02x",
			domain_id, topic, type_camel_case, rihs_hash[0], rihs_hash[1], rihs_hash[2],
			rihs_hash[3], rihs_hash[4], rihs_hash[5], rihs_hash[6], rihs_hash[7],
			rihs_hash[8], rihs_hash[9], rihs_hash[10], rihs_hash[11], rihs_hash[12],
			rihs_hash[13], rihs_hash[14], rihs_hash[15], rihs_hash[16], rihs_hash[17],
			rihs_hash[18], rihs_hash[19], rihs_hash[20], rihs_hash[21], rihs_hash[22],
			rihs_hash[23], rihs_hash[24], rihs_hash[25], rihs_hash[26], rihs_hash[27],
			rihs_hash[28], rihs_hash[29], rihs_hash[30], rihs_hash[31]);
}

int generate_rmw_zenoh_topic_liveliness_keyexpr(const z_id_t *id, const char *topic,
						const uint8_t *rihs_hash, char *type_camel_case,
						char *keyexpr, const char *entity_str)
{
	// NOT REALLY COMPLIANT WITH RMW_ZENOH_CPP but get's the job done
	// TODO build a correct keyexpr

	char topic_lv[TOPIC_INFO_SIZE];
	char *str = &topic_lv[0];

	strncpy(topic_lv, topic, sizeof(topic_lv));

	while (*str) {
		if (*str == '/') {
			*str = '%';
		}

		str++;
	}

	return snprintf(keyexpr, KEYEXPR_SIZE,
			"@ros2_lv/%" PRId32 "/"
			"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x/"
			"0/11/%s/%%/%%/"
			"spinali_%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x/"
			"%s/%s_/RIHS01_"
			"%02x%02x%02x%02x%02x%02x%02x%02x"
			"%02x%02x%02x%02x%02x%02x%02x%02x"
			"%02x%02x%02x%02x%02x%02x%02x%02x"
			"%02x%02x%02x%02x%02x%02x%02x%02x"
			"/::,7:,:,:,,",
			domain_id, id->id[0], id->id[1], id->id[2], id->id[3], id->id[4], id->id[5],
			id->id[6], id->id[7], id->id[8], id->id[9], id->id[10], id->id[11],
			id->id[12], id->id[13], id->id[14], id->id[15], entity_str, zenoh_guid[0],
			zenoh_guid[1], zenoh_guid[2], zenoh_guid[3], zenoh_guid[4], zenoh_guid[5],
			zenoh_guid[6], zenoh_guid[7], zenoh_guid[8], zenoh_guid[9], zenoh_guid[10],
			zenoh_guid[11], zenoh_guid[12], zenoh_guid[13], zenoh_guid[14],
			zenoh_guid[15], topic_lv, type_camel_case, rihs_hash[0], rihs_hash[1],
			rihs_hash[2], rihs_hash[3], rihs_hash[4], rihs_hash[5], rihs_hash[6],
			rihs_hash[7], rihs_hash[8], rihs_hash[9], rihs_hash[10], rihs_hash[11],
			rihs_hash[12], rihs_hash[13], rihs_hash[14], rihs_hash[15], rihs_hash[16],
			rihs_hash[17], rihs_hash[18], rihs_hash[19], rihs_hash[20], rihs_hash[21],
			rihs_hash[22], rihs_hash[23], rihs_hash[24], rihs_hash[25], rihs_hash[26],
			rihs_hash[27], rihs_hash[28], rihs_hash[29], rihs_hash[30], rihs_hash[31]);
}

static int zenoh_liveliness_init(struct context *ctx)
{
	char keyexpr[KEYEXPR_SIZE];

	z_id_t self_id = z_info_zid(z_loan(ctx->s));

	if (generate_rmw_zenoh_node_liveliness_keyexpr(&self_id, keyexpr)) {

		z_view_keyexpr_t ke;

		if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
			LOG_ERR("%s is not a valid key expression", keyexpr);
			return -1;
		}

		z_owned_liveliness_token_t token;

		if (z_liveliness_declare_token(z_loan(ctx->s), &token, z_loan(ke), NULL) < 0) {
			LOG_ERR("Unable to create liveliness token!");
			return -1;
		}
	}

	for (int i = 0; i < NUM_PUBLISHERS; i++) {
		z_view_keyexpr_t ke;
		synapse_zenoh_publisher *pub = &publishers[i];

		generate_rmw_zenoh_topic_liveliness_keyexpr(&self_id, pub->topic_name,
							    pub->msg_type->rihs_hash,
							    pub->msg_type->msg_name, keyexpr, "MP");

		if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
			LOG_ERR("%s is not a valid key expression\n", keyexpr);
			return -1;
		}

		z_owned_liveliness_token_t token;

		if (z_liveliness_declare_token(z_loan(ctx->s), &token, z_loan(ke), NULL) < 0) {
			LOG_ERR("Unable to create liveliness token!\n");
			return -1;
		}

		generate_rmw_zenoh_topic_keyexpr(pub->topic_name, pub->msg_type->rihs_hash,
						 pub->msg_type->msg_name, keyexpr);

		if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
			LOG_ERR("%s is not a valid key expression", keyexpr);
			return -1;
		}

		if (z_declare_publisher(z_loan(ctx->s), &pub->pub, z_loan(ke), NULL) < 0) {
			LOG_ERR("Unable to declare publisher for key expression!");
			return -1;
		}
	}

	return 0;
}

static int zenoh_session_init(struct context *ctx)
{
	char mode[NET_MODE_SIZE];
	char locator[NET_LOCATOR_SIZE];
	z_owned_config_t config;
	int ret = 0;

	// TODO DYNAMIC
	strcpy(mode, "client");
	strcpy(locator, "tcp/192.0.2.2:7447");

	LOG_INF("Opening session...");

	do {
		z_config_default(&config);
		zp_config_insert(z_loan_mut(config), Z_CONFIG_MODE_KEY, mode);

		if (locator[0] != 0) {
			zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY, locator);

		} else if (strcmp(Z_CONFIG_MODE_PEER, mode) == 0) {
			zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY,
					 Z_CONFIG_MULTICAST_LOCATOR_DEFAULT);
		}

		if (ret == _Z_ERR_TRANSPORT_OPEN_FAILED) {
			LOG_WRN("Unable to open session, make sure zenohd is running on %s",
				locator);

		} else if (ret == _Z_ERR_SCOUT_NO_RESULTS) {
			LOG_WRN("Unable to open session, scout no results");

		} else if (ret < 0) {
			LOG_WRN("Unable to open session, ret: %d", ret);
		}

		if (ret != 0) {
			sleep(5); // Wait 5 seconds when doing a retry
		}

	} while ((ret = z_open(&ctx->s, z_move(config), NULL)) < 0);

	// Start read and lease tasks for zenoh-pico
	if (zp_start_read_task(z_loan_mut(ctx->s), NULL) < 0 ||
	    zp_start_lease_task(z_loan_mut(ctx->s), NULL) < 0) {
		LOG_ERR("Unable to start read and lease tasks");
		z_drop(z_move(ctx->s));
		ret = -EINVAL;
	}

	zenoh_liveliness_init(ctx);

	return ret;
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
	zenoh_session_init(ctx);

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

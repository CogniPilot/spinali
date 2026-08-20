/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_linkaddr.h>
#include <zephyr/sys/printk.h>

#include "ipv6.h"

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <zenoh-pico.h>

#include <synapse/cdr_catalog.h>

#include <synapse_time_status.h>
#include <synapse_topic_list.h>

LOG_MODULE_REGISTER(synapse_ros_cdr, CONFIG_SPINALI_SYNAPSE_ROS_CDR_LOG_LEVEL);

#if defined(CONFIG_SPINALI_SYNAPSE_ROS_CDR_TOPIC_GNSS_FIX)
typedef synapse_topic_GnssFix_t ros_cdr_sample_t;
#define ROS_CDR_NODE_ID     11
#define ROS_CDR_GID_TOPIC_TAG 0x01U
#define ROS_CDR_TOPIC_KEY                                                                    \
	"0/synapse/gnss_fix/synapse_msgs::msg::dds_::GnssFix_/"                              \
	"RIHS01_ac8d665c1bf6f81796d95bdd6a2285537bbfcb34869ba0e042f8ce24f75d9f0e"
#define ROS_CDR_NODE_TOKEN_SUFFIX                                                            \
	"/0/0/NN/%synapse%rtk_gnss/%synapse/rtk_gnss"
#define ROS_CDR_PUBLISHER_TOKEN_SUFFIX                                                       \
	"/0/1/MP/%synapse%rtk_gnss/%synapse/rtk_gnss/"                                      \
	"%synapse%gnss_fix/synapse_msgs::msg::dds_::GnssFix_/"                              \
	"RIHS01_ac8d665c1bf6f81796d95bdd6a2285537bbfcb34869ba0e042f8ce24f75d9f0e/"         \
	"2::,1:,:,:,,"
#define ROS_CDR_TOPIC       topic_nav_sat_fix
#define ROS_CDR_TOPIC_NAME  "GnssFix"
#define ROS_CDR_TOTAL_BYTES SYNAPSE_CDR_GNSS_FIX_TOTAL_BYTES
#else
typedef synapse_topic_OpticalFlowVelocityData_t ros_cdr_sample_t;
#define ROS_CDR_NODE_ID     12
#define ROS_CDR_GID_TOPIC_TAG 0x02U
#define ROS_CDR_TOPIC_KEY                                                                    \
	"0/synapse/optical_flow_velocity/synapse_msgs::msg::dds_::OpticalFlowVelocity_/"       \
	"RIHS01_8f46bb3da905598105f99e502394842afa66d849de841143565a193074829d09"
#define ROS_CDR_NODE_TOKEN_SUFFIX                                                            \
	"/0/0/NN/%synapse%optical_flow/%synapse/optical_flow"
#define ROS_CDR_PUBLISHER_TOKEN_SUFFIX                                                       \
	"/0/1/MP/%synapse%optical_flow/%synapse/optical_flow/"                               \
	"%synapse%optical_flow_velocity/synapse_msgs::msg::dds_::OpticalFlowVelocity_/"       \
	"RIHS01_8f46bb3da905598105f99e502394842afa66d849de841143565a193074829d09/"           \
	"2::,1:,:,:,,"
#define ROS_CDR_TOPIC       topic_optical_flow_vel
#define ROS_CDR_TOPIC_NAME  "OpticalFlowVelocity"
#define ROS_CDR_TOTAL_BYTES SYNAPSE_CDR_OPTICAL_FLOW_VELOCITY_TOTAL_BYTES
#endif

#define ATTACHMENT_SIZE 33U
#define SESSION_RETRY_MS 1000

struct ros_cdr_context {
	struct zros_node node;
	struct zros_sub sub;
	ros_cdr_sample_t sample;
	z_owned_session_t session;
	z_owned_liveliness_token_t node_token;
	z_owned_liveliness_token_t publisher_token;
	z_owned_publisher_t publisher;
	struct k_thread thread;
	int64_t publication_sequence;
	uint32_t published;
	uint32_t failed;
	bool time_ever_synced;
	char zid_str[33];
	char node_token_key[128];
	char publisher_token_key[320];
	uint8_t publisher_gid[16];
};

static K_THREAD_STACK_DEFINE(g_stack, CONFIG_SPINALI_SYNAPSE_ROS_CDR_THREAD_STACK_SIZE);
static struct ros_cdr_context g_ctx = {
	.publication_sequence = 1,
};

static void store_i64_le(uint8_t *bytes, int64_t value)
{
	uint64_t bits = (uint64_t)value;

	for (size_t index = 0U; index < sizeof(bits); ++index) {
		bytes[index] = (uint8_t)(bits >> (8U * index));
	}
}

static struct net_if *application_iface_get(void)
{
	struct net_if *vlan = net_eth_get_vlan_iface(NULL, CONFIG_SPINALI_SYNAPSE_ROS_CDR_VLAN_ID);
	struct net_if *base;
	int result;

	if (vlan == NULL) {
		base = net_if_get_default();
		if (base == NULL) {
			return NULL;
		}
		if (net_eth_is_vlan_interface(base)) {
			base = net_eth_get_vlan_main(base);
		}
		if (base == NULL || !net_if_is_up(base)) {
			return NULL;
		}

		result = net_eth_vlan_enable(base, CONFIG_SPINALI_SYNAPSE_ROS_CDR_VLAN_ID);
		if (result != 0 && result != -EALREADY) {
			LOG_ERR("VLAN %d enable failed: %d", CONFIG_SPINALI_SYNAPSE_ROS_CDR_VLAN_ID,
				result);
			return NULL;
		}
		vlan = net_eth_get_vlan_iface(base, CONFIG_SPINALI_SYNAPSE_ROS_CDR_VLAN_ID);
		if (vlan == NULL) {
			vlan = net_eth_get_vlan_iface(NULL, CONFIG_SPINALI_SYNAPSE_ROS_CDR_VLAN_ID);
		}
	}

	if (vlan == NULL || (!net_if_is_up(vlan) && net_if_up(vlan) != 0)) {
		return NULL;
	}

	net_if_set_default(vlan);
	return vlan;
}

static int router_neighbor_add(struct net_if *iface)
{
	struct net_in6_addr router;
	uint8_t address[NET_ETH_ADDR_LEN];
	struct net_linkaddr link_address;

	if (net_addr_pton(AF_INET6, CONFIG_SPINALI_SYNAPSE_ROS_CDR_ROUTER_ADDRESS,
			  &router) != 0 ||
	    net_bytes_from_str(address, (int)sizeof(address),
			       CONFIG_SPINALI_SYNAPSE_ROS_CDR_ROUTER_MAC) < 0 ||
	    net_linkaddr_create(&link_address, address, (uint8_t)sizeof(address),
				NET_LINK_ETHERNET) < 0) {
		return -EINVAL;
	}

	return net_ipv6_nbr_add(iface, &router, &link_address, false,
				NET_IPV6_NBR_STATE_STATIC) == NULL
		       ? -ENOMEM
		       : 0;
}

/*
 * Derive this node's Zenoh identity from its role, matching the locally
 * administered MAC convention 02:04:9f:00:hh:ll where the low 12 bits spell the
 * decimal node id followed by a hex zero (node 11 -> 0x110 -> 02:04:9f:00:01:10).
 *
 * The session ZID and publisher GID are built from the node's EUI-64 interface
 * identifier 00:04:9f:ff:fe:00:hh:ll. The 32-character lowercase-hex ZID places
 * the IID in the low 8 bytes and zero in the high 8 bytes. The zenoh-pico parser
 * reads the string right-to-left, little-endian (the exact inverse of
 * id-to-string), so printing the IID bytes in forward order after 16 leading
 * zeros yields a string the opened session reports back unchanged. The publisher
 * GID reuses the same IID root with a per-topic tag so every node and topic pair
 * advertises a distinct, reproducible identity with no captured rmw_zenoh dump.
 */
static void role_identity_init(struct ros_cdr_context *ctx)
{
	uint8_t bcd = (uint8_t)(((ROS_CDR_NODE_ID / 10) << 4) | (ROS_CDR_NODE_ID % 10));
	uint16_t field = (uint16_t)((uint16_t)bcd << 4);
	uint8_t hh = (uint8_t)(field >> 8);
	uint8_t ll = (uint8_t)(field & 0xffU);
	const uint8_t iid[8] = {0x00U, 0x04U, 0x9fU, 0xffU, 0xfeU, 0x00U, hh, ll};

	(void)snprintk(ctx->zid_str, sizeof(ctx->zid_str),
		       "%016x%02x%02x%02x%02x%02x%02x%02x%02x", 0U, iid[0], iid[1],
		       iid[2], iid[3], iid[4], iid[5], iid[6], iid[7]);

	(void)snprintk(ctx->node_token_key, sizeof(ctx->node_token_key), "@ros2_lv/0/%s%s",
		       ctx->zid_str, ROS_CDR_NODE_TOKEN_SUFFIX);
	(void)snprintk(ctx->publisher_token_key, sizeof(ctx->publisher_token_key),
		       "@ros2_lv/0/%s%s", ctx->zid_str, ROS_CDR_PUBLISHER_TOKEN_SUFFIX);

	memcpy(ctx->publisher_gid, iid, sizeof(iid));
	ctx->publisher_gid[8] = ROS_CDR_GID_TOPIC_TAG;
	memset(&ctx->publisher_gid[9], 0, sizeof(ctx->publisher_gid) - 9U);
}

static int token_declare(struct ros_cdr_context *ctx, const char *key,
			 z_owned_liveliness_token_t *token)
{
	z_view_keyexpr_t keyexpr;
	int result = z_view_keyexpr_from_str(&keyexpr, key);

	if (result < 0) {
		return result;
	}
	return z_liveliness_declare_token(z_loan(ctx->session), token, z_loan(keyexpr), NULL);
}

static int transport_open(struct ros_cdr_context *ctx)
{
	struct net_if *iface = application_iface_get();
	z_owned_config_t config;
	z_view_keyexpr_t topic_keyexpr;
	z_publisher_options_t publisher_options;
	int result;

	if (iface == NULL || router_neighbor_add(iface) < 0) {
		return -ENETDOWN;
	}

	result = z_config_default(&config);
	if (result < 0) {
		return result;
	}
	if (zp_config_insert(z_loan_mut(config), Z_CONFIG_SESSION_ZID_KEY,
			     ctx->zid_str) < 0 ||
	    zp_config_insert(z_loan_mut(config), Z_CONFIG_MODE_KEY, "client") < 0 ||
	    zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY,
			     CONFIG_SPINALI_SYNAPSE_ROS_CDR_ROUTER) < 0 ||
	    zp_config_insert(z_loan_mut(config), Z_CONFIG_MULTICAST_SCOUTING_KEY, "false") < 0) {
		z_drop(z_move(config));
		return -EINVAL;
	}

	result = z_open(&ctx->session, z_move(config), NULL);
	if (result < 0) {
		return result;
	}

	result = token_declare(ctx, ctx->node_token_key, &ctx->node_token);
	if (result < 0) {
		z_drop(z_move(ctx->session));
		return result;
	}
	result = token_declare(ctx, ctx->publisher_token_key, &ctx->publisher_token);
	if (result < 0) {
		z_drop(z_move(ctx->node_token));
		z_drop(z_move(ctx->session));
		return result;
	}
	result = z_view_keyexpr_from_str(&topic_keyexpr, ROS_CDR_TOPIC_KEY);
	if (result < 0) {
		z_drop(z_move(ctx->publisher_token));
		z_drop(z_move(ctx->node_token));
		z_drop(z_move(ctx->session));
		return result;
	}

	z_publisher_options_default(&publisher_options);
	publisher_options.reliability = Z_RELIABILITY_BEST_EFFORT;
	result = z_declare_publisher(z_loan(ctx->session), &ctx->publisher,
				     z_loan(topic_keyexpr), &publisher_options);
	if (result < 0) {
		z_drop(z_move(ctx->publisher_token));
		z_drop(z_move(ctx->node_token));
		z_drop(z_move(ctx->session));
		return result;
	}

	LOG_INF("ROS CDR mirror connected on VLAN %d iface %u",
		CONFIG_SPINALI_SYNAPSE_ROS_CDR_VLAN_ID, net_if_get_by_iface(iface));
	return 0;
}

static int64_t publication_timestamp_ns(struct ros_cdr_context *ctx)
{
	int64_t offset_ns = 0;
	uint8_t status = synapse_time_status_resolve(&ctx->time_ever_synced, &offset_ns);

	if (status == SYNAPSE_TYPES_TIME_STATUS_LOCAL_FREERUN) {
		return 0;
	}
	return (int64_t)k_ticks_to_ns_floor64(k_uptime_ticks()) + offset_ns;
}

static int sample_encode(struct ros_cdr_context *ctx, uint8_t *cdr, size_t capacity,
			 size_t *written)
{
#if defined(CONFIG_SPINALI_SYNAPSE_ROS_CDR_TOPIC_GNSS_FIX)
	const synapse_cdr_gnss_fix_t value = {
		.timestamp_ns = ctx->sample.timestamp_ns,
		.time_unix_ns = ctx->sample.time_unix_ns,
		.latitude_deg_e7 = ctx->sample.latitude_deg_e7,
		.longitude_deg_e7 = ctx->sample.longitude_deg_e7,
		.altitude_msl_mm = ctx->sample.altitude_msl_mm,
		.altitude_ellipsoid_mm = ctx->sample.altitude_ellipsoid_mm,
		.horizontal_accuracy_mm = ctx->sample.horizontal_accuracy_mm,
		.vertical_accuracy_mm = ctx->sample.vertical_accuracy_mm,
		.velocity_accuracy_mm_s = ctx->sample.velocity_accuracy_mm_s,
		.yaw_accuracy_cdeg = ctx->sample.yaw_accuracy_cdeg,
		.hdop_centi = ctx->sample.hdop_centi,
		.vdop_centi = ctx->sample.vdop_centi,
		.ground_speed_cm_s = ctx->sample.ground_speed_cm_s,
		.course_over_ground_cdeg = ctx->sample.course_over_ground_cdeg,
		.yaw_cdeg = ctx->sample.yaw_cdeg,
		.velocity_up_cm_s = ctx->sample.velocity_up_cm_s,
		.flags = ctx->sample.flags,
		.fix_type = ctx->sample.fix_type,
		.satellites_used = ctx->sample.satellites_used,
		.satellites_visible = ctx->sample.satellites_visible,
		.time_status = ctx->sample.time_status,
		.id = ctx->sample.id,
	};

	return synapse_cdr_encode_gnss_fix(&value, cdr, capacity, written);
#else
	synapse_cdr_optical_flow_velocity_t value = {
		.timestamp_ns = ctx->sample.timestamp_ns,
		.velocity_flu_m_s = {
			ctx->sample.velocity_flu_m_s.x,
			ctx->sample.velocity_flu_m_s.y,
		},
		.distance_m = ctx->sample.distance_m,
		.roll_rad = ctx->sample.roll_rad,
		.pitch_rad = ctx->sample.pitch_rad,
		.quality = ctx->sample.quality,
		.flags = ctx->sample.flags,
		.time_status = ctx->sample.time_status,
		.id = ctx->sample.id,
	};

	return synapse_cdr_encode_optical_flow_velocity(&value, cdr, capacity, written);
#endif
}

static int sample_publish(struct ros_cdr_context *ctx)
{
	uint8_t cdr[ROS_CDR_TOTAL_BYTES];
	uint8_t attachment[ATTACHMENT_SIZE];
	size_t written = 0U;
	z_owned_bytes_t payload;
	z_owned_bytes_t attachment_bytes;
	z_put_options_t options;
	z_view_keyexpr_t topic_keyexpr;
	int result;

	result = sample_encode(ctx, cdr, sizeof(cdr), &written);
	if (result != SYNAPSE_CDR_OK || written != sizeof(cdr)) {
		return -EINVAL;
	}

	store_i64_le(&attachment[0], ctx->publication_sequence++);
	store_i64_le(&attachment[8], publication_timestamp_ns(ctx));
	attachment[16] = (uint8_t)sizeof(ctx->publisher_gid);
	memcpy(&attachment[17], ctx->publisher_gid, sizeof(ctx->publisher_gid));

	result = z_bytes_copy_from_buf(&payload, cdr, sizeof(cdr));
	if (result < 0) {
		return result;
	}
	result = z_bytes_copy_from_buf(&attachment_bytes, attachment, sizeof(attachment));
	if (result < 0) {
		z_drop(z_move(payload));
		return result;
	}

	result = z_view_keyexpr_from_str(&topic_keyexpr, ROS_CDR_TOPIC_KEY);
	if (result < 0) {
		z_drop(z_move(attachment_bytes));
		z_drop(z_move(payload));
		return result;
	}

	z_put_options_default(&options);
	options.reliability = Z_RELIABILITY_BEST_EFFORT;
	options.attachment = z_move(attachment_bytes);
	return z_put(z_loan(ctx->session), z_loan(topic_keyexpr), z_move(payload), &options);
}

static void ros_cdr_run(void *first, void *second, void *third)
{
	struct ros_cdr_context *ctx = first;
	int result;

	ARG_UNUSED(second);
	ARG_UNUSED(third);

	zros_node_init(&ctx->node, "synapse_ros_cdr");
	result = zros_sub_init(&ctx->sub, &ctx->node, &ROS_CDR_TOPIC, &ctx->sample,
			       CONFIG_SPINALI_SYNAPSE_ROS_CDR_RATE_HZ);
	if (result < 0) {
		LOG_ERR("%s subscription failed: %d", ROS_CDR_TOPIC_NAME, result);
		return;
	}

	role_identity_init(ctx);

	while ((result = transport_open(ctx)) < 0) {
		LOG_WRN("Zenoh router unavailable: %d", result);
		k_sleep(K_MSEC(SESSION_RETRY_MS));
	}

	for (;;) {
		(void)zros_sub_wait(&ctx->sub, K_FOREVER);
		if (zros_sub_update(&ctx->sub) != 0) {
			continue;
		}

		result = sample_publish(ctx);
		if (result < 0) {
			ctx->failed++;
			LOG_WRN("CDR publication failed: %d", result);
		} else {
			ctx->published++;
		}
	}
}

static int ros_cdr_start(void)
{
	k_tid_t thread = k_thread_create(&g_ctx.thread, g_stack, K_THREAD_STACK_SIZEOF(g_stack),
					 ros_cdr_run, &g_ctx, NULL, NULL,
					 CONFIG_SPINALI_SYNAPSE_ROS_CDR_THREAD_PRIORITY, 0,
					 K_FOREVER);

	k_thread_name_set(thread, "synapse_ros_cdr");
	k_thread_start(thread);
	return 0;
}

SYS_INIT(ros_cdr_start, APPLICATION, 93);

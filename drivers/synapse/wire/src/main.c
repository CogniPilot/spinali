/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_linkaddr.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/util.h>

#include "ipv6.h"

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <synapse/wire.h>

#include <synapse_topic_list.h>

LOG_MODULE_REGISTER(synapse_wire, CONFIG_SPINALI_SYNAPSE_WIRE_LOG_LEVEL);

#define SYNAPSE_SCHEMA_SET_WIRE_ID UINT64_C(0x232721f0ee5b6c32)
#define SOCKET_RETRY_MS            1000

#if defined(CONFIG_SPINALI_SYNAPSE_WIRE_TOPIC_GNSS_FIX)
typedef synapse_topic_GnssFix_t wire_sample_t;
#define WIRE_TOPIC_ID           UINT16_C(8)
#define WIRE_TOPIC              topic_nav_sat_fix
#define WIRE_TOPIC_NAME         "GnssFix"
#define WIRE_PAYLOAD_SIZE       64U
#define WIRE_CANONICAL_PAD_SIZE 6U
#else
typedef synapse_topic_OpticalFlowVelocityData_t wire_sample_t;
#define WIRE_TOPIC_ID           UINT16_C(10)
#define WIRE_TOPIC              topic_optical_flow_vel
#define WIRE_TOPIC_NAME         "OpticalFlowVelocity"
#define WIRE_PAYLOAD_SIZE       32U
#define WIRE_CANONICAL_PAD_SIZE 0U
#endif

#define DATAGRAM_SIZE (SYNAPSE_WIRE_V1_HEADER_SIZE + WIRE_PAYLOAD_SIZE)

BUILD_ASSERT(sizeof(wire_sample_t) == WIRE_PAYLOAD_SIZE);
BUILD_ASSERT(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);

struct wire_context {
	struct zros_node node;
	struct zros_sub sub;
	wire_sample_t sample;
	struct k_thread thread;
	uint8_t datagram[DATAGRAM_SIZE];
	uint64_t session_id;
	uint32_t sequence;
	uint32_t sent;
	uint32_t failed;
};

static K_THREAD_STACK_DEFINE(g_stack, CONFIG_SPINALI_SYNAPSE_WIRE_THREAD_STACK_SIZE);
static struct wire_context g_ctx;

static uint64_t session_id_create(uint64_t sample_timestamp_ns)
{
	uint8_t device_id[16];
	uint64_t value = sample_timestamp_ns ^ k_cycle_get_64();
	ssize_t length = hwinfo_get_device_id(device_id, sizeof(device_id));

	if (length > 0) {
		for (ssize_t index = 0; index < length; ++index) {
			value ^= (uint64_t)device_id[index]
				 << ((uint32_t)index % sizeof(value)) * 8U;
			value = (value << 7U) | (value >> 57U);
		}
	}

	return value == 0U ? UINT64_C(1) : value;
}

static struct net_if *application_iface_get(void)
{
	struct net_if *vlan = net_eth_get_vlan_iface(NULL, CONFIG_SPINALI_SYNAPSE_WIRE_VLAN_ID);
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

		result = net_eth_vlan_enable(base, CONFIG_SPINALI_SYNAPSE_WIRE_VLAN_ID);
		if (result != 0 && result != -EALREADY) {
			LOG_ERR("VLAN %d enable failed: %d", CONFIG_SPINALI_SYNAPSE_WIRE_VLAN_ID,
				result);
			return NULL;
		}

		vlan = net_eth_get_vlan_iface(base, CONFIG_SPINALI_SYNAPSE_WIRE_VLAN_ID);
		if (vlan == NULL) {
			vlan = net_eth_get_vlan_iface(NULL, CONFIG_SPINALI_SYNAPSE_WIRE_VLAN_ID);
		}
	}
	if (vlan != NULL && !net_if_is_up(vlan) && net_if_up(vlan) != 0) {
		return NULL;
	}
	return vlan;
}

static int destination_neighbor_add(struct net_if *iface,
				    const struct net_in6_addr *destination)
{
	uint8_t address[NET_ETH_ADDR_LEN];
	struct net_linkaddr link_address;

	if (net_bytes_from_str(address, (int)sizeof(address),
			       CONFIG_SPINALI_SYNAPSE_WIRE_DESTINATION_MAC) < 0 ||
	    net_linkaddr_create(&link_address, address, (uint8_t)sizeof(address),
				NET_LINK_ETHERNET) < 0) {
		return -EINVAL;
	}

	return net_ipv6_nbr_add(iface, destination, &link_address, false,
				NET_IPV6_NBR_STATE_STATIC) == NULL
		       ? -ENOMEM
		       : 0;
}

static int socket_open(struct sockaddr_in6 *destination)
{
	struct sockaddr_in6 local = {0};
	struct net_if *iface = application_iface_get();
	int fd;
	int hop_limit = 1;
	int priority = CONFIG_SPINALI_SYNAPSE_WIRE_PCP;

	if (iface == NULL) {
		return -ENETDOWN;
	}

	memset(destination, 0, sizeof(*destination));
	destination->sin6_family = AF_INET6;
	destination->sin6_port = htons(CONFIG_SPINALI_SYNAPSE_WIRE_PORT);
	destination->sin6_scope_id = (uint32_t)net_if_get_by_iface(iface);
	if (net_addr_pton(AF_INET6, CONFIG_SPINALI_SYNAPSE_WIRE_DESTINATION,
			  &destination->sin6_addr) != 0) {
		return -EINVAL;
	}
	if (destination_neighbor_add(iface, &destination->sin6_addr) != 0) {
		LOG_ERR("static neighbor %s at %s setup failed",
			CONFIG_SPINALI_SYNAPSE_WIRE_DESTINATION,
			CONFIG_SPINALI_SYNAPSE_WIRE_DESTINATION_MAC);
		return -EINVAL;
	}

	fd = zsock_socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	if (fd < 0) {
		return -errno;
	}

	if (zsock_setsockopt(fd, IPPROTO_IPV6, IPV6_UNICAST_HOPS, &hop_limit, sizeof(hop_limit)) !=
		    0 ||
	    zsock_setsockopt(fd, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority)) != 0) {
		int error = -errno;
		(void)zsock_close(fd);
		return error;
	}

	local.sin6_family = AF_INET6;
	local.sin6_port = htons(CONFIG_SPINALI_SYNAPSE_WIRE_PORT);
	local.sin6_scope_id = destination->sin6_scope_id;
	if (zsock_bind(fd, (const struct sockaddr *)&local, sizeof(local)) != 0) {
		int error = -errno;
		(void)zsock_close(fd);
		return error;
	}

	LOG_INF("TX VLAN %d iface %u to [%s]:%d PCP %d", CONFIG_SPINALI_SYNAPSE_WIRE_VLAN_ID,
		destination->sin6_scope_id, CONFIG_SPINALI_SYNAPSE_WIRE_DESTINATION,
		CONFIG_SPINALI_SYNAPSE_WIRE_PORT, CONFIG_SPINALI_SYNAPSE_WIRE_PCP);
	return fd;
}

static int sample_send(struct wire_context *ctx, int fd, const struct sockaddr_in6 *destination)
{
	wire_sample_t payload = ctx->sample;
	const bool synchronized = payload.time_status == SYNAPSE_TYPES_TIME_STATUS_GPTP_SYNCED;
	const synapse_wire_header_t header = {
		.magic = SYNAPSE_WIRE_V1_MAGIC,
		.wire_protocol_version = SYNAPSE_WIRE_V1_VERSION,
		.topic_id = WIRE_TOPIC_ID,
		.source_node_id = CONFIG_SPINALI_SYNAPSE_WIRE_SOURCE_NODE_ID,
		.sequence = ctx->sequence,
		.capture_timestamp_ns = synchronized ? payload.timestamp_ns : 0U,
		.schema_set_id = SYNAPSE_SCHEMA_SET_WIRE_ID,
		.payload_length = sizeof(payload),
		.flags = synchronized ? SYNAPSE_WIRE_FLAG_CAPTURE_TIME_GPTP_SYNCED : 0U,
		.source_session_id = ctx->session_id,
	};
	size_t encoded_size = 0U;
	ssize_t sent;

	if (WIRE_CANONICAL_PAD_SIZE > 0U) {
		memset(&((uint8_t *)&payload)[sizeof(payload) - WIRE_CANONICAL_PAD_SIZE], 0,
		       WIRE_CANONICAL_PAD_SIZE);
	}
	if (synapse_wire_encode(ctx->datagram, sizeof(ctx->datagram), &header, &payload,
				sizeof(payload), &encoded_size) != SYNAPSE_WIRE_STATUS_OK) {
		return -EINVAL;
	}

	sent = zsock_sendto(fd, ctx->datagram, encoded_size, 0,
			    (const struct sockaddr *)destination, sizeof(*destination));
	if (sent != (ssize_t)encoded_size) {
		return sent < 0 ? -errno : -EIO;
	}

	ctx->sequence++;
	ctx->sent++;
	return 0;
}

static void wire_run(void *first, void *second, void *third)
{
	struct wire_context *ctx = first;
	struct sockaddr_in6 destination;
	int fd = -1;
	int result;

	ARG_UNUSED(second);
	ARG_UNUSED(third);

	zros_node_init(&ctx->node, "synapse_wire_tx");
	result = zros_sub_init(&ctx->sub, &ctx->node, &WIRE_TOPIC, &ctx->sample,
			       CONFIG_SPINALI_SYNAPSE_WIRE_RATE_HZ);
	if (result < 0) {
		LOG_ERR("%s subscription failed: %d", WIRE_TOPIC_NAME, result);
		return;
	}

	for (;;) {
		if (fd < 0) {
			fd = socket_open(&destination);
			if (fd < 0) {
				k_sleep(K_MSEC(SOCKET_RETRY_MS));
				continue;
			}
		}

		(void)zros_sub_wait(&ctx->sub, K_MSEC(SOCKET_RETRY_MS));
		if (zros_sub_update(&ctx->sub) != 0) {
			continue;
		}

		if (ctx->session_id == 0U) {
			ctx->session_id = session_id_create(ctx->sample.timestamp_ns);
		}
		result = sample_send(ctx, fd, &destination);
		if (result < 0) {
			ctx->failed++;
			LOG_WRN("send failed: %d", result);
			(void)zsock_close(fd);
			fd = -1;
		}
	}
}

static int wire_start(void)
{
	k_tid_t thread = k_thread_create(&g_ctx.thread, g_stack, K_THREAD_STACK_SIZEOF(g_stack),
					 wire_run, &g_ctx, NULL, NULL,
					 CONFIG_SPINALI_SYNAPSE_WIRE_THREAD_PRIORITY, 0, K_FOREVER);

	k_thread_name_set(thread, "synapse_wire_tx");
	k_thread_start(thread);
	return 0;
}

SYS_INIT(wire_start, APPLICATION, 92);

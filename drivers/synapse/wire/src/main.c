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

/*
 * Every stream carries one fixed-layout Synapse sample type on its own UDP
 * port with its own sequence and session, so the receiver validates each
 * independently. The primary stream is the Kconfig choice; the raw
 * OpticalFlowData stream is optional and rides beside the velocity stream,
 * because the vehicle's tightly coupled flow path fuses the integrated flow
 * angles, the delta angle over the same window and the integration time,
 * none of which the 32-byte velocity sample carries.
 */
struct wire_stream {
	const char *name;
	struct zros_topic *topic;
	uint16_t topic_id;
	uint16_t payload_size;
	uint16_t canonical_pad_size;
	uint16_t port;
	size_t time_status_offset;
};

#if defined(CONFIG_SPINALI_SYNAPSE_WIRE_TOPIC_GNSS_FIX)
#define WIRE_PRIMARY_TYPE              synapse_topic_GnssFix_t
#define WIRE_PRIMARY_TOPIC_ID          UINT16_C(8)
#define WIRE_PRIMARY_TOPIC             topic_nav_sat_fix
#define WIRE_PRIMARY_TOPIC_NAME        "GnssFix"
#define WIRE_PRIMARY_PAYLOAD_SIZE      64U
#define WIRE_PRIMARY_CANONICAL_PAD     6U
#else
#define WIRE_PRIMARY_TYPE              synapse_topic_OpticalFlowVelocityData_t
#define WIRE_PRIMARY_TOPIC_ID          UINT16_C(10)
#define WIRE_PRIMARY_TOPIC             topic_optical_flow_vel
#define WIRE_PRIMARY_TOPIC_NAME        "OpticalFlowVelocity"
#define WIRE_PRIMARY_PAYLOAD_SIZE      32U
#define WIRE_PRIMARY_CANONICAL_PAD     0U
#endif

#define WIRE_RAW_TOPIC_ID              UINT16_C(9)
#define WIRE_RAW_PAYLOAD_SIZE          88U

#if defined(CONFIG_SPINALI_SYNAPSE_WIRE_TOPIC_OPTICAL_FLOW_RAW)
#define WIRE_STREAM_COUNT              2U
#define WIRE_MAX_PAYLOAD_SIZE          MAX(WIRE_PRIMARY_PAYLOAD_SIZE, WIRE_RAW_PAYLOAD_SIZE)
#else
#define WIRE_STREAM_COUNT              1U
#define WIRE_MAX_PAYLOAD_SIZE          WIRE_PRIMARY_PAYLOAD_SIZE
#endif

#define DATAGRAM_SIZE (SYNAPSE_WIRE_V1_HEADER_SIZE + WIRE_MAX_PAYLOAD_SIZE)

BUILD_ASSERT(sizeof(WIRE_PRIMARY_TYPE) == WIRE_PRIMARY_PAYLOAD_SIZE);
BUILD_ASSERT(offsetof(WIRE_PRIMARY_TYPE, timestamp_ns) == 0U);
#if defined(CONFIG_SPINALI_SYNAPSE_WIRE_TOPIC_OPTICAL_FLOW_RAW)
BUILD_ASSERT(sizeof(synapse_topic_OpticalFlowData_t) == WIRE_RAW_PAYLOAD_SIZE);
BUILD_ASSERT(offsetof(synapse_topic_OpticalFlowData_t, timestamp_ns) == 0U);
#endif
BUILD_ASSERT(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);

static const struct wire_stream g_streams[WIRE_STREAM_COUNT] = {
	{
		.name = WIRE_PRIMARY_TOPIC_NAME,
		.topic = &WIRE_PRIMARY_TOPIC,
		.topic_id = WIRE_PRIMARY_TOPIC_ID,
		.payload_size = WIRE_PRIMARY_PAYLOAD_SIZE,
		.canonical_pad_size = WIRE_PRIMARY_CANONICAL_PAD,
		.port = CONFIG_SPINALI_SYNAPSE_WIRE_PORT,
		.time_status_offset = offsetof(WIRE_PRIMARY_TYPE, time_status),
	},
#if defined(CONFIG_SPINALI_SYNAPSE_WIRE_TOPIC_OPTICAL_FLOW_RAW)
	{
		.name = "OpticalFlow",
		.topic = &topic_optical_flow,
		.topic_id = WIRE_RAW_TOPIC_ID,
		.payload_size = WIRE_RAW_PAYLOAD_SIZE,
		.canonical_pad_size = 0U,
		.port = CONFIG_SPINALI_SYNAPSE_WIRE_OPTICAL_FLOW_RAW_PORT,
		.time_status_offset = offsetof(synapse_topic_OpticalFlowData_t, time_status),
	},
#endif
};

struct wire_context {
	const struct wire_stream *stream;
	struct zros_node node;
	struct zros_sub sub;
	uint8_t sample[WIRE_MAX_PAYLOAD_SIZE] __aligned(8);
	struct k_thread thread;
	uint8_t datagram[DATAGRAM_SIZE];
	uint64_t session_id;
	uint32_t sequence;
	uint32_t sent;
	uint32_t failed;
};

static K_THREAD_STACK_ARRAY_DEFINE(g_stacks, WIRE_STREAM_COUNT,
				   CONFIG_SPINALI_SYNAPSE_WIRE_THREAD_STACK_SIZE);
static struct wire_context g_contexts[WIRE_STREAM_COUNT];

static uint64_t sample_timestamp_ns(const struct wire_context *ctx)
{
	uint64_t timestamp_ns;

	memcpy(&timestamp_ns, ctx->sample, sizeof(timestamp_ns));
	return timestamp_ns;
}

static uint8_t sample_time_status(const struct wire_context *ctx)
{
	return ctx->sample[ctx->stream->time_status_offset];
}

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

static int socket_open(const struct wire_stream *stream, struct sockaddr_in6 *destination)
{
	struct sockaddr_in6 local = {0};
	struct net_if *iface = application_iface_get();
	int fd;
	int hop_limit = 1;
	uint8_t priority = CONFIG_SPINALI_SYNAPSE_WIRE_PCP;

	if (iface == NULL) {
		return -ENETDOWN;
	}

	memset(destination, 0, sizeof(*destination));
	destination->sin6_family = AF_INET6;
	destination->sin6_port = htons(stream->port);
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
	local.sin6_port = htons(stream->port);
	local.sin6_scope_id = destination->sin6_scope_id;
	if (zsock_bind(fd, (const struct sockaddr *)&local, sizeof(local)) != 0) {
		int error = -errno;

		(void)zsock_close(fd);
		return error;
	}

	LOG_INF("%s TX VLAN %d iface %u to [%s]:%d PCP %d", stream->name,
		CONFIG_SPINALI_SYNAPSE_WIRE_VLAN_ID, destination->sin6_scope_id,
		CONFIG_SPINALI_SYNAPSE_WIRE_DESTINATION, stream->port,
		CONFIG_SPINALI_SYNAPSE_WIRE_PCP);

	return fd;
}

static int sample_send(struct wire_context *ctx, int fd, const struct sockaddr_in6 *destination)
{
	const struct wire_stream *stream = ctx->stream;
	uint8_t payload[WIRE_MAX_PAYLOAD_SIZE] __aligned(8);
	const bool synchronized =
		sample_time_status(ctx) == SYNAPSE_TYPES_TIME_STATUS_GPTP_SYNCED;
	const synapse_wire_header_t header = {
		.magic = SYNAPSE_WIRE_V1_MAGIC,
		.wire_protocol_version = SYNAPSE_WIRE_V1_VERSION,
		.topic_id = stream->topic_id,
		.source_node_id = CONFIG_SPINALI_SYNAPSE_WIRE_SOURCE_NODE_ID,
		.sequence = ctx->sequence,
		.capture_timestamp_ns = synchronized ? sample_timestamp_ns(ctx) : 0U,
		.schema_set_id = SYNAPSE_SCHEMA_SET_WIRE_ID,
		.payload_length = stream->payload_size,
		.flags = synchronized ? SYNAPSE_WIRE_FLAG_CAPTURE_TIME_GPTP_SYNCED : 0U,
		.source_session_id = ctx->session_id,
	};
	size_t encoded_size = 0U;
	ssize_t sent;

	memcpy(payload, ctx->sample, stream->payload_size);
	if (stream->canonical_pad_size > 0U) {
		memset(&payload[stream->payload_size - stream->canonical_pad_size], 0,
		       stream->canonical_pad_size);
	}

	if (synapse_wire_encode(ctx->datagram, sizeof(ctx->datagram), &header, payload,
				stream->payload_size, &encoded_size) != SYNAPSE_WIRE_STATUS_OK) {
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
	const struct wire_stream *stream = ctx->stream;
	struct sockaddr_in6 destination;
	int fd = -1;
	int result;

	ARG_UNUSED(second);
	ARG_UNUSED(third);

	zros_node_init(&ctx->node, "synapse_wire_tx");
	result = zros_sub_init(&ctx->sub, &ctx->node, stream->topic, ctx->sample,
			       CONFIG_SPINALI_SYNAPSE_WIRE_RATE_HZ);
	if (result < 0) {
		LOG_ERR("%s subscription failed: %d", stream->name, result);
		return;
	}

	for (;;) {
		if (fd < 0) {
			fd = socket_open(stream, &destination);
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
			ctx->session_id = session_id_create(sample_timestamp_ns(ctx));
		}

		result = sample_send(ctx, fd, &destination);
		if (result < 0) {
			ctx->failed++;
			LOG_WRN("%s send failed: %d", stream->name, result);
			(void)zsock_close(fd);
			fd = -1;
		}
	}
}

static int wire_start(void)
{
	for (size_t index = 0U; index < WIRE_STREAM_COUNT; ++index) {
		struct wire_context *ctx = &g_contexts[index];
		k_tid_t thread;

		ctx->stream = &g_streams[index];
		thread = k_thread_create(&ctx->thread, g_stacks[index],
					 K_THREAD_STACK_SIZEOF(g_stacks[index]), wire_run, ctx,
					 NULL, NULL, CONFIG_SPINALI_SYNAPSE_WIRE_THREAD_PRIORITY,
					 0, K_FOREVER);
		k_thread_name_set(thread, index == 0U ? "synapse_wire_tx" : "synapse_wire_tx_raw");
		k_thread_start(thread);
	}

	return 0;
}

SYS_INIT(wire_start, APPLICATION, 92);

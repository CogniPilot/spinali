/*
 * Copyright (c) 2026 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 *
 * CAN over Ethernet (COE): bridges each CAN bus to a UDP socket using the
 * cannelloni data frame format (version 2, op DATA, big-endian), so the
 * Linux side can use cannelloni + vcan for native SocketCAN integration.
 *
 * Bus N listens on UDP port SPINALI_COE_PORT_BASE + N and forwards CAN
 * frames to the configured peer (or the last peer that sent a datagram).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(coe, CONFIG_SPINALI_COE_LOG_LEVEL);

#define COE_VERSION 2U
#define COE_OP_DATA 0U
#define COE_BUS_COUNT 2U
#define COE_MTU 1400U

#define CAN_EFF_FLAG 0x80000000UL
#define CAN_RTR_FLAG 0x40000000UL

struct coe_msg {
	uint8_t bus;
	struct can_frame frame;
};

struct coe_bus {
	const struct device *dev;
	int sock;
	struct sockaddr_in peer;
	bool peer_locked;
	uint8_t seq;
	uint32_t rx_can;
	uint32_t tx_can;
	uint32_t rx_udp;
	uint32_t tx_udp;
};

static struct coe_bus g_bus[COE_BUS_COUNT] = {
	{.dev = DEVICE_DT_GET(DT_ALIAS(coe_can0))},
	{.dev = DEVICE_DT_GET(DT_ALIAS(coe_can1))},
};

K_MSGQ_DEFINE(g_canq, sizeof(struct coe_msg), 32, 4);

static void coe_rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	struct coe_msg msg = {.bus = (uint8_t)(uintptr_t)user_data, .frame = *frame};

	ARG_UNUSED(dev);
	g_bus[msg.bus].rx_can++;
	if (k_msgq_put(&g_canq, &msg, K_NO_WAIT) != 0) {
		LOG_WRN("can%u: udp-bound queue full, frame dropped", msg.bus);
	}
}

static size_t coe_encode(const struct can_frame *frame, uint8_t *out)
{
	uint32_t id = frame->id;
	size_t n = 0;
	bool fd = (frame->flags & CAN_FRAME_FDF) != 0U;
	uint8_t len = fd ? can_dlc_to_bytes(frame->dlc) : frame->dlc;

	if (frame->flags & CAN_FRAME_IDE) {
		id |= CAN_EFF_FLAG;
	}
	if (frame->flags & CAN_FRAME_RTR) {
		id |= CAN_RTR_FLAG;
	}
	sys_put_be32(id, &out[n]);
	n += 4;
	out[n++] = fd ? (len | 0x80U) : len;
	if (fd) {
		out[n++] = (frame->flags & CAN_FRAME_BRS) ? 0x01U : 0x00U;
	}
	if (!(frame->flags & CAN_FRAME_RTR)) {
		memcpy(&out[n], frame->data, fd ? len : MIN(len, 8U));
		n += fd ? len : MIN(len, 8U);
	}
	return n;
}

static int coe_decode(const uint8_t *in, size_t avail, struct can_frame *frame)
{
	size_t n = 0;
	uint32_t id;
	uint8_t len;
	bool fd;

	if (avail < 5U) {
		return -EINVAL;
	}
	id = sys_get_be32(&in[n]);
	n += 4;
	len = in[n++];
	fd = (len & 0x80U) != 0U;
	len &= 0x7FU;

	memset(frame, 0, sizeof(*frame));
	frame->id = id & 0x1FFFFFFFUL;
	if (id & CAN_EFF_FLAG) {
		frame->flags |= CAN_FRAME_IDE;
	}
	if (fd) {
		if (n >= avail) {
			return -EINVAL;
		}
		frame->flags |= CAN_FRAME_FDF;
		if (in[n++] & 0x01U) {
			frame->flags |= CAN_FRAME_BRS;
		}
		if (len > CAN_MAX_DLEN || n + len > avail) {
			return -EINVAL;
		}
		frame->dlc = can_bytes_to_dlc(len);
		memcpy(frame->data, &in[n], len);
	} else {
		if (id & CAN_RTR_FLAG) {
			frame->flags |= CAN_FRAME_RTR;
			frame->dlc = MIN(len, 8U);
			return (int)n;
		}
		if (len > 8U || n + len > avail) {
			return -EINVAL;
		}
		frame->dlc = len;
		memcpy(frame->data, &in[n], len);
	}
	return (int)(n + ((frame->flags & CAN_FRAME_RTR) ? 0U : len));
}

static void coe_udp_tx_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	uint8_t pkt[COE_MTU];
	struct coe_msg msg;

	while (true) {
		k_msgq_get(&g_canq, &msg, K_FOREVER);
		struct coe_bus *bus = &g_bus[msg.bus];
		size_t n = 5;

		pkt[0] = COE_VERSION;
		pkt[1] = COE_OP_DATA;
		pkt[2] = bus->seq++;
		n += coe_encode(&msg.frame, &pkt[n]);
		/* opportunistically batch whatever else is queued for this bus */
		uint16_t count = 1;
		struct coe_msg more;

		while (n + 72U < sizeof(pkt) && k_msgq_peek(&g_canq, &more) == 0 &&
		       more.bus == msg.bus) {
			k_msgq_get(&g_canq, &more, K_NO_WAIT);
			n += coe_encode(&more.frame, &pkt[n]);
			count++;
		}
		sys_put_be16(count, &pkt[3]);
		if (zsock_sendto(bus->sock, pkt, n, 0, (struct sockaddr *)&bus->peer,
				 sizeof(bus->peer)) >= 0) {
			bus->tx_udp++;
		}
	}
}

static void coe_udp_rx_thread(void *p1, void *b, void *c)
{
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	struct coe_bus *bus = &g_bus[(uintptr_t)p1];
	uint8_t pkt[COE_MTU];
	struct sockaddr_in from;
	socklen_t fromlen;

	while (true) {
		fromlen = sizeof(from);
		ssize_t r = zsock_recvfrom(bus->sock, pkt, sizeof(pkt), 0,
					   (struct sockaddr *)&from, &fromlen);
		if (r < 5 || pkt[0] != COE_VERSION || pkt[1] != COE_OP_DATA) {
			continue;
		}
		bus->rx_udp++;
		/* learn the peer from inbound traffic */
		bus->peer.sin_addr = from.sin_addr;
		bus->peer_locked = true;

		uint16_t count = sys_get_be16(&pkt[3]);
		size_t off = 5;

		for (uint16_t i = 0; i < count && off < (size_t)r; i++) {
			struct can_frame frame;
			int used = coe_decode(&pkt[off], (size_t)r - off, &frame);

			if (used < 0) {
				LOG_WRN("bus%u: bad frame in datagram", (unsigned)(uintptr_t)p1);
				break;
			}
			off += (size_t)used;
			if (can_send(bus->dev, &frame, K_MSEC(100), NULL, NULL) == 0) {
				bus->tx_can++;
			} else {
				LOG_WRN("bus%u: can_send failed", (unsigned)(uintptr_t)p1);
			}
		}
	}
}

K_THREAD_DEFINE(coe_tx, 4096, coe_udp_tx_thread, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(coe_rx0, 4096, coe_udp_rx_thread, (void *)0, NULL, NULL, 6, 0, 500);
K_THREAD_DEFINE(coe_rx1, 4096, coe_udp_rx_thread, (void *)1, NULL, NULL, 6, 0, 500);

int main(void)
{
	printk("CogniPilot Spinali: CAN over Ethernet\n");

	for (uint8_t i = 0; i < COE_BUS_COUNT; i++) {
		struct coe_bus *bus = &g_bus[i];

		if (!device_is_ready(bus->dev)) {
			LOG_ERR("can%u not ready", i);
			continue;
		}
		if (can_set_mode(bus->dev, CAN_MODE_FD) != 0) {
			LOG_WRN("can%u: FD mode unavailable, classic only", i);
			(void)can_set_mode(bus->dev, CAN_MODE_NORMAL);
		}
		if (can_start(bus->dev) != 0) {
			LOG_ERR("can%u: start failed", i);
			continue;
		}

		const struct can_filter fstd = {.id = 0, .mask = 0, .flags = 0};
		const struct can_filter fext = {.id = 0, .mask = 0, .flags = CAN_FILTER_IDE};

		(void)can_add_rx_filter(bus->dev, coe_rx_cb, (void *)(uintptr_t)i, &fstd);
		(void)can_add_rx_filter(bus->dev, coe_rx_cb, (void *)(uintptr_t)i, &fext);

		bus->sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		struct sockaddr_in local = {.sin_family = AF_INET,
					    .sin_port = htons(CONFIG_SPINALI_COE_PORT_BASE + i),
					    .sin_addr = {.s_addr = INADDR_ANY}};
		(void)zsock_bind(bus->sock, (struct sockaddr *)&local, sizeof(local));
		bus->peer.sin_family = AF_INET;
		bus->peer.sin_port = htons(CONFIG_SPINALI_COE_PORT_BASE + i);
		zsock_inet_pton(AF_INET, CONFIG_SPINALI_COE_PEER_ADDR, &bus->peer.sin_addr);
		LOG_INF("can%u: bridged on udp/%d", i, CONFIG_SPINALI_COE_PORT_BASE + i);
	}
	return 0;
}

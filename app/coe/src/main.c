/*
 * Copyright (c) 2026 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 *
 * CAN over Ethernet (COE): bridges each CAN bus onto raw Ethernet as IEEE
 * 1722 NTSCF AVTPDUs carrying ACF-CAN messages (ethertype 0x22F0), so the
 * Linux side can use an IEEE 1722 ACF-CAN bridge for native SocketCAN
 * integration.
 *
 * Each bus owns a 64-bit IEEE 1722 stream ID: the interface MAC in the upper
 * 48 bits and the bus stream index (SPINALI_COE_STREAM_UID_BASE + bus) in the
 * lower 16, so a node's streams are unique on the network without
 * coordination. It also owns its own NTSCF sequence number. Frames queued by
 * the CAN receive callbacks are batched into a single AVTPDU per transmission
 * and sent to SPINALI_COE_DST_MAC, which is replaced by the source MAC of the
 * first valid inbound AVTPDU. Inbound AVTPDUs are demultiplexed by the low 16
 * bit stream index and their ACF-CAN messages are queued to the matching bus,
 * which owns a writer
 * thread of its own so that a bus without a peer to acknowledge its frames
 * cannot hold up the other bus or the packet socket.
 *
 * Every frame bridged toward Ethernet carries the time of its arrival, taken
 * from the PTP hardware clock of the Ethernet MAC (PHC) and sent as the
 * ACF-CAN message timestamp with MTV set. The same PHC is disciplined to GNSS
 * time by the timing library and served to the network by gPTP, so the
 * timestamps are on the PTP timescale and traceable to GPS network wide. Until
 * that discipline has locked once, the PHC holds whatever the boot seed left
 * it at, and frames go out with MTV clear rather than claiming a timescale the
 * clock is not on.
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_SHELL)
#include <zephyr/shell/shell.h>
#endif
#if defined(CONFIG_SPINALI_TIMING)
#include "pps_servo.h"
#endif

LOG_MODULE_REGISTER(coe, CONFIG_SPINALI_COE_LOG_LEVEL);

#define COE_BUS_COUNT 2U

/* IEEE 1722 NTSCF control format header: three quadlets. */
#define COE_AVTP_SUBTYPE_NTSCF 0x82U
#define COE_AVTP_VERSION 0U
#define COE_NTSCF_HDR_LEN 12U
#define COE_NTSCF_SV BIT(7)
#define COE_NTSCF_DATA_LEN_MAX 0x7FFU

/* IEEE 1722 ACF-CAN message: four quadlets of header ahead of the payload. */
#define COE_ACF_TYPE_CAN 0x01U
#define COE_ACF_HDR_LEN 4U
#define COE_ACF_CAN_HDR_LEN 16U
#define COE_ACF_CAN_MSG_MAX (COE_ACF_CAN_HDR_LEN + 64U)

/* Flag bits in the third octet of the ACF-CAN header. */
#define COE_ACF_CAN_MTV BIT(5)
#define COE_ACF_CAN_RTR BIT(4)
#define COE_ACF_CAN_EFF BIT(3)
#define COE_ACF_CAN_BRS BIT(2)
#define COE_ACF_CAN_FDF BIT(1)
#define COE_ACF_CAN_ESI BIT(0)

#define COE_PDU_MAX 1450U
#define COE_QUADLET 4U
#define COE_CAN_ID_MASK 0x1FFFFFFFUL
#define COE_CAN_STD_ID_MAX 0x7FFUL

/*
 * Interoperability bound rather than a format limit: widely deployed ACF-CAN
 * listeners decode an AVTPDU into a fixed per-PDU array of 15 CAN frames, so
 * batching more than that into one AVTPDU overruns them. The byte bound on the
 * PDU still applies on top of this count.
 */
#define COE_ACF_CAN_MSG_PER_PDU 15U

/* Depth of the per-bus queue between the AVTPDU receiver and its bus writer. */
#define COE_BUS_TXQ_DEPTH 32U

/* Pause after a packet socket receive error, so a persistent fault cannot spin. */
#define COE_RX_ERR_BACKOFF_MS 10U

/*
 * IEEE 1722 stream IDs carry the talker's 48 bit MAC in the upper bits and a
 * 16 bit stream index in the lower bits, so every talker's streams are unique
 * on the network without coordination. Bus N uses index UID_BASE + N.
 */
#define COE_STREAM_UID_BASE ((uint16_t)CONFIG_SPINALI_COE_STREAM_UID_BASE)
#define COE_STREAM_ID(mac48, uid) (((uint64_t)(mac48) << 16) | (uint64_t)(uint16_t)(uid))

BUILD_ASSERT(COE_PDU_MAX - COE_NTSCF_HDR_LEN <= COE_NTSCF_DATA_LEN_MAX,
	     "AVTPDU payload must fit the 11 bit ntscf_data_length field");
BUILD_ASSERT(COE_BUS_COUNT <= 32U, "can_bus_id is a five bit field");

struct coe_msg {
	/* PHC time of frame arrival, in nanoseconds on the PTP timescale.
	 * Meaningful only while ts_valid is set.
	 */
	uint64_t ts_ns;
	struct can_frame frame;
	uint8_t bus;
	bool ts_valid;
};

struct coe_bus {
	const struct device *dev;
	struct k_msgq *txq;
	uint64_t stream_id;
	uint8_t seq;
	bool txq_up;
	uint32_t rx_can;
	uint32_t tx_can;
	uint32_t tx_drop;
	uint32_t rx_pdu;
	uint32_t tx_pdu;
	uint32_t tx_err;
	int tx_errno_last;
};

/* One queue per bus, so a bus that cannot transmit only backs up its own. */
K_MSGQ_DEFINE(g_txq0, sizeof(struct can_frame), COE_BUS_TXQ_DEPTH, 4);
K_MSGQ_DEFINE(g_txq1, sizeof(struct can_frame), COE_BUS_TXQ_DEPTH, 4);

static struct coe_bus g_bus[COE_BUS_COUNT] = {
	{.dev = DEVICE_DT_GET(DT_ALIAS(coe_can0)), .txq = &g_txq0, .txq_up = true},
	{.dev = DEVICE_DT_GET(DT_ALIAS(coe_can1)), .txq = &g_txq1, .txq_up = true},
};

/* Multicast destination inside the MAAP dynamic pool. */
static const uint8_t g_dst_mac_fallback[NET_ETH_ADDR_LEN] = {0x91, 0xE0, 0xF0, 0x00, 0x0C, 0x0E};

static uint8_t g_dst_mac[NET_ETH_ADDR_LEN];
static bool g_peer_known;
static struct k_spinlock g_dst_lock;

static int g_sock_rx = -1;
static int g_sock_tx = -1;
static int g_ifindex;

/* PTP hardware clock of the Ethernet MAC, resolved once at start up. NULL
 * when the interface exposes none, which sends every frame with MTV clear.
 */
static const struct device *g_phc;

/* Aligned to the message type: an entry carries a 64 bit timestamp. */
K_MSGQ_DEFINE(g_canq, sizeof(struct coe_msg), 32, __alignof__(struct coe_msg));
static K_SEM_DEFINE(g_ready, 0, 1);

/* Gate that holds the transport threads until the sockets and the buses are up. */
static void coe_wait_ready(void)
{
	k_sem_take(&g_ready, K_FOREVER);
	k_sem_give(&g_ready);
}

static bool coe_parse_mac(const char *str, uint8_t *mac)
{
	for (uint8_t i = 0; i < NET_ETH_ADDR_LEN; i++) {
		uint8_t octet = 0;

		for (uint8_t digit = 0; digit < 2U; digit++) {
			char c = *str++;

			if (c >= '0' && c <= '9') {
				octet = (uint8_t)((octet << 4) | (uint8_t)(c - '0'));
			} else if (c >= 'a' && c <= 'f') {
				octet = (uint8_t)((octet << 4) | (uint8_t)(c - 'a' + 10));
			} else if (c >= 'A' && c <= 'F') {
				octet = (uint8_t)((octet << 4) | (uint8_t)(c - 'A' + 10));
			} else {
				return false;
			}
		}
		mac[i] = octet;
		if (i < (NET_ETH_ADDR_LEN - 1U) && *str++ != ':') {
			return false;
		}
	}
	return *str == '\0';
}

static void coe_dst_get(uint8_t *mac)
{
	k_spinlock_key_t key = k_spin_lock(&g_dst_lock);

	memcpy(mac, g_dst_mac, NET_ETH_ADDR_LEN);
	k_spin_unlock(&g_dst_lock, key);
}

/* The first peer seen on a known stream replaces the configured destination. */
static void coe_learn_peer(const struct sockaddr_ll *from, socklen_t fromlen)
{
	uint8_t mac[NET_ETH_ADDR_LEN];
	k_spinlock_key_t key;
	bool learned = false;

	if (fromlen < sizeof(*from) || from->sll_halen != NET_ETH_ADDR_LEN) {
		return;
	}

	key = k_spin_lock(&g_dst_lock);
	if (!g_peer_known) {
		memcpy(g_dst_mac, from->sll_addr, NET_ETH_ADDR_LEN);
		memcpy(mac, g_dst_mac, NET_ETH_ADDR_LEN);
		g_peer_known = true;
		learned = true;
	}
	k_spin_unlock(&g_dst_lock, key);

	if (learned) {
		LOG_INF("peer %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3],
			mac[4], mac[5]);
	}
}

/*
 * Whether the PHC has been placed on the GNSS timescale. Without the timing
 * library nothing disciplines it, so its readings carry no traceable meaning
 * and no frame may claim one.
 */
static inline bool coe_disciplined(void)
{
#if defined(CONFIG_SPINALI_TIMING)
	return pps_servo_disciplined();
#else
	return false;
#endif
}

/*
 * Runs in the CAN controller interrupt, which is where the arrival time is
 * worth taking: the PHC is read before the frame is queued, so the timestamp
 * carries no scheduling delay. Reading it here is safe because the get path of
 * the ENET QoS PTP clock only reads the system-time registers, re-reading on a
 * seconds roll-over, and takes no lock; the mutex that driver holds is confined
 * to the set, adjust and rate-adjust paths, none of which run here.
 *
 * The timestamp is only asserted once the discipline servo has locked at least
 * once. Before that the PHC runs from the boot seed, which is the RTC at best
 * and the epoch at worst, so a timestamp taken from it would be a claim of PTP
 * traceability the clock cannot back. Holdover keeps the claim standing: the
 * clock coasts on the learned rate, which is what the announced clockClass 7
 * already tells the network.
 *
 * That read is consistent across a nanosecond roll-over, which is what the
 * driver re-reads for, but it is not serialized against a phase step: seconds
 * and nanoseconds are read in sequence, so a step landing between the two
 * reads can in principle mix the two sides of it. Only the servo steps the
 * clock, and once it has locked it steps solely on an error above 100 ms, so
 * the exposure is bounded to an event that disciplined operation has not
 * produced in practice.
 *
 * Frames are stamped as the interrupt reaches them rather than as they landed
 * on the wire. One interrupt drains every mailbox left pending, so frames that
 * arrive close together are read out serially, in mailbox order, and their
 * timestamps are spread by the microseconds the handler spends between them.
 */
static void coe_rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	struct coe_msg msg = {.bus = (uint8_t)(uintptr_t)user_data, .frame = *frame};
	struct net_ptp_time now;

	ARG_UNUSED(dev);

	if ((g_phc != NULL) && coe_disciplined() && (ptp_clock_get(g_phc, &now) == 0)) {
		msg.ts_ns = ((uint64_t)now.second * NSEC_PER_SEC) + now.nanosecond;
		msg.ts_valid = true;
	}

	g_bus[msg.bus].rx_can++;
	if (k_msgq_put(&g_canq, &msg, K_NO_WAIT) != 0) {
		LOG_WRN("can%u: transport queue full, frame dropped", msg.bus);
	}
}

static void coe_ntscf_write(uint8_t *out, uint64_t stream_id, uint8_t seq, uint16_t data_len)
{
	out[0] = COE_AVTP_SUBTYPE_NTSCF;
	out[1] = (uint8_t)(COE_NTSCF_SV | (COE_AVTP_VERSION << 4) |
			   ((data_len >> 8) & 0x07U));
	out[2] = (uint8_t)(data_len & 0xFFU);
	out[3] = seq;
	sys_put_be64(stream_id, &out[4]);
}

/* Encodes one CAN frame as an ACF-CAN message and returns its octet count. */
static size_t coe_acf_can_encode(const struct coe_msg *msg, uint8_t *out)
{
	const struct can_frame *frame = &msg->frame;
	bool fd = (frame->flags & CAN_FRAME_FDF) != 0U;
	bool rtr = (frame->flags & CAN_FRAME_RTR) != 0U;
	uint8_t len = fd ? can_dlc_to_bytes(frame->dlc) : (uint8_t)MIN(frame->dlc, CAN_MAX_DLC);
	uint8_t pad = (uint8_t)((COE_QUADLET - (len % COE_QUADLET)) % COE_QUADLET);
	uint16_t quadlets = (uint16_t)((COE_ACF_CAN_HDR_LEN + len + pad) / COE_QUADLET);
	uint8_t flags = (uint8_t)(pad << 6);

	if (msg->ts_valid) {
		flags |= COE_ACF_CAN_MTV;
	}
	if (rtr) {
		flags |= COE_ACF_CAN_RTR;
	}
	if ((frame->flags & CAN_FRAME_IDE) != 0U) {
		flags |= COE_ACF_CAN_EFF;
	}
	if (fd) {
		flags |= COE_ACF_CAN_FDF;
		if ((frame->flags & CAN_FRAME_BRS) != 0U) {
			flags |= COE_ACF_CAN_BRS;
		}
		if ((frame->flags & CAN_FRAME_ESI) != 0U) {
			flags |= COE_ACF_CAN_ESI;
		}
	}

	out[0] = (uint8_t)((COE_ACF_TYPE_CAN << 1) | ((quadlets >> 8) & 0x01U));
	out[1] = (uint8_t)(quadlets & 0xFFU);
	out[2] = flags;
	out[3] = (uint8_t)(msg->bus & 0x1FU);
	/* Quadlets 1 and 2 hold the 64 bit message timestamp. A frame whose
	 * arrival time could not be read, or was read before the clock became
	 * traceable, is sent with MTV clear and the field zeroed, which a
	 * listener must read as "no timestamp".
	 */
	sys_put_be64(msg->ts_valid ? msg->ts_ns : 0U, &out[4]);
	sys_put_be32(frame->id & COE_CAN_ID_MASK, &out[12]);
	if (rtr) {
		/* A remote request carries no data, only the requested length. */
		memset(&out[COE_ACF_CAN_HDR_LEN], 0, len);
	} else {
		memcpy(&out[COE_ACF_CAN_HDR_LEN], frame->data, len);
	}
	memset(&out[COE_ACF_CAN_HDR_LEN + len], 0, pad);

	return (size_t)COE_ACF_CAN_HDR_LEN + len + pad;
}

/* Decodes one bounds-checked ACF-CAN message of msg_len octets. */
static int coe_acf_can_decode(const uint8_t *msg, size_t msg_len, struct can_frame *frame)
{
	uint8_t flags = msg[2];
	uint8_t pad = (uint8_t)(flags >> 6);
	bool fd = (flags & COE_ACF_CAN_FDF) != 0U;
	uint32_t id = (uint32_t)(sys_get_be32(&msg[12]) & COE_CAN_ID_MASK);
	size_t payload;

	if (msg_len < ((size_t)COE_ACF_CAN_HDR_LEN + pad)) {
		return -EINVAL;
	}
	payload = msg_len - COE_ACF_CAN_HDR_LEN - pad;
	if (payload > (fd ? 64U : 8U)) {
		return -EINVAL;
	}

	memset(frame, 0, sizeof(*frame));
	frame->id = id;
	if ((flags & COE_ACF_CAN_EFF) != 0U) {
		frame->flags |= CAN_FRAME_IDE;
	} else if (id > COE_CAN_STD_ID_MAX) {
		return -EINVAL;
	}

	if (fd) {
		if ((flags & COE_ACF_CAN_RTR) != 0U) {
			/* CAN FD has no remote frames. */
			return -EINVAL;
		}
		if (can_dlc_to_bytes(can_bytes_to_dlc((uint8_t)payload)) != payload) {
			/* Not a length that CAN FD can carry. */
			return -EINVAL;
		}
		frame->flags |= CAN_FRAME_FDF;
		if ((flags & COE_ACF_CAN_BRS) != 0U) {
			frame->flags |= CAN_FRAME_BRS;
		}
		/* ESI reports the sender error state and is not replayed on the
		 * local bus: the controller drives it from its own state and
		 * rejects frames that request it.
		 */
		frame->dlc = can_bytes_to_dlc((uint8_t)payload);
		memcpy(frame->data, &msg[COE_ACF_CAN_HDR_LEN], payload);
	} else if ((flags & COE_ACF_CAN_RTR) != 0U) {
		frame->flags |= CAN_FRAME_RTR;
		frame->dlc = (uint8_t)payload;
	} else {
		frame->dlc = (uint8_t)payload;
		memcpy(frame->data, &msg[COE_ACF_CAN_HDR_LEN], payload);
	}

	return 0;
}

static void coe_tx_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	static uint8_t pdu[COE_PDU_MAX];
	struct sockaddr_ll dst = {
		.sll_family = AF_PACKET,
		.sll_protocol = htons(ETH_P_TSN),
		.sll_halen = NET_ETH_ADDR_LEN,
	};
	struct coe_msg msg;
	bool tx_up = true;

	coe_wait_ready();
	dst.sll_ifindex = g_ifindex;

	while (true) {
		k_msgq_get(&g_canq, &msg, K_FOREVER);
		struct coe_bus *bus = &g_bus[msg.bus];
		size_t n = COE_NTSCF_HDR_LEN;
		unsigned int count = 1U;
		struct coe_msg more;

		n += coe_acf_can_encode(&msg, &pdu[n]);
		/* opportunistically batch whatever else is queued for this bus */
		while (count < COE_ACF_CAN_MSG_PER_PDU &&
		       n + COE_ACF_CAN_MSG_MAX <= COE_PDU_MAX &&
		       k_msgq_peek(&g_canq, &more) == 0 && more.bus == msg.bus) {
			k_msgq_get(&g_canq, &more, K_NO_WAIT);
			n += coe_acf_can_encode(&more, &pdu[n]);
			count++;
		}
		coe_ntscf_write(pdu, bus->stream_id, bus->seq, (uint16_t)(n - COE_NTSCF_HDR_LEN));

		coe_dst_get(dst.sll_addr);
		if (zsock_sendto(g_sock_tx, pdu, n, 0, (struct sockaddr *)&dst, sizeof(dst)) >= 0) {
			/* Receivers account for loss by sequence continuity, so a
			 * number is consumed only by a PDU that reached the wire.
			 */
			bus->seq++;
			bus->tx_pdu++;
			if (!tx_up) {
				LOG_INF("transport up");
				tx_up = true;
			}
		} else {
			bus->tx_err++;
			bus->tx_errno_last = errno;
			if (tx_up) {
				LOG_WRN("send failed: %d", errno);
				tx_up = false;
			}
		}
	}
}

static void coe_rx_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	static uint8_t pdu[COE_PDU_MAX];
	struct sockaddr_ll from;
	socklen_t fromlen;
	bool rx_up = true;

	coe_wait_ready();

	while (true) {
		memset(&from, 0, sizeof(from));
		fromlen = sizeof(from);
		ssize_t r = zsock_recvfrom(g_sock_rx, pdu, sizeof(pdu), 0, (struct sockaddr *)&from,
					   &fromlen);
		if (r < 0) {
			if (rx_up) {
				LOG_WRN("receive failed: %d", errno);
				rx_up = false;
			}
			k_sleep(K_MSEC(COE_RX_ERR_BACKOFF_MS));
			continue;
		}
		if (!rx_up) {
			LOG_INF("receive up");
			rx_up = true;
		}
		if (r < (ssize_t)COE_NTSCF_HDR_LEN) {
			continue;
		}
		if (pdu[0] != COE_AVTP_SUBTYPE_NTSCF || (pdu[1] & COE_NTSCF_SV) == 0U ||
		    ((pdu[1] >> 4) & 0x07U) != COE_AVTP_VERSION) {
			continue;
		}

		size_t data_len = ((size_t)(pdu[1] & 0x07U) << 8) | pdu[2];
		size_t end = COE_NTSCF_HDR_LEN + data_len;

		if (end > (size_t)r) {
			LOG_WRN("ntscf_data_length %u exceeds %d received octets",
				(unsigned int)data_len, (int)r);
			continue;
		}

		uint64_t stream_id = sys_get_be64(&pdu[4]);
		uint16_t uid = (uint16_t)(stream_id & 0xFFFFU);

		/*
		 * Demultiplex on the low 16 bit stream index only: the upper 48
		 * bits are the talker's MAC, which differs from ours, so bus N at
		 * the far end maps to bus N here. A uid below the base underflows
		 * to a large value and is rejected by the bounds check.
		 */
		uint32_t index = (uint32_t)((int)uid - (int)COE_STREAM_UID_BASE);

		if (index >= COE_BUS_COUNT) {
			continue;
		}

		struct coe_bus *bus = &g_bus[index];

		bus->rx_pdu++;
		coe_learn_peer(&from, fromlen);

		size_t off = COE_NTSCF_HDR_LEN;

		while (off + COE_ACF_HDR_LEN <= end) {
			uint8_t type = (uint8_t)(pdu[off] >> 1);
			size_t msg_len = ((((size_t)pdu[off] & 0x01U) << 8) | pdu[off + 1U]) *
					 COE_QUADLET;
			struct can_frame frame;

			if (msg_len < COE_ACF_HDR_LEN || off + msg_len > end) {
				LOG_WRN("bus%u: truncated ACF message", (unsigned int)index);
				break;
			}
			if (type != COE_ACF_TYPE_CAN || msg_len < COE_ACF_CAN_HDR_LEN) {
				off += msg_len;
				continue;
			}
			if (coe_acf_can_decode(&pdu[off], msg_len, &frame) != 0) {
				LOG_WRN("bus%u: malformed ACF-CAN message", (unsigned int)index);
				off += msg_len;
				continue;
			}
			off += msg_len;

			/* Hand the frame to the bus writer instead of transmitting
			 * it here: a bus whose frames go unacknowledged must not
			 * hold up the decoding of traffic bound for the other bus.
			 */
			if (k_msgq_put(bus->txq, &frame, K_NO_WAIT) != 0) {
				bus->tx_drop++;
				if (bus->txq_up) {
					LOG_WRN("bus%u: bus queue full, frames dropped",
						(unsigned int)index);
					bus->txq_up = false;
				}
			} else if (!bus->txq_up) {
				LOG_INF("bus%u: bus queue draining, %u frames dropped",
					(unsigned int)index, bus->tx_drop);
				bus->txq_up = true;
			}
		}
	}
}

/* Drains one bus queue onto its CAN controller, one bus per thread. */
static void coe_bus_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	unsigned int index = (unsigned int)(uintptr_t)a;
	struct coe_bus *bus = &g_bus[index];
	struct can_frame frame;
	bool can_up = true;

	coe_wait_ready();

	while (true) {
		k_msgq_get(bus->txq, &frame, K_FOREVER);

		if (can_send(bus->dev, &frame, K_MSEC(100), NULL, NULL) == 0) {
			bus->tx_can++;
			if (!can_up) {
				LOG_INF("bus%u: can transmit up", index);
				can_up = true;
			}
		} else if (can_up) {
			LOG_WRN("bus%u: can_send failed", index);
			can_up = false;
		}
	}
}

K_THREAD_DEFINE(coe_tx, 4096, coe_tx_thread, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(coe_rx, 4096, coe_rx_thread, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(coe_bus0, 2048, coe_bus_thread, (void *)(uintptr_t)0, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(coe_bus1, 2048, coe_bus_thread, (void *)(uintptr_t)1, NULL, NULL, 6, 0, 0);

#if defined(CONFIG_SHELL)
/* Bench read-out of the counters each path keeps, reachable over the same
 * mcumgr shell transport as the rest of the diagnostics.
 */
static int cmd_coe_stats(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t mac[NET_ETH_ADDR_LEN];
	k_spinlock_key_t key;
	bool learned;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (uint8_t i = 0; i < COE_BUS_COUNT; i++) {
		const struct coe_bus *bus = &g_bus[i];

		shell_print(sh,
			    "bus%u: rx_can %u tx_can %u tx_drop %u rx_pdu %u tx_pdu %u "
			    "tx_err %u errno %d",
			    (unsigned int)i, bus->rx_can, bus->tx_can, bus->tx_drop,
			    bus->rx_pdu, bus->tx_pdu, bus->tx_err, bus->tx_errno_last);
	}

	key = k_spin_lock(&g_dst_lock);
	memcpy(mac, g_dst_mac, sizeof(mac));
	learned = g_peer_known;
	k_spin_unlock(&g_dst_lock, key);

	shell_print(sh, "dst %02x:%02x:%02x:%02x:%02x:%02x (%s), phc %s, timestamps %s",
		    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
		    learned ? "learned" : "configured",
		    (g_phc != NULL) ? g_phc->name : "none",
		    coe_disciplined() ? "disciplined" : "withheld");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_coe,
			       SHELL_CMD(stats, NULL, "Per bus counters and bridge state.",
					 cmd_coe_stats),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(coe, &sub_coe, "CAN over Ethernet commands", NULL);
#endif /* CONFIG_SHELL */

int main(void)
{
	printk("CogniPilot Spinali: CAN over Ethernet\n");

	if (!coe_parse_mac(CONFIG_SPINALI_COE_DST_MAC, g_dst_mac)) {
		LOG_WRN("cannot parse destination MAC \"%s\", using the built-in default",
			CONFIG_SPINALI_COE_DST_MAC);
		memcpy(g_dst_mac, g_dst_mac_fallback, sizeof(g_dst_mac));
	}

	g_ifindex = net_if_get_by_iface(net_if_get_default());
	if (g_ifindex < 0) {
		LOG_ERR("no default network interface");
		return 0;
	}

	/*
	 * The talker half of every stream ID is this interface's MAC address,
	 * which the unique-mac driver derives from the chip UUID, so the stream
	 * IDs are stable and unique per board with no coordination.
	 */
	uint64_t mac48 = 0;
	struct net_linkaddr *ll = net_if_get_link_addr(net_if_get_default());

	if (ll != NULL && ll->len == 6U) {
		for (int b = 0; b < 6; b++) {
			mac48 = (mac48 << 8) | ll->addr[b];
		}
	} else {
		LOG_ERR("interface MAC unavailable; stream IDs will not be unique");
	}

	/*
	 * The clock that timestamps arriving frames is the one the Ethernet MAC
	 * already runs for gPTP, so a timestamp on the wire means the same
	 * instant to every node the grandmaster serves. It is resolved once
	 * here, ahead of the receive filters, so the interrupt path only ever
	 * reads it.
	 */
	g_phc = net_eth_get_ptp_clock(net_if_get_default());
	if (g_phc == NULL) {
		LOG_WRN("no PTP clock on the default interface, frames go out untimestamped");
	} else if (!device_is_ready(g_phc)) {
		LOG_WRN("PTP clock %s not ready, frames go out untimestamped", g_phc->name);
		g_phc = NULL;
	}

	/*
	 * Transmission and reception own separate sockets. A socket call holds
	 * the lock of its file descriptor for the whole call, so a receive
	 * blocked waiting for a packet would hold off every send issued on that
	 * same descriptor until inbound traffic happened to release it, which
	 * makes full duplex over one socket impossible.
	 *
	 * Only the receiving socket carries the AVTP ethertype. The
	 * transmitting socket takes protocol 0, which inbound delivery never
	 * matches, so its receive path stays empty and contends with nothing.
	 * That costs no reach on the wire: the link layer protocol type of a
	 * transmitted packet comes from the destination address of the send,
	 * which carries the AVTP ethertype, and binding supplies the egress
	 * interface and the source MAC address.
	 */
	g_sock_rx = zsock_socket(AF_PACKET, SOCK_DGRAM, htons(ETH_P_TSN));
	if (g_sock_rx < 0) {
		LOG_ERR("receive packet socket failed: %d", errno);
		return 0;
	}

	g_sock_tx = zsock_socket(AF_PACKET, SOCK_DGRAM, 0);
	if (g_sock_tx < 0) {
		LOG_ERR("transmit packet socket failed: %d", errno);
		return 0;
	}

	struct sockaddr_ll local_rx = {
		.sll_family = AF_PACKET,
		.sll_protocol = htons(ETH_P_TSN),
		.sll_ifindex = g_ifindex,
	};
	struct sockaddr_ll local_tx = {
		.sll_family = AF_PACKET,
		.sll_protocol = 0,
		.sll_ifindex = g_ifindex,
	};

	if (zsock_bind(g_sock_rx, (struct sockaddr *)&local_rx, sizeof(local_rx)) < 0) {
		LOG_ERR("receive bind to interface %d failed: %d", g_ifindex, errno);
		return 0;
	}

	if (zsock_bind(g_sock_tx, (struct sockaddr *)&local_tx, sizeof(local_tx)) < 0) {
		LOG_ERR("transmit bind to interface %d failed: %d", g_ifindex, errno);
		return 0;
	}

	bool started[COE_BUS_COUNT] = {false};

	for (uint8_t i = 0; i < COE_BUS_COUNT; i++) {
		struct coe_bus *bus = &g_bus[i];

		bus->stream_id = COE_STREAM_ID(mac48, COE_STREAM_UID_BASE + i);

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

		started[i] = true;
	}

	k_sem_give(&g_ready);

	/*
	 * The catch-all receive filters come last, once the socket is open and
	 * the transport threads are released: a filter installed any earlier
	 * would fill the transport queue against gated consumers and shed
	 * frames on a bus that is already busy at boot.
	 */
	for (uint8_t i = 0; i < COE_BUS_COUNT; i++) {
		const struct can_filter fstd = {.id = 0, .mask = 0, .flags = 0};
		const struct can_filter fext = {.id = 0, .mask = 0, .flags = CAN_FILTER_IDE};
		struct coe_bus *bus = &g_bus[i];

		if (!started[i]) {
			continue;
		}

		(void)can_add_rx_filter(bus->dev, coe_rx_cb, (void *)(uintptr_t)i, &fstd);
		(void)can_add_rx_filter(bus->dev, coe_rx_cb, (void *)(uintptr_t)i, &fext);

		LOG_INF("can%u: bridged on stream 0x%016llx", i,
			(unsigned long long)bus->stream_id);
	}

	return 0;
}

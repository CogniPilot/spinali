/*
 * Copyright (c) 2025 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 *
 * GNSS PPS capture: hardware-timestamp the F9P TIMEPULSE edge against
 * the ENET QoS PTP hardware clock (PHC).
 *
 * The TIMEPULSE pin (P3_8) is muxed to CT_INP4 and routed through
 * INPUTMUX to CTIMER4 capture channel 0. CTIMER4 and the ENET PTP
 * reference both run from PLL0 at 150 MHz, so the pulse edge is latched
 * in hardware with ~6.7 ns resolution and zero interrupt-latency error:
 * the ISR only translates the latched tick count to PHC time via a
 * bracketed read.
 *
 * Each edge is paired with the most recent UBX-TIM-TP message, which
 * carries the GNSS time-of-week of exactly this pulse. The pair
 * (PHC-time-of-edge, GNSS-time-of-edge) is the input for disciplining
 * the PHC to GNSS time.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/drivers/gnss/u_blox_f9p.h>
#include <zephyr/dt-bindings/counter/counter-capture.h>
#include <zephyr/logging/log.h>

#include "pps_servo.h"

LOG_MODULE_REGISTER(pps_capture, LOG_LEVEL_INF);

#define PPS_COUNTER_NODE DT_NODELABEL(ctimer4)
#define PPS_PHC_NODE	 DT_NODELABEL(enet_ptp_clock)
#define PPS_GNSS_NODE	 DT_NODELABEL(gnss1)
#define PPS_CAPTURE_CHAN 0

/* CTIMER4 and the ENET PTP reference share PLL0 at 150 MHz. At the
 * PHC's nominal addend it advances exactly 20/3 ns per CTIMER tick.
 * Once the discipline servo starts trimming the addend this constant
 * becomes servo-owned; the tick deltas here are small (ISR latency)
 * so the induced error stays sub-ns either way.
 */
#define PPS_TICK_NS_NUM 20U
#define PPS_TICK_NS_DEN 3U

struct pps_capture_state {
	const struct device *counter;
	const struct device *phc;
	const struct device *gnss;
	uint64_t last_edge_ns;
	uint32_t edge_count;
};

static struct pps_capture_state pps_state;

static void pps_capture_cb(const struct device *dev, uint8_t chan,
			   counter_capture_flags_t flags, uint32_t ticks,
			   void *user_data)
{
	ARG_UNUSED(chan);
	ARG_UNUSED(flags);

	struct pps_capture_state *st = user_data;
	struct net_ptp_time now;
	uint32_t t_now = 0;

	/* Bracketed read: captured tick -> PHC time. Both counters tick
	 * from PLL0, so the translation is exact arithmetic.
	 */
	(void)counter_get_value(dev, &t_now);
	ptp_clock_get(st->phc, &now);

	uint32_t lat_ticks = t_now - ticks;
	uint64_t lat_ns = ((uint64_t)lat_ticks * PPS_TICK_NS_NUM) / PPS_TICK_NS_DEN;
	uint64_t p_now_ns = ((uint64_t)now.second * NSEC_PER_SEC) + now.nanosecond;
	uint64_t edge_ns = p_now_ns - lat_ns;

	int64_t period_ns = 0;

	if (st->edge_count > 0U) {
		period_ns = (int64_t)(edge_ns - st->last_edge_ns);
	}
	st->last_edge_ns = edge_ns;
	st->edge_count++;

	LOG_DBG("PPS #%u: PHC %llu.%09u s, period %lld ns, lat %u ticks",
		st->edge_count, (unsigned long long)(edge_ns / NSEC_PER_SEC),
		(uint32_t)(edge_ns % NSEC_PER_SEC), (long long)period_ns, lat_ticks);

	pps_servo_edge(edge_ns, lat_ticks);
}

static int pps_capture_init(void)
{
	int err;

	pps_state.counter = DEVICE_DT_GET(PPS_COUNTER_NODE);
	pps_state.phc = DEVICE_DT_GET(PPS_PHC_NODE);
	pps_state.gnss = DEVICE_DT_GET(PPS_GNSS_NODE);

	if (!device_is_ready(pps_state.counter) || !device_is_ready(pps_state.phc) ||
	    !device_is_ready(pps_state.gnss)) {
		LOG_ERR("PPS capture devices not ready (counter %d, phc %d, gnss %d)",
			device_is_ready(pps_state.counter), device_is_ready(pps_state.phc),
			device_is_ready(pps_state.gnss));
		return -ENODEV;
	}

	err = counter_capture_configure(pps_state.counter, PPS_CAPTURE_CHAN,
					COUNTER_CAPTURE_RISING_EDGE | COUNTER_CAPTURE_CONTINUOUS,
					pps_capture_cb, &pps_state);
	if (err != 0) {
		LOG_ERR("Failed to configure PPS capture: %d", err);
		return err;
	}

	err = counter_start(pps_state.counter);
	if (err != 0) {
		LOG_ERR("Failed to start PPS counter: %d", err);
		return err;
	}

	err = counter_enable_capture(pps_state.counter, PPS_CAPTURE_CHAN);
	if (err != 0) {
		LOG_ERR("Failed to enable PPS capture: %d", err);
		return err;
	}

	LOG_INF("GNSS PPS capture armed (CT_INP4 -> CTIMER4 ch0, PLL0/150 MHz)");

	return 0;
}

SYS_INIT(pps_capture_init, APPLICATION, 99);

/*
 * Copyright (c) 2025 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 *
 * Grandmaster absolute-accuracy audit: measures true PHC-vs-GNSS error
 * with the receiver itself as the reference instrument, no external
 * test equipment involved.
 *
 * A GPIO line wired to the F9P's EXTINT input is driven high at a PHC
 * time the application measures precisely (a bracketed read around the
 * pin write). The receiver timestamps that same edge in GNSS time and
 * reports it as UBX-TIM-TM2. The difference between the two is the
 * absolute error of the disciplined clock, measured end to end through
 * the hardware rather than inferred from the servo's own residuals.
 *
 * This is an independent instrument, not part of the control loop: the
 * discipline servo steers the PHC from the TIMEPULSE input, while this
 * module only observes. The audit therefore also covers error sources
 * the servo cannot see, such as a systematic bias in the pulse path.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/drivers/gnss/u_blox_f9p.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tm_audit, LOG_LEVEL_INF);

#define TM_AUDIT_PHC_NODE  DT_NODELABEL(enet_ptp_clock)
#define TM_AUDIT_GNSS_NODE DT_NODELABEL(gnss1)
#define TM_AUDIT_GPIO_NODE DT_PATH(zephyr_user)

BUILD_ASSERT(DT_NODE_HAS_PROP(TM_AUDIT_GPIO_NODE, timemark_gpios),
	     "zephyr,user timemark-gpios is required by the grandmaster audit");

/* GPS epoch (1980-01-06 00:00:00 UTC) in Unix seconds, and the fixed
 * GPS-to-TAI offset (TAI = GPS + 19 s).
 */
#define GPS_EPOCH_UNIX	 315964800LL
#define GPS_TAI_OFFSET	 19LL
#define SECONDS_PER_WEEK 604800LL

/* Values of the TIM-TM2 timebase field (flags bits 3-4), read with
 * UBX_TIM_TM2_FLAGS_TIME_BASE(): 0 is receiver-local time.
 */
#define TM_AUDIT_TIME_BASE_GNSS 1
#define TM_AUDIT_TIME_BASE_UTC	2

/* One mark per period, emitted in phases so the workqueue is never
 * blocked waiting on the pin or on the receiver:
 *
 *   ARM  : pin driven low, settling for SETTLE_MS (> 1 ms)
 *   EDGE : bracketed rising edge
 *   CLEAR: pin returned to idle, PULSE_MS after the edge
 *   PAIR : TIM-TM2 fetched and scored, PAIR_MS after the edge
 *
 * SETTLE_MS + PAIR_MS + REST_MS is exactly one period, so marks are
 * emitted at a steady rate.
 */
#define TM_AUDIT_PERIOD_MS 10000
#define TM_AUDIT_SETTLE_MS 2
#define TM_AUDIT_PULSE_MS  10
#define TM_AUDIT_PAIR_MS   1490
#define TM_AUDIT_REST_MS   (TM_AUDIT_PERIOD_MS - TM_AUDIT_SETTLE_MS - TM_AUDIT_PAIR_MS)

enum tm_audit_phase {
	TM_AUDIT_ARM,
	TM_AUDIT_EDGE,
	TM_AUDIT_CLEAR,
	TM_AUDIT_PAIR,
};

struct tm_audit {
	const struct device *phc;
	const struct device *gnss;

	struct k_work_delayable work;
	enum tm_audit_phase phase;

	uint64_t edge_ns;     /* PHC time of the emitted edge (midpoint) */
	uint32_t edge_win_ns; /* bracket width; uncertainty is half of it */
	bool edge_valid;

	uint32_t last_seq;
	bool have_seq;

	uint32_t count;
	int64_t sum_ns;
	int64_t min_ns;
	int64_t max_ns;
};

static const struct gpio_dt_spec tm_gpio =
	GPIO_DT_SPEC_GET(TM_AUDIT_GPIO_NODE, timemark_gpios);

static struct tm_audit audit;

/* Convert the rising-edge fields of a TIM-TM2 mark to PTP (TAI-based)
 * nanoseconds. Mirrors the TIM-TP conversion used by the discipline
 * servo, applied to the mark's own week / time-of-week fields.
 *
 * Returns 0 on success, -EAGAIN when leap-second data is required but
 * absent, -ENOTSUP when the mark carries receiver-local time only.
 */
static int mark_to_ptp_ns(const struct ubx_tim_tm2 *tm2, uint64_t *ptp_ns)
{
	int64_t tow_s = tm2->tow_ms_r / 1000U;
	int64_t ns = (int64_t)(tm2->tow_ms_r % 1000U) * NSEC_PER_MSEC;

	/* TIM-TM2 states the sub-millisecond part of a mark directly in
	 * nanoseconds, unlike TIM-TP's 2^-32 ms scaling.
	 */
	ns += (int64_t)tm2->tow_sub_ms_r;

	int64_t total_s = GPS_EPOCH_UNIX + (int64_t)tm2->wn_r * SECONDS_PER_WEEK + tow_s;

	switch (UBX_TIM_TM2_FLAGS_TIME_BASE(tm2->flags)) {
	case TM_AUDIT_TIME_BASE_GNSS:
		/* PTP = GPS + 19 */
		total_s += GPS_TAI_OFFSET;
		break;

	case TM_AUDIT_TIME_BASE_UTC: {
		/* PTP = UTC + (19 + leap) */
		struct ubx_nav_timels timels;

		if ((u_blox_f9p_leap_get(audit.gnss, &timels) != 0) ||
		    ((timels.valid & UBX_NAV_TIMELS_VALID_CURR_LS) == 0U)) {
			return -EAGAIN;
		}
		total_s += GPS_TAI_OFFSET + timels.curr_ls;
		break;
	}

	default:
		/* Receiver-local time: no relation to GNSS time. */
		return -ENOTSUP;
	}

	*ptp_ns = (uint64_t)total_s * NSEC_PER_SEC + (uint64_t)ns;

	return 0;
}

/* Emission gate. The audit is only meaningful once the receiver has
 * resolved the GNSS timescale, which is also the precondition for the
 * discipline servo to be tracking at all. Leap data (NAV-TIMELS with a
 * valid current count) proves timescale resolution without reaching
 * into the servo's state, and the age of the last mark proves the
 * receiver is still reporting.
 */
static bool tm_audit_receiver_ready(void)
{
	struct ubx_nav_timels timels;

	if ((u_blox_f9p_leap_get(audit.gnss, &timels) != 0) ||
	    ((timels.valid & UBX_NAV_TIMELS_VALID_CURR_LS) == 0U)) {
		LOG_WRN("Audit gated: leap-second data not available");
		return false;
	}

	/* Mark freshness is deliberately NOT gated here: reset transients
	 * on the EXTINT line can leave one spurious mark recorded before
	 * the first emission, and marks only refresh after an emission, so
	 * a staleness precondition would deadlock. Stale marks are instead
	 * rejected at pairing time by the sequence check.
	 */
	return true;
}

/* Emit the audit edge and record the PHC time at which it happened.
 *
 * The pin write is bracketed by two PHC reads with interrupts locked:
 *
 *	t0 = PHC read ; pin driven high ; t1 = PHC read
 *
 * The edge therefore falls between t0 and t1. Its time is taken as the
 * midpoint of that window and half the window is the measurement
 * uncertainty, recorded and logged with every sample. Locking
 * interrupts bounds the window to two register reads plus one GPIO
 * write; without the lock an interrupt landing inside the bracket
 * would widen it arbitrarily. gpio_pin_set_dt() resolves to a single
 * set-register write on this GPIO controller, so the bracket is only a
 * few hundred nanoseconds wide and is dominated by the PHC reads.
 */
static void tm_audit_emit(void)
{
	struct net_ptp_time t0;
	struct net_ptp_time t1;
	unsigned int key;

	key = irq_lock();
	(void)ptp_clock_get(audit.phc, &t0);
	(void)gpio_pin_set_dt(&tm_gpio, 1);
	(void)ptp_clock_get(audit.phc, &t1);
	irq_unlock(key);

	uint64_t t0_ns = ((uint64_t)t0.second * NSEC_PER_SEC) + t0.nanosecond;
	uint64_t t1_ns = ((uint64_t)t1.second * NSEC_PER_SEC) + t1.nanosecond;

	audit.edge_win_ns = (uint32_t)(t1_ns - t0_ns);
	audit.edge_ns = t0_ns + (audit.edge_win_ns / 2U);
	audit.edge_valid = true;
}

/* Pair the emitted edge with the receiver's timestamp of it and fold
 * the result into the session statistics.
 */
static void tm_audit_pair(void)
{
	struct ubx_tim_tm2 tm2;
	uint32_t seq = 0;
	uint64_t gnss_ns = 0;
	bool advanced;
	int err;

	if (!audit.edge_valid) {
		return;
	}
	audit.edge_valid = false;

	if (u_blox_f9p_timemark_get(audit.gnss, &tm2, NULL, &seq) != 0) {
		LOG_WRN("No TIM-TM2 mark reported for the audit edge");
		return;
	}

	advanced = !audit.have_seq || (seq != audit.last_seq);
	audit.last_seq = seq;
	audit.have_seq = true;

	if (!advanced) {
		LOG_WRN("Stale TIM-TM2 (seq %u) - skipping audit sample", seq);
		return;
	}

	if ((tm2.flags & UBX_TIM_TM2_FLAGS_NEW_RISING_EDGE) == 0U) {
		LOG_WRN("TIM-TM2 reports no new rising edge - skipping audit sample");
		return;
	}

	if ((tm2.flags & UBX_TIM_TM2_FLAGS_TIME_VALID) == 0U) {
		LOG_WRN("TIM-TM2 time not valid - skipping audit sample");
		return;
	}

	err = mark_to_ptp_ns(&tm2, &gnss_ns);
	if (err == -EAGAIN) {
		LOG_WRN("Leap-second data unavailable - skipping audit sample");
		return;
	} else if (err != 0) {
		LOG_WRN("TIM-TM2 on the receiver timebase - skipping audit sample");
		return;
	}

	int64_t err_ns = (int64_t)(gnss_ns - audit.edge_ns);

	/* Gross outliers (clock still settling, mis-paired epoch) are
	 * reported but kept out of the session statistics.
	 */
	if ((err_ns > 5000) || (err_ns < -5000)) {
		LOG_WRN("audit: outlier %+lld ns excluded from statistics", err_ns);
		return;
	}

	audit.count++;
	audit.sum_ns += err_ns;
	if ((audit.count == 1U) || (err_ns < audit.min_ns)) {
		audit.min_ns = err_ns;
	}
	if ((audit.count == 1U) || (err_ns > audit.max_ns)) {
		audit.max_ns = err_ns;
	}

	LOG_INF("audit: err %+lld ns (edge window %u ns, rx acc %u ns), "
		"n %u mean %+lld ns, min %+lld, max %+lld",
		err_ns, audit.edge_win_ns, tm2.acc_est, audit.count,
		audit.sum_ns / (int64_t)audit.count, audit.min_ns, audit.max_ns);
}

static void tm_audit_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	switch (audit.phase) {
	case TM_AUDIT_ARM:
		if (!tm_audit_receiver_ready()) {
			/* Nothing is emitted while the receiver has no
			 * usable time; retry a full period later.
			 */
			k_work_reschedule(&audit.work, K_MSEC(TM_AUDIT_PERIOD_MS));
			break;
		}

		(void)gpio_pin_set_dt(&tm_gpio, 0);
		audit.phase = TM_AUDIT_EDGE;
		k_work_reschedule(&audit.work, K_MSEC(TM_AUDIT_SETTLE_MS));
		break;

	case TM_AUDIT_EDGE:
		tm_audit_emit();
		audit.phase = TM_AUDIT_CLEAR;
		k_work_reschedule(&audit.work, K_MSEC(TM_AUDIT_PULSE_MS));
		break;

	case TM_AUDIT_CLEAR:
		(void)gpio_pin_set_dt(&tm_gpio, 0);
		audit.phase = TM_AUDIT_PAIR;
		k_work_reschedule(&audit.work,
				  K_MSEC(TM_AUDIT_PAIR_MS - TM_AUDIT_PULSE_MS));
		break;

	case TM_AUDIT_PAIR:
		tm_audit_pair();
		audit.phase = TM_AUDIT_ARM;
		k_work_reschedule(&audit.work, K_MSEC(TM_AUDIT_REST_MS));
		break;
	}
}

static int tm_audit_init(void)
{
	int err;

	audit.phc = DEVICE_DT_GET(TM_AUDIT_PHC_NODE);
	audit.gnss = DEVICE_DT_GET(TM_AUDIT_GNSS_NODE);
	audit.phase = TM_AUDIT_ARM;

	if (!device_is_ready(audit.phc) || !device_is_ready(audit.gnss) ||
	    !device_is_ready(tm_gpio.port)) {
		LOG_ERR("Audit devices not ready (phc %d, gnss %d, gpio %d)",
			device_is_ready(audit.phc), device_is_ready(audit.gnss),
			device_is_ready(tm_gpio.port));
		return -ENODEV;
	}

	/* Idle low: the receiver is configured to mark rising edges. */
	err = gpio_pin_configure_dt(&tm_gpio, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		LOG_ERR("Failed to configure the timemark GPIO: %d", err);
		return err;
	}

	k_work_init_delayable(&audit.work, tm_audit_work_handler);
	k_work_schedule(&audit.work, K_MSEC(TM_AUDIT_PERIOD_MS));

	LOG_INF("Grandmaster audit armed (EXTINT mark every %d s)",
		TM_AUDIT_PERIOD_MS / 1000);

	return 0;
}

SYS_INIT(tm_audit_init, APPLICATION, 99);

/*
 * Copyright (c) 2025 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 *
 * GNSS PPS discipline servo: steers the ENET QoS PTP hardware clock
 * (PHC) onto GNSS time.
 *
 * Input:  hardware-captured PHC time of each TIMEPULSE edge (from
 *         pps_capture) paired with the UBX-TIM-TP message labelling
 *         that exact edge with GNSS time-of-week.
 * Output: one initial phase step (ptp_clock_set), then rate-only
 *         corrections (ptp_clock_rate_adjust) from a PI controller,
 *         so PHC time stays monotonic while tracking GNSS.
 *
 * The PHC runs on the PTP timescale (TAI-based, PTP epoch 1970-01-01
 * TAI): PTP seconds = GPS seconds + GPS_TAI_OFFSET, with GPS time
 * derived from TIM-TP week/tow. TIM-TP defaults to the UTC timebase,
 * converted using the leap-second count from UBX-NAV-TIMELS.
 *
 * On first lock (and periodically after) the battery-domain RTC is
 * set from GNSS so later boots can seed the PHC with plausible time
 * before a fix is available.
 *
 * Servo state also drives what the node announces as a gPTP
 * grandmaster: the clock quality and the UTC/leap time properties are
 * updated on lock and on holdover, so the node never advertises GNSS
 * traceability it does not currently have. The same state is published
 * through pps_servo_disciplined(), so an application that timestamps
 * traffic from the PHC can tell a disciplined reading from the boot
 * seed it starts out with.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/drivers/gnss/u_blox_f9p.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/gptp.h>
#include <zephyr/sys/atomic.h>

#include "pps_servo.h"

LOG_MODULE_REGISTER(pps_servo, LOG_LEVEL_INF);

#define PHC_NODE  DT_NODELABEL(enet_ptp_clock)
#define GNSS_NODE DT_NODELABEL(gnss1)
#define RTC_NODE  DT_NODELABEL(rtc)

/* GPS epoch (1980-01-06 00:00:00 UTC) in Unix seconds, and the fixed
 * GPS-to-TAI offset (TAI = GPS + 19 s).
 */
#define GPS_EPOCH_UNIX	  315964800LL
#define GPS_TAI_OFFSET	  19LL
#define SECONDS_PER_WEEK  604800LL

/* PI gains for the 1 Hz loop, in ppb per ns of phase error. */
#define SERVO_KP	  0.4
#define SERVO_KI	  0.08
#define INTEG_CLAMP_PPB	  50000.0
#define CORR_CLAMP_PPB	  100000.0

#define STEP_THRESHOLD_NS 100000000LL /* |err| above this: step, don't slew */
#define SEED_STEP_THRESHOLD_NS 1000000LL /* pre-first-lock step threshold */
#define LOCK_THRESHOLD_NS 1000LL      /* |err| below this counts toward lock */
#define UNLOCK_THRESHOLD_NS 10000LL   /* |err| above this drops lock */
#define LOCK_COUNT	  5
#define RTC_UPDATE_PERIOD_S 3600
#define TP_STALE_MS	  1500

/* Announced grandmaster quality. Class 6 is a clock synchronized to a
 * primary reference, class 7 the same clock in holdover. Accuracy 0x21
 * (100 ns) matches the locked servo; 0x27 (100 us) bounds the drift of
 * the frozen learned rate over an outage long enough to matter. Time
 * source 0x20 is GPS. Anything before the first lock stays at the
 * Kconfig defaults (class 248, internal oscillator).
 */
#define GM_CLASS_LOCKED	   6
#define GM_CLASS_HOLDOVER  7
#define GM_ACCURACY_100NS  0x21
#define GM_ACCURACY_100US  0x27
#define GM_TIME_SOURCE_GPS 0x20

enum servo_state {
	SERVO_WAIT_LABEL,
	SERVO_TRACK,
};

struct pps_servo {
	const struct device *phc;
	const struct device *gnss;
	const struct device *rtc;

	struct k_spinlock lock;
	uint64_t pending_edge_ns;
	uint32_t pending_lat_ticks;
	bool pending;

	enum servo_state state;
	uint32_t last_tp_seq;
	double integ_ppb;
	bool locked;
	/* Set on the first lock and never cleared: from then on the PHC is
	 * on the GNSS timescale, and holdover keeps it there by coasting on
	 * the learned rate. Published atomically because consumers of the
	 * timestamps this discipline underwrites read it from interrupt
	 * context through pps_servo_disciplined().
	 */
	atomic_t ever_locked;
	uint32_t lock_streak;
	uint32_t sample_count;
	int64_t last_rtc_update_s;

	uint8_t announced_class;
	int16_t utc_offset;
	bool utc_offset_valid;
	bool leap61;
	bool leap59;

	struct k_work work;
	struct k_work_delayable holdover_work;
};

static struct pps_servo servo;

/* Days-from-civil inverse (Howard Hinnant's algorithm) for RTC set. */
static void unix_to_civil(int64_t unix_s, struct rtc_time *out)
{
	int64_t days = unix_s / 86400;
	int64_t rem = unix_s % 86400;

	if (rem < 0) {
		rem += 86400;
		days -= 1;
	}

	int64_t z = days + 719468;
	int64_t era = (z >= 0 ? z : z - 146096) / 146097;
	int64_t doe = z - era * 146097;
	int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
	int64_t y = yoe + era * 400;
	int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
	int64_t mp = (5 * doy + 2) / 153;
	int64_t d = doy - (153 * mp + 2) / 5 + 1;
	int64_t m = mp < 10 ? mp + 3 : mp - 9;

	if (m <= 2) {
		y += 1;
	}

	out->tm_year = (int)(y - 1900);
	out->tm_mon = (int)(m - 1);
	out->tm_mday = (int)d;
	out->tm_hour = (int)(rem / 3600);
	out->tm_min = (int)((rem % 3600) / 60);
	out->tm_sec = (int)(rem % 60);
	out->tm_wday = (int)((days + 4) % 7); /* 1970-01-01 was a Thursday */
	out->tm_yday = -1;
	out->tm_isdst = -1;
	out->tm_nsec = 0;
}

/* Convert the TIM-TP label of an edge to PTP (TAI-based) nanoseconds.
 * Returns 0 on success, -EAGAIN when leap data is required but absent.
 */
static int label_to_ptp_ns(const struct ubx_tim_tp *tp, uint64_t *ptp_ns)
{
	int64_t tow_s = tp->tow_ms / 1000;
	int64_t ns = (int64_t)(tp->tow_ms % 1000) * NSEC_PER_MSEC;

	/* Sub-millisecond part: 2^-32 ms units -> ns. */
	ns += ((int64_t)tp->tow_sub_ms * NSEC_PER_MSEC) >> 32;

	if ((tp->flags & UBX_TIM_TP_FLAGS_QERR_INVALID) == 0U) {
		ns += tp->q_err / 1000; /* ps -> ns, signed */
	}

	int64_t total_s = GPS_EPOCH_UNIX + (int64_t)tp->week * SECONDS_PER_WEEK + tow_s;

	if ((tp->flags & UBX_TIM_TP_FLAGS_TIME_BASE_UTC) != 0U) {
		/* UTC timebase: PTP = UTC + (19 + leap) */
		struct ubx_nav_timels timels;

		if ((u_blox_f9p_leap_get(servo.gnss, &timels) != 0) ||
		    ((timels.valid & UBX_NAV_TIMELS_VALID_CURR_LS) == 0U)) {
			return -EAGAIN;
		}
		total_s += GPS_TAI_OFFSET + timels.curr_ls;
	} else {
		/* GPS timebase: PTP = GPS + 19 */
		total_s += GPS_TAI_OFFSET;
	}

	if (ns < 0) {
		ns += NSEC_PER_SEC;
		total_s -= 1;
	}

	*ptp_ns = (uint64_t)total_s * NSEC_PER_SEC + (uint64_t)ns;

	return 0;
}

static void servo_update_rtc(uint64_t gnss_ptp_ns)
{
	struct ubx_nav_timels timels;
	struct rtc_time rt;
	int64_t tai_s = (int64_t)(gnss_ptp_ns / NSEC_PER_SEC);
	int err;

	if (servo.rtc == NULL) {
		return;
	}

	if ((u_blox_f9p_leap_get(servo.gnss, &timels) != 0) ||
	    ((timels.valid & UBX_NAV_TIMELS_VALID_CURR_LS) == 0U)) {
		return;
	}

	unix_to_civil(tai_s - GPS_TAI_OFFSET - timels.curr_ls, &rt);

	err = rtc_set_time(servo.rtc, &rt);
	if (err != 0) {
		LOG_WRN("RTC set failed: %d", err);
	} else {
		servo.last_rtc_update_s = tai_s;
		LOG_INF("RTC set from GNSS: %04d-%02d-%02d %02d:%02d:%02d UTC",
			rt.tm_year + 1900, rt.tm_mon + 1, rt.tm_mday,
			rt.tm_hour, rt.tm_min, rt.tm_sec);
	}
}

/* Refresh the announced UTC offset and leap-second flags from the
 * latest NAV-TIMELS report, and re-announce them. The cached values are
 * kept when the receiver has no valid leap data, so a holdover
 * announcement carries the last known UTC offset forward.
 */
static void servo_announce_time_props(void)
{
	struct ubx_nav_timels timels;

	if ((u_blox_f9p_leap_get(servo.gnss, &timels) == 0) &&
	    ((timels.valid & UBX_NAV_TIMELS_VALID_CURR_LS) != 0U)) {
		servo.utc_offset = (int16_t)(GPS_TAI_OFFSET + timels.curr_ls);
		servo.utc_offset_valid = true;
		servo.leap61 = (timels.ls_change > 0);
		servo.leap59 = (timels.ls_change < 0);
	}

	/* Frequency is traceable only while the servo is steering the PHC
	 * from PPS; time stays traceable through holdover.
	 */
	gptp_update_time_properties(servo.utc_offset, servo.utc_offset_valid,
				    servo.leap61, servo.leap59, true,
				    servo.announced_class == GM_CLASS_LOCKED);
}

/* Announce a change of grandmaster quality. Repeated events at the
 * already-announced class are dropped.
 */
static void servo_announce(uint8_t clock_class, const char *reason)
{
	if (clock_class == servo.announced_class) {
		return;
	}

	/* Holdover of a primary reference is only claimable by a clock
	 * that was announced as one.
	 */
	if ((clock_class == GM_CLASS_HOLDOVER) &&
	    (servo.announced_class != GM_CLASS_LOCKED)) {
		return;
	}

	const struct gptp_gm_quality quality = {
		.clock_class = clock_class,
		.clock_accuracy = (clock_class == GM_CLASS_LOCKED) ? GM_ACCURACY_100NS
								   : GM_ACCURACY_100US,
		.time_source = GM_TIME_SOURCE_GPS,
	};

	gptp_update_gm_quality(&quality);
	servo.announced_class = clock_class;

	servo_announce_time_props();

	LOG_INF("announce: clockClass %u (%s)", (unsigned int)clock_class, reason);
}

static void servo_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	uint64_t edge_ns;
	uint32_t lat_ticks;
	bool have_edge = false;

	K_SPINLOCK(&servo.lock) {
		if (servo.pending) {
			edge_ns = servo.pending_edge_ns;
			lat_ticks = servo.pending_lat_ticks;
			servo.pending = false;
			have_edge = true;
		}
	}

	if (!have_edge) {
		return;
	}

	k_work_reschedule(&servo.holdover_work, K_MSEC(2500));

	/* Pair the edge with a fresh TIM-TP label. */
	struct ubx_tim_tp tp;
	int64_t tp_uptime_ticks = 0;
	uint32_t tp_seq = 0;

	if (u_blox_f9p_timepulse_get(servo.gnss, &tp, &tp_uptime_ticks, &tp_seq) != 0) {
		LOG_WRN("PPS edge without any TIM-TP label yet");
		return;
	}

	if (tp_seq == servo.last_tp_seq) {
		LOG_WRN("Stale TIM-TP label (seq %u) - skipping sample", tp_seq);
		return;
	}

	int64_t tp_age_ms = k_ticks_to_ms_floor64(k_uptime_ticks() - tp_uptime_ticks);

	if (tp_age_ms > TP_STALE_MS) {
		LOG_WRN("TIM-TP label too old (%lld ms) - skipping sample", tp_age_ms);
		return;
	}

	servo.last_tp_seq = tp_seq;

	uint64_t gnss_ns;

	if (label_to_ptp_ns(&tp, &gnss_ns) != 0) {
		if (servo.state == SERVO_WAIT_LABEL) {
			LOG_INF("Waiting for leap-second data (NAV-TIMELS)");
		}
		return;
	}

	int64_t err_ns = (int64_t)(gnss_ns - edge_ns);

	servo.sample_count++;

	/* Before the first lock the RTC seed may leave a large offset that
	 * is still under the holdover step threshold; slewing it out would
	 * take minutes, so step on anything past the seed threshold. After
	 * a lock has ever been achieved, only step for gross errors so
	 * holdover recovery stays slew-only and monotonic.
	 */
	int64_t step_thresh = (atomic_get(&servo.ever_locked) != 0) ? STEP_THRESHOLD_NS
								    : SEED_STEP_THRESHOLD_NS;

	if ((err_ns > step_thresh) || (err_ns < -step_thresh)) {
		/* Phase step: move the PHC by the measured error. The
		 * error is applied to a fresh reading so the elapsed
		 * time since the edge is preserved.
		 */
		struct net_ptp_time now;

		ptp_clock_get(servo.phc, &now);

		uint64_t now_ns = ((uint64_t)now.second * NSEC_PER_SEC) + now.nanosecond +
				  (uint64_t)err_ns;
		struct net_ptp_time set_time = {
			.second = now_ns / NSEC_PER_SEC,
			.nanosecond = (uint32_t)(now_ns % NSEC_PER_SEC),
		};

		ptp_clock_set(servo.phc, &set_time);

		servo.locked = false;
		servo.lock_streak = 0;
		servo.state = SERVO_TRACK;

		LOG_INF("PHC stepped by %lld ns to GNSS time", err_ns);
		return;
	}

	servo.state = SERVO_TRACK;

	/* PI rate correction. err > 0 means the PHC is behind GNSS and
	 * must speed up (ratio > 1).
	 */
	servo.integ_ppb += SERVO_KI * (double)err_ns;
	if (servo.integ_ppb > INTEG_CLAMP_PPB) {
		servo.integ_ppb = INTEG_CLAMP_PPB;
	} else if (servo.integ_ppb < -INTEG_CLAMP_PPB) {
		servo.integ_ppb = -INTEG_CLAMP_PPB;
	}

	double corr_ppb = SERVO_KP * (double)err_ns + servo.integ_ppb;

	if (corr_ppb > CORR_CLAMP_PPB) {
		corr_ppb = CORR_CLAMP_PPB;
	} else if (corr_ppb < -CORR_CLAMP_PPB) {
		corr_ppb = -CORR_CLAMP_PPB;
	}

	ptp_clock_rate_adjust(servo.phc, 1.0 + (corr_ppb * 1e-9));

	/* Lock tracking */
	if ((err_ns < LOCK_THRESHOLD_NS) && (err_ns > -LOCK_THRESHOLD_NS)) {
		if (servo.lock_streak < LOCK_COUNT) {
			servo.lock_streak++;
			if ((servo.lock_streak == LOCK_COUNT) && !servo.locked) {
				servo.locked = true;
				/* Reached only on a sample the step branch above
				 * let through, so every phase step the PHC will
				 * ever take before this point has already been
				 * applied and waited out.
				 */
				atomic_set(&servo.ever_locked, 1);
				LOG_INF("Servo LOCKED (|err| < %lld ns for %d samples)",
					LOCK_THRESHOLD_NS, LOCK_COUNT);
				servo_update_rtc(gnss_ns);
				servo_announce(GM_CLASS_LOCKED, "locked");
			}
		}
	} else if ((err_ns > UNLOCK_THRESHOLD_NS) || (err_ns < -UNLOCK_THRESHOLD_NS)) {
		if (servo.locked) {
			LOG_WRN("Servo lock lost (err %lld ns)", err_ns);
			servo_announce(GM_CLASS_HOLDOVER, "lock lost");
		}
		servo.locked = false;
		servo.lock_streak = 0;
	}

	if (servo.locked &&
	    ((int64_t)(gnss_ns / NSEC_PER_SEC) - servo.last_rtc_update_s >=
	     RTC_UPDATE_PERIOD_S)) {
		servo_update_rtc(gnss_ns);
		servo_announce_time_props();
	}

	LOG_INF("servo: err %+6lld ns, corr %+6d ppb (int %+6d), lat %u, %s",
		err_ns, (int)corr_ppb, (int)servo.integ_ppb, lat_ticks,
		servo.locked ? "LOCKED" : "tracking");
}

static void holdover_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	/* No PPS edge for 2.5 s: freeze at the learned rate. */
	LOG_WRN("PPS lost - HOLDOVER at %+d ppb (crystal free-run)",
		(int)servo.integ_ppb);
	servo.locked = false;
	servo.lock_streak = 0;
	servo_announce(GM_CLASS_HOLDOVER, "holdover");
}

bool pps_servo_disciplined(void)
{
	return atomic_get(&servo.ever_locked) != 0;
}

void pps_servo_edge(uint64_t phc_edge_ns, uint32_t lat_ticks)
{
	K_SPINLOCK(&servo.lock) {
		servo.pending_edge_ns = phc_edge_ns;
		servo.pending_lat_ticks = lat_ticks;
		servo.pending = true;
	}

	k_work_submit(&servo.work);
}

static int pps_servo_init(void)
{
	servo.phc = DEVICE_DT_GET(PHC_NODE);
	servo.gnss = DEVICE_DT_GET(GNSS_NODE);
	servo.rtc = COND_CODE_1(DT_NODE_HAS_STATUS_OKAY(RTC_NODE),
				(DEVICE_DT_GET(RTC_NODE)), (NULL));
	servo.state = SERVO_WAIT_LABEL;

	k_work_init(&servo.work, servo_work_handler);
	k_work_init_delayable(&servo.holdover_work, holdover_work_handler);

	if (!device_is_ready(servo.phc) || !device_is_ready(servo.gnss)) {
		LOG_ERR("Servo devices not ready");
		return -ENODEV;
	}

	/* Seed the PHC from the RTC if it holds plausible time, so the
	 * grandmaster serves approximately-correct time before GNSS
	 * locks. TAI = UTC + 37 s as of 2026; the servo replaces this
	 * the moment real leap data arrives.
	 */
	if ((servo.rtc != NULL) && device_is_ready(servo.rtc)) {
		struct rtc_time rt;

		if ((rtc_get_time(servo.rtc, &rt) == 0) && (rt.tm_year >= 125) &&
		    (rt.tm_year <= 199)) {
			int64_t days = 0;
			int64_t y = rt.tm_year + 1900;
			int64_t m = rt.tm_mon + 1;
			int64_t d = rt.tm_mday;
			/* civil-from-days forward direction */
			int64_t yy = (m <= 2) ? y - 1 : y;
			int64_t era = (yy >= 0 ? yy : yy - 399) / 400;
			int64_t yoe = yy - era * 400;
			int64_t mp = (m + 9) % 12;
			int64_t doy = (153 * mp + 2) / 5 + d - 1;
			int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;

			days = era * 146097 + doe - 719468;

			int64_t unix_s = days * 86400 + rt.tm_hour * 3600 +
					 rt.tm_min * 60 + rt.tm_sec;
			struct net_ptp_time seed = {
				.second = (uint64_t)(unix_s + 37),
				.nanosecond = 0,
			};

			ptp_clock_set(servo.phc, &seed);
			LOG_INF("PHC seeded from RTC: %04d-%02d-%02d %02d:%02d:%02d UTC",
				(int)y, (int)m, (int)d, rt.tm_hour, rt.tm_min, rt.tm_sec);
		} else {
			LOG_INF("RTC time not plausible - PHC starts at epoch");
		}
	}

	LOG_INF("GNSS PPS discipline servo ready");

	return 0;
}

SYS_INIT(pps_servo_init, APPLICATION, 98);

/*
 * Copyright (c) 2026 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared resolver for the synapse.types.TimeStatus wire field: reports whether
 * a producer's published timestamps are on the shared gPTP grandmaster
 * timescale, and the boot-to-PHC offset that carries them there.
 */
#ifndef SYNAPSE_TIME_STATUS_H
#define SYNAPSE_TIME_STATUS_H

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#if defined(CONFIG_NET_GPTP)
#include <zephyr/net/gptp.h>
#endif

#include <synapse_topic_list.h>

/*
 * Resolve this node's clock discipline state and, when it is on the shared
 * timescale, the offset that carries a boot-clock timestamp onto it.
 *
 * gptp_event_capture reads the gPTP-disciplined PHC: the slave clock on a
 * follower node, or the node's own served clock on the grandmaster (only while
 * that served clock is disciplined, so a not-yet-locked grandmaster still reads
 * as node-local time). It also reports whether a grandmaster is present in the
 * domain.
 *
 * Sensor sample stamps are taken from the free-running kernel boot clock, a
 * different clock from the PHC, so a boot-clock timestamp is placed on the
 * shared timescale by adding the offset between the two clocks read at the same
 * instant: offset = phc_now - boot_now. Every timestamp in one published
 * message is shifted by that one offset, so the relative structure of the
 * stamps is preserved while their absolute reference moves onto the
 * GNSS-traceable domain.
 *
 * The returned time_status names the domain honestly: GptpSynced while a
 * grandmaster is present, GptpHoldover once one has been lost after a prior
 * lock (the PHC then coasts on the last learned rate), and LocalFreerun before
 * any lock or when the gPTP stack is absent, where the offset stays zero and
 * the stamps remain node-local monotonic boot time.
 *
 * The caller owns the ever-synced latch behind time_ever_synced: it is set the
 * first time a grandmaster is seen and never cleared, so a later loss is
 * published as holdover rather than as never-synchronized.
 */
static inline uint8_t synapse_time_status_resolve(bool *time_ever_synced, int64_t *offset_ns)
{
	*offset_ns = 0;

#if defined(CONFIG_NET_GPTP)
	struct net_ptp_time phc;
	bool gm_present = false;

	/* nonzero: no disciplined PHC to read. Either gPTP is not up yet, or
	 * this node is an as-yet-undisciplined grandmaster: node-local time is
	 * all there is to publish.
	 */
	if (gptp_event_capture(&phc, &gm_present) != 0) {
		return SYNAPSE_TYPES_TIME_STATUS_LOCAL_FREERUN;
	}

	/* never disciplined: the PHC carries no traceable meaning to claim */
	if (!gm_present && !*time_ever_synced) {
		return SYNAPSE_TYPES_TIME_STATUS_LOCAL_FREERUN;
	}

	int64_t phc_now_ns = (int64_t)phc.second * 1000000000LL + (int64_t)phc.nanosecond;
	int64_t boot_now_ns = k_ticks_to_ns_floor64(k_uptime_ticks());

	*offset_ns = phc_now_ns - boot_now_ns;

	if (gm_present) {
		*time_ever_synced = true;
		return SYNAPSE_TYPES_TIME_STATUS_GPTP_SYNCED;
	}

	/* was synced, grandmaster lost: the disciplined PHC now coasts */
	return SYNAPSE_TYPES_TIME_STATUS_GPTP_HOLDOVER;
#else
	ARG_UNUSED(time_ever_synced);
	return SYNAPSE_TYPES_TIME_STATUS_LOCAL_FREERUN;
#endif
}

#endif /* SYNAPSE_TIME_STATUS_H */

/* vi: ts=4 sw=4 et */

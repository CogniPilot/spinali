/*
 * Copyright (c) 2025 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SPINALI_TIMING_PPS_SERVO_H_
#define SPINALI_TIMING_PPS_SERVO_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Feed one hardware-captured PPS edge to the discipline servo.
 *
 * Callable from ISR context: records the PHC time of the edge and
 * defers all servo work (TIM-TP pairing, PI update, clock adjustment)
 * to the system workqueue.
 *
 * @param phc_edge_ns PHC time of the pulse edge, in nanoseconds.
 * @param lat_ticks   Capture-to-read latency in CTIMER ticks (diagnostic).
 */
void pps_servo_edge(uint64_t phc_edge_ns, uint32_t lat_ticks);

/**
 * @brief Report whether the PHC has been placed on the GNSS timescale.
 *
 * True once the servo has locked at least once, and never false again:
 * the initial phase step is complete by then and the clock is only
 * slewed afterwards, so the reading is on the disciplined timescale.
 * A later loss of PPS does not clear it, because the PHC then coasts on
 * the learned rate, the state the node announces as clockClass 7
 * (holdover of a primary reference) rather than as unsynchronized.
 *
 * Callable from ISR context: reads a flag the servo publishes
 * atomically from workqueue context.
 *
 * @return true when PHC readings are GNSS-traceable, false before the
 *         first lock, when they are whatever the boot seed left behind.
 */
bool pps_servo_disciplined(void);

#endif /* SPINALI_TIMING_PPS_SERVO_H_ */

/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SYNAPSE_TOPIC_LIST_H
#define SYNAPSE_TOPIC_LIST_H

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

#include <zros/zros_topic.h>

#include "synapse_argus_topic.h"
#include "synapse_gnss_topic.h"
#include "synapse_imu_topic.h"
#include "synapse_mag_topic.h"
#include "synapse_optical_flow_raw_topic.h"
#include "synapse_optical_flow_topic.h"
#include "synapse_rtcm3_topic.h"
#include "synapse_status_topic.h"

/*
 * The topic registry. One row per topic instance:
 *   X(name, payload type, shell printer, gate)
 * The gate is exactly the condition under which this node compiles a producer
 * or consumer of the topic, so declaration, broker registration, shell
 * tab-completion, and echo dispatch are generated only for the topics an
 * application actually carries. The streamed sensor topics gate on the very
 * devicetree alias their stream driver keys off, so the topic exists if and
 * only if the sensor does; a stale CONFIG_ZROS_SENSE_STREAM_* left on for
 * absent hardware cannot conjure a dead topic. The rest gate on the driver or
 * application that owns them. The mirror type headers above are always
 * included so the printers compile regardless of gating; the linker drops the
 * unused ones.
 */
#define SYNAPSE_TOPIC_TABLE(X)                                                                      \
	X(imu0, synapse_topic_InertialSample_t, snprint_imu,                                        \
	  DT_NODE_EXISTS(DT_ALIAS(imu_stream_0)))                                                   \
	X(imu1, synapse_topic_InertialSample_t, snprint_imu,                                        \
	  DT_NODE_EXISTS(DT_ALIAS(imu_stream_1)))                                                   \
	X(imu2, synapse_topic_InertialSample_t, snprint_imu,                                        \
	  DT_NODE_EXISTS(DT_ALIAS(imu_stream_2)))                                                   \
	X(mag0, synapse_topic_MagneticField_t, snprint_mag,                                         \
	  DT_NODE_EXISTS(DT_ALIAS(mag_stream_0)))                                                   \
	X(mag1, synapse_topic_MagneticField_t, snprint_mag,                                         \
	  DT_NODE_EXISTS(DT_ALIAS(mag_stream_1)))                                                   \
	X(nav_sat_fix, synapse_topic_GnssFix_t, snprint_gnss, CONFIG_ZROS_SENSE_GNSS)               \
	X(argus, synapse_topic_ArgusResults_t, snprint_argus,                                       \
	  DT_NODE_EXISTS(DT_ALIAS(argus_stream_0)))                                                 \
	X(optical_flow_raw, synapse_topic_PixartPaa3905_t, snprint_pixart_paa3905,                  \
	  DT_NODE_EXISTS(DT_ALIAS(optical_flow_stream_0)))                                          \
	X(status, synapse_topic_Status_t, snprint_status, CONFIG_ZROS_SENSE_STREAM_IMU)             \
	X(rtcm3, synapse_topic_Rtcm3_t, snprint_rtcm3, CONFIG_ZROS_SENSE_RTCM3_SUB)                 \
	X(optical_flow, synapse_topic_OpticalFlowData_t, snprint_optical_flow,                      \
	  CONFIG_SPINALI_VEHICLE_OPTICAL_FLOW)                                                       \
	X(optical_flow_vel, synapse_topic_OpticalFlowVelocityData_t, snprint_optical_flow_vel,      \
	  CONFIG_SPINALI_VEHICLE_OPTICAL_FLOW)

#define _SYNAPSE_TOPIC_DECLARE(name, type, printer, gate)                                          \
	IF_ENABLED(gate, (ZROS_TOPIC_DECLARE(name, type);))
SYNAPSE_TOPIC_TABLE(_SYNAPSE_TOPIC_DECLARE)
#undef _SYNAPSE_TOPIC_DECLARE

/********************************************************************
 * alias
 ********************************************************************/
#if DT_NODE_EXISTS(DT_ALIAS(imu_stream_0))
#define topic_imu topic_imu0
#endif
#if DT_NODE_EXISTS(DT_ALIAS(mag_stream_0))
#define topic_mag topic_mag0
#endif

#endif // SYNAPSE_TOPIC_LIST_H
// vi: ts=4 sw=4 et

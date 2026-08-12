/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SYNAPSE_TOPIC_LIST_H
#define SYNAPSE_TOPIC_LIST_H

#include <zros/zros_topic.h>

#include "synapse_argus_topic.h"
#include "synapse_gnss_topic.h"
#include "synapse_imu_topic.h"
#include "synapse_mag_topic.h"
#include "synapse_optical_flow_raw_topic.h"
#include "synapse_rtcm3_topic.h"
#include "synapse_status_topic.h"

#if defined(CONFIG_SPINALI_VEHICLE_OPTICAL_FLOW) || defined(CONFIG_SPINALI_SYNAPSE_ZENOH)
#include "synapse_optical_flow_topic.h"
#endif

/********************************************************************
 * topics
 ********************************************************************/
ZROS_TOPIC_DECLARE(argus, synapse_topic_ArgusResults_t);
ZROS_TOPIC_DECLARE(imu0, synapse_topic_InertialSample_t);
ZROS_TOPIC_DECLARE(imu1, synapse_topic_InertialSample_t);
ZROS_TOPIC_DECLARE(imu2, synapse_topic_InertialSample_t);
ZROS_TOPIC_DECLARE(mag0, synapse_topic_MagneticField_t);
ZROS_TOPIC_DECLARE(mag1, synapse_topic_MagneticField_t);
ZROS_TOPIC_DECLARE(nav_sat_fix, synapse_topic_GnssFix_t);
ZROS_TOPIC_DECLARE(optical_flow_raw, synapse_topic_PixartPaa3905_t);
ZROS_TOPIC_DECLARE(rtcm3, synapse_topic_Rtcm3_t);
ZROS_TOPIC_DECLARE(status, synapse_topic_Status_t);
#if defined(CONFIG_SPINALI_VEHICLE_OPTICAL_FLOW) || defined(CONFIG_SPINALI_SYNAPSE_ZENOH)
ZROS_TOPIC_DECLARE(optical_flow, synapse_topic_OpticalFlowData_t);
ZROS_TOPIC_DECLARE(optical_flow_vel, synapse_topic_OpticalFlowVelocityData_t);
#endif

/********************************************************************
 * alias
 ********************************************************************/
#define topic_imu topic_imu0
#define topic_mag topic_mag0

#endif // SYNAPSE_TOPIC_LIST_H_
// vi: ts=4 sw=4 et

/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/sys/slist.h>

#include <zros/private/zros_broker_struct.h>
#include <zros/private/zros_topic_struct.h>
#include <zros/zros_broker.h>

#include "synapse_topic_list.h"

/********************************************************************
 * topics
 ********************************************************************/
ZROS_TOPIC_DEFINE(argus, synapse_topic_ArgusResults_t);
ZROS_TOPIC_DEFINE(imu0, synapse_topic_InertialSample_t);
ZROS_TOPIC_DEFINE(imu1, synapse_topic_InertialSample_t);
ZROS_TOPIC_DEFINE(imu2, synapse_topic_InertialSample_t);
ZROS_TOPIC_DEFINE(mag0, synapse_topic_MagneticField_t);
ZROS_TOPIC_DEFINE(mag1, synapse_topic_MagneticField_t);
ZROS_TOPIC_DEFINE(nav_sat_fix, synapse_topic_GnssFix_t);
ZROS_TOPIC_DEFINE(optical_flow_raw, synapse_topic_PixartPaa3905_t);
ZROS_TOPIC_DEFINE(rtcm3, synapse_topic_Rtcm3_t);
ZROS_TOPIC_DEFINE(status, synapse_topic_Status_t);
#if defined(CONFIG_SPINALI_VEHICLE_OPTICAL_FLOW) || defined(CONFIG_SPINALI_SYNAPSE_ZENOH)
ZROS_TOPIC_DEFINE(optical_flow, synapse_topic_OpticalFlowData_t);
ZROS_TOPIC_DEFINE(optical_flow_vel, synapse_topic_OpticalFlowVelocityData_t);
#endif

static struct zros_topic *topic_list[] = {
	&topic_argus,
	&topic_imu0,
	&topic_imu1,
	&topic_imu2,
	&topic_mag0,
	&topic_mag1,
	&topic_nav_sat_fix,
	&topic_optical_flow_raw,
	&topic_rtcm3,
	&topic_status,
#if defined(CONFIG_SPINALI_VEHICLE_OPTICAL_FLOW) || defined(CONFIG_SPINALI_SYNAPSE_ZENOH)
	&topic_optical_flow,
	&topic_optical_flow_vel,
#endif
};

static int set_topic_list()
{
	for (size_t i = 0; i < ARRAY_SIZE(topic_list); i++) {
		zros_broker_add_topic(topic_list[i]);
	}
	return 0;
}

SYS_INIT(set_topic_list, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// vi: ts=4 sw=4 et

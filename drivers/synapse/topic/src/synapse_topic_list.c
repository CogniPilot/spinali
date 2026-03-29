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

//*******************************************************************
// helper functions
//*******************************************************************
void stamp_msg(synapse_pb_Timestamp *msg, int64_t ticks)
{
	int64_t sec = ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	int32_t nanosec = (ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 /
			  CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	msg->seconds = sec;
	msg->nanos = nanosec;
}

/********************************************************************
 * topics
 ********************************************************************/
ZROS_TOPIC_DEFINE(argus, synapse_pb_ArgusResults);
ZROS_TOPIC_DEFINE(imu0, synapse_pb_Imu);
ZROS_TOPIC_DEFINE(optical_flow_raw, synapse_pb_PixartPAA3905);
ZROS_TOPIC_DEFINE(status, synapse_pb_Status);
ZROS_TOPIC_DEFINE(vehicle_optical_flow, synapse_topic_VehicleOpticalFlowData_t);
ZROS_TOPIC_DEFINE(vehicle_optical_flow_vel, synapse_topic_VehicleOpticalFlowVelData_t);

static struct zros_topic *topic_list[] = {
	&topic_argus,
	&topic_imu0,
	&topic_optical_flow_raw,
	&topic_status,
	&topic_vehicle_optical_flow,
	&topic_vehicle_optical_flow_vel,
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

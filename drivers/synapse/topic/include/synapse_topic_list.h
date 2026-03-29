/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SYNAPSE_TOPIC_LIST_H
#define SYNAPSE_TOPIC_LIST_H

#include <zros/zros_topic.h>

#include <synapse_pb/argus_results.pb.h>
#include <synapse_pb/imu.pb.h>
#include <synapse_pb/pixart_paa3905.pb.h>
#include <synapse_pb/status.pb.h>

#include "synapse_optical_flow_reader.h"

/********************************************************************
 * helper
 ********************************************************************/
void stamp_msg(synapse_pb_Timestamp *hdr, int64_t ticks);

/********************************************************************
 * topics
 ********************************************************************/
ZROS_TOPIC_DECLARE(topic_argus, synapse_pb_ArgusResults);
ZROS_TOPIC_DECLARE(topic_imu0, synapse_pb_Imu);
ZROS_TOPIC_DECLARE(topic_optical_flow_raw, synapse_pb_PixartPAA3905);
ZROS_TOPIC_DECLARE(topic_status, synapse_pb_Status);
ZROS_TOPIC_DECLARE(topic_vehicle_optical_flow, synapse_topic_VehicleOpticalFlowData_t);
ZROS_TOPIC_DECLARE(topic_vehicle_optical_flow_vel, synapse_topic_VehicleOpticalFlowVelData_t);

/********************************************************************
 * alias
 ********************************************************************/
#define topic_imu topic_imu0

#endif // SYNAPSE_TOPIC_LIST_H_
// vi: ts=4 sw=4 et

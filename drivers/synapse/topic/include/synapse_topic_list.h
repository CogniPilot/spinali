/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SYNAPSE_TOPIC_LIST_H
#define SYNAPSE_TOPIC_LIST_H

#include <zros/zros_topic.h>

#include <synapse_pb/actuators.pb.h>
#include <synapse_pb/altimeter.pb.h>
#include <synapse_pb/argus_results.pb.h>
#include <synapse_pb/battery_state.pb.h>
#include <synapse_pb/bezier_trajectory.pb.h>
#include <synapse_pb/frame.pb.h>
#include <synapse_pb/imu.pb.h>
#include <synapse_pb/input.pb.h>
#include <synapse_pb/led_array.pb.h>
#include <synapse_pb/magnetic_field.pb.h>
#include <synapse_pb/nav_sat_fix.pb.h>
#include <synapse_pb/odometry.pb.h>
#include <synapse_pb/pwm.pb.h>
#include <synapse_pb/quaternion.pb.h>
#include <synapse_pb/rtcm3.pb.h>
#include <synapse_pb/safety.pb.h>
#include <synapse_pb/status.pb.h>
#include <synapse_pb/twist.pb.h>
#include <synapse_pb/vector3.pb.h>
#include <synapse_pb/wheel_odometry.pb.h>

#if defined(CONFIG_SPINALI_VEHICLE_OPTICAL_FLOW) || defined(CONFIG_SPINALI_SYNAPSE_ZENOH)
#include "synapse_optical_flow_topic.h"
#endif

/********************************************************************
 * helper
 ********************************************************************/
void stamp_msg(synapse_pb_Timestamp *hdr, int64_t ticks);
const char *mode_str(synapse_pb_Status_Mode mode);
const char *input_source_str(synapse_pb_Status_InputSource src);
const char *topic_source_str(synapse_pb_Status_TopicSource src);
const char *armed_str(synapse_pb_Status_Arming arming);
const char *safety_str(synapse_pb_Safety_Status safety);
const char *fuel_str(synapse_pb_Status_Fuel fuel);
const char *status_safety_str(synapse_pb_Status_Safety safety);
const char *link_status_str(synapse_pb_Status_LinkStatus status);

/********************************************************************
 * topics
 ********************************************************************/
ZROS_TOPIC_DECLARE(accel_ff, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(accel_sp, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(actuators, synapse_pb_Actuators);
ZROS_TOPIC_DECLARE(altimeter, synapse_pb_Altimeter);
ZROS_TOPIC_DECLARE(angular_velocity_ff, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(angular_velocity_sp, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(attitude_sp, synapse_pb_Quaternion);
ZROS_TOPIC_DECLARE(battery_state, synapse_pb_BatteryState);
ZROS_TOPIC_DECLARE(bezier_trajectory, synapse_pb_BezierTrajectory);
ZROS_TOPIC_DECLARE(bezier_trajectory_ethernet, synapse_pb_BezierTrajectory);
ZROS_TOPIC_DECLARE(clock_offset_ethernet, synapse_pb_Time);
ZROS_TOPIC_DECLARE(cmd_vel, synapse_pb_Twist);
ZROS_TOPIC_DECLARE(cmd_vel_ethernet, synapse_pb_Twist);
ZROS_TOPIC_DECLARE(argus, synapse_pb_ArgusResults);
ZROS_TOPIC_DECLARE(force_sp, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(imu0, synapse_pb_Imu);
ZROS_TOPIC_DECLARE(imu1, synapse_pb_Imu);
ZROS_TOPIC_DECLARE(imu2, synapse_pb_Imu);
ZROS_TOPIC_DECLARE(imu_q31_array, synapse_pb_ImuQ31Array);
ZROS_TOPIC_DECLARE(input, synapse_pb_Input);
ZROS_TOPIC_DECLARE(input_sbus, synapse_pb_Input);
ZROS_TOPIC_DECLARE(input_ethernet, synapse_pb_Input);
ZROS_TOPIC_DECLARE(led_array, synapse_pb_LEDArray);
ZROS_TOPIC_DECLARE(magnetic_field, synapse_pb_MagneticField);
ZROS_TOPIC_DECLARE(moment_ff, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(moment_sp, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(nav_sat_fix, synapse_pb_NavSatFix);
ZROS_TOPIC_DECLARE(odometry_estimator, synapse_pb_Odometry);
ZROS_TOPIC_DECLARE(odometry_ethernet, synapse_pb_Odometry);
ZROS_TOPIC_DECLARE(optical_flow_raw, synapse_pb_PixartPAA3905);
ZROS_TOPIC_DECLARE(orientation_sp, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(position_sp, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(pwm, synapse_pb_Pwm);
ZROS_TOPIC_DECLARE(rtcm3, synapse_pb_Rtcm3);
ZROS_TOPIC_DECLARE(safety, synapse_pb_Safety);
ZROS_TOPIC_DECLARE(status, synapse_pb_Status);
ZROS_TOPIC_DECLARE(velocity_sp, synapse_pb_Vector3);
#if defined(CONFIG_SPINALI_VEHICLE_OPTICAL_FLOW) || defined(CONFIG_SPINALI_SYNAPSE_ZENOH)
ZROS_TOPIC_DECLARE(optical_flow, synapse_topic_OpticalFlowData_t);
ZROS_TOPIC_DECLARE(optical_flow_vel, synapse_topic_OpticalFlowVelocityData_t);
#endif
ZROS_TOPIC_DECLARE(wheel_odometry, synapse_pb_WheelOdometry);

/********************************************************************
 * alias
 ********************************************************************/
#define topic_imu topic_imu0

#endif // SYNAPSE_TOPIC_LIST_H_
// vi: ts=4 sw=4 et

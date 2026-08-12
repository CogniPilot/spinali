/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SYNAPSE_SHELL_PRINT_H
#define SYNAPSE_SHELL_PRINT_H

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include <synapse_topic_list.h>

typedef int snprint_t(char *buf, size_t n, void *msg);

int snprint_argus(char *buf, size_t n, synapse_topic_ArgusResults_t *m);
int snprint_gnss(char *buf, size_t n, synapse_topic_GnssFix_t *m);
int snprint_imu(char *buf, size_t n, synapse_topic_InertialSample_t *m);
int snprint_mag(char *buf, size_t n, synapse_topic_MagneticField_t *m);
int snprint_pixart_paa3905(char *buf, size_t n, synapse_topic_PixartPaa3905_t *m);
int snprint_status(char *buf, size_t n, synapse_topic_Status_t *m);
int snprint_rtcm3(char *buf, size_t n, synapse_topic_Rtcm3_t *m);
int snprint_optical_flow(char *buf, size_t n, synapse_topic_OpticalFlowData_t *m);
int snprint_optical_flow_vel(char *buf, size_t n, synapse_topic_OpticalFlowVelocityData_t *m);

#endif // SYNAPSE_SHELL_PRINT_H
// vi: ts=4 sw=4 et

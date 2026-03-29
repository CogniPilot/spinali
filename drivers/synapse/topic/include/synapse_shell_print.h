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

int snprint_argus(char *buf, size_t n, synapse_pb_ArgusResults *m);
int snprint_imu(char *buf, size_t n, synapse_pb_Imu *m);
int snprint_pixart_paa3905(char *buf, size_t n, synapse_pb_PixartPAA3905 *m);
int snprint_status(char *buf, size_t n, synapse_pb_Status *m);

#endif // SYNAPSE_SHELL_PRINT_H
// vi: ts=4 sw=4 et

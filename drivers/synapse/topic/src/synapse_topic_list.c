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
#define _SYNAPSE_TOPIC_DEFINE(name, type, printer, gate)                                           \
	IF_ENABLED(gate, (ZROS_TOPIC_DEFINE(name, type);))
SYNAPSE_TOPIC_TABLE(_SYNAPSE_TOPIC_DEFINE)
#undef _SYNAPSE_TOPIC_DEFINE

static int set_topic_list(void)
{
#define _SYNAPSE_TOPIC_ADD(name, type, printer, gate)                                              \
	IF_ENABLED(gate, (zros_broker_add_topic(&topic_##name);))
	SYNAPSE_TOPIC_TABLE(_SYNAPSE_TOPIC_ADD)
#undef _SYNAPSE_TOPIC_ADD
	return 0;
}

SYS_INIT(set_topic_list, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// vi: ts=4 sw=4 et

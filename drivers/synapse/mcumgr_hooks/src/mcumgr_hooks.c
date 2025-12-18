/*
 * Copyright (c) 2025 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 *
 * MCUmgr custom hooks for hardware identification
 */

#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/hwinfo.h>

/* Suppress redundant strnlen declaration warning from zcbor_common.h */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wredundant-decls"
#include <zcbor_common.h>
#pragma GCC diagnostic pop

#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
#include <zephyr/mgmt/mcumgr/grp/os_mgmt/os_mgmt.h>

/* Custom format bitmask for hardware ID */
#define OS_MGMT_INFO_FORMAT_HWID OS_MGMT_INFO_FORMAT_USER_CUSTOM_START

/* Maximum hardware ID size (16 bytes = 128 bits is common) */
#define HWID_MAX_SIZE 16

static enum mgmt_cb_return os_info_custom_hook(uint32_t event, enum mgmt_cb_return prev_status,
					       int32_t *rc, uint16_t *group, bool *abort_more,
					       void *data, size_t data_size)
{
	ARG_UNUSED(prev_status);
	ARG_UNUSED(rc);
	ARG_UNUSED(group);
	ARG_UNUSED(abort_more);
	ARG_UNUSED(data_size);

	if (event == MGMT_EVT_OP_OS_MGMT_INFO_CHECK) {
		/* Check if we handle the 'h' format character for hardware ID */
		struct os_mgmt_info_check *check_data = (struct os_mgmt_info_check *)data;
		size_t i;

		for (i = 0; i < check_data->format->len; i++) {
			if (check_data->format->value[i] == 'h') {
				*check_data->format_bitmask |= OS_MGMT_INFO_FORMAT_HWID;
				++(*check_data->valid_formats);
			}
		}
	} else if (event == MGMT_EVT_OP_OS_MGMT_INFO_APPEND) {
		/* Append hardware ID if requested */
		struct os_mgmt_info_append *append_data = (struct os_mgmt_info_append *)data;

		if ((*append_data->format_bitmask & OS_MGMT_INFO_FORMAT_HWID) ||
		    append_data->all_format_specified) {
			uint8_t hwid[HWID_MAX_SIZE];
			char hwid_str[HWID_MAX_SIZE * 2 + 8]; /* "hwid:" + hex + null */
			ssize_t hwid_len;
			int ret;
			uint16_t remaining;

			hwid_len = hwinfo_get_device_id(hwid, sizeof(hwid));
			if (hwid_len > 0) {
				int offset = 0;

				/* Build "hwid:XXXX..." string */
				offset = snprintf(hwid_str, sizeof(hwid_str), "hwid:");
				for (int j = 0;
				     j < hwid_len && offset < (int)(sizeof(hwid_str) - 2); j++) {
					offset += snprintf(&hwid_str[offset],
							   sizeof(hwid_str) - offset, "%02x",
							   hwid[j]);
				}

				remaining = append_data->buffer_size - *append_data->output_length;

				/* Append to output with space separator if needed */
				if (*append_data->prior_output) {
					ret = snprintf(
						(char *)&append_data
							->output[*append_data->output_length],
						remaining, " %s", hwid_str);
				} else {
					ret = snprintf(
						(char *)&append_data
							->output[*append_data->output_length],
						remaining, "%s", hwid_str);
				}

				if (ret > 0 && ret < remaining) {
					*append_data->output_length += (uint16_t)ret;
					*append_data->prior_output = true;
				}
			}

			/* Clear the bit we handled */
			*append_data->format_bitmask &= ~OS_MGMT_INFO_FORMAT_HWID;
		}
	}

	return MGMT_CB_OK;
}

static struct mgmt_callback os_info_callback = {
	.callback = os_info_custom_hook,
	.event_id = MGMT_EVT_OP_OS_MGMT_INFO_CHECK | MGMT_EVT_OP_OS_MGMT_INFO_APPEND,
};

static int register_os_info_hook(void)
{
	mgmt_callback_register(&os_info_callback);
	return 0;
}

SYS_INIT(register_os_info_hook, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* vi: ts=4 sw=4 noet */

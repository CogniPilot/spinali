/*
 * Copyright (c) 2025 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 *
 * HCDF (Hardware Configuration Descriptive Format) MCUmgr group
 * Allows Dendrite to query device's HCDF URL and content hash
 */

#ifndef H_HCDF_MGMT_
#define H_HCDF_MGMT_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Group ID for HCDF management group.
 * Uses user group space starting at 64.
 */
#define MGMT_GROUP_ID_HCDF 100

/**
 * Command IDs for HCDF management group.
 */
#define HCDF_MGMT_ID_INFO 0

/**
 * Command result codes for HCDF management group.
 */
enum hcdf_mgmt_err_code_t {
	/** No error, this is implied if there is no ret value in the response */
	HCDF_MGMT_ERR_OK = 0,

	/** Unknown error occurred. */
	HCDF_MGMT_ERR_UNKNOWN,

	/** HCDF not configured on this device. */
	HCDF_MGMT_ERR_NOT_CONFIGURED,
};

#ifdef __cplusplus
}
#endif

#endif /* H_HCDF_MGMT_ */

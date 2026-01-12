/*
 * Copyright (c) 2025 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 *
 * HCDF MCUmgr group implementation
 * Returns the device's HCDF URL and content hash for Dendrite visualization
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/mgmt/mcumgr/mgmt/handlers.h>

/* Suppress redundant strnlen declaration warning from zcbor_common.h */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wredundant-decls"
#include <zcbor_common.h>
#pragma GCC diagnostic pop

#include <zcbor_encode.h>
#include <zephyr/logging/log.h>

#include <hcdf_mgmt.h>

LOG_MODULE_REGISTER(mcumgr_hcdf_grp, CONFIG_MCUMGR_GRP_HCDF_LOG_LEVEL);

/**
 * Handler for HCDF info read request.
 * Returns the HCDF URL and SHA256 hash configured via Kconfig.
 *
 * Response format (CBOR):
 * {
 *   "url": "https://hcdf.cognipilot.org/board/app.hcdf",
 *   "sha": "abc123..."
 * }
 */
static int hcdf_mgmt_info(struct smp_streamer *ctxt)
{
	zcbor_state_t *zse = ctxt->writer->zs;
	bool ok;

	LOG_DBG("HCDF info request");

#if defined(CONFIG_MCUMGR_GRP_HCDF_URL)
	const char *url = CONFIG_MCUMGR_GRP_HCDF_URL;
#else
	const char *url = "";
#endif

#if defined(CONFIG_MCUMGR_GRP_HCDF_SHA)
	const char *sha = CONFIG_MCUMGR_GRP_HCDF_SHA;
#else
	const char *sha = "";
#endif

	/* Check if HCDF is configured */
	if (strlen(url) == 0) {
		ok = smp_add_cmd_err(zse, MGMT_GROUP_ID_HCDF, HCDF_MGMT_ERR_NOT_CONFIGURED);
		return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
	}

	/* Encode response */
	ok = zcbor_tstr_put_lit(zse, "url") &&
	     zcbor_tstr_put_term(zse, url, CONFIG_MCUMGR_GRP_HCDF_URL_MAX_LEN) &&
	     zcbor_tstr_put_lit(zse, "sha") &&
	     zcbor_tstr_put_term(zse, sha, 65); /* SHA256 is 64 hex chars + null */

	return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

#ifdef CONFIG_MCUMGR_SMP_SUPPORT_ORIGINAL_PROTOCOL
static int hcdf_mgmt_translate_error_code(uint16_t err)
{
	int rc;

	switch (err) {
	case HCDF_MGMT_ERR_NOT_CONFIGURED:
		rc = MGMT_ERR_ENOENT;
		break;

	case HCDF_MGMT_ERR_UNKNOWN:
	default:
		rc = MGMT_ERR_EUNKNOWN;
	}

	return rc;
}
#endif

static const struct mgmt_handler hcdf_mgmt_handlers[] = {
	[HCDF_MGMT_ID_INFO] =
		{
			.mh_read = hcdf_mgmt_info,
			.mh_write = NULL,
		},
};

static struct mgmt_group hcdf_mgmt_group = {
	.mg_handlers = hcdf_mgmt_handlers,
	.mg_handlers_count = ARRAY_SIZE(hcdf_mgmt_handlers),
	.mg_group_id = MGMT_GROUP_ID_HCDF,
#ifdef CONFIG_MCUMGR_SMP_SUPPORT_ORIGINAL_PROTOCOL
	.mg_translate_error = hcdf_mgmt_translate_error_code,
#endif
};

static void hcdf_mgmt_register_group(void)
{
	mgmt_register_group(&hcdf_mgmt_group);
}

MCUMGR_HANDLER_DEFINE(hcdf_mgmt, hcdf_mgmt_register_group);

/* vi: ts=4 sw=4 noet */

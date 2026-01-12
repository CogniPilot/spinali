/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_SPINALI_CORE_COMMON_BOOT_BANNER)
#include <stdio.h>
#endif

#include <spinali/core/common.h>

LOG_MODULE_REGISTER(rtk_gnss_boot_banner, CONFIG_SPINALI_RTK_GNSS_LOG_LEVEL);

static int rtk_gnss_boot_banner_sys_init(void)
{
	LOG_INF("Spinali RTK GNSS %d.%d.%d", CONFIG_SPINALI_VERSION_MAJOR,
		CONFIG_SPINALI_VERSION_MINOR, CONFIG_SPINALI_VERSION_PATCH);
#if defined(CONFIG_SPINALI_CORE_COMMON_BOOT_BANNER)
	printf("%s", banner_spinali);
#endif
	return 0;
};

SYS_INIT(rtk_gnss_boot_banner_sys_init, APPLICATION, 0);

// vi: ts=4 sw=4 et

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

LOG_MODULE_REGISTER(optical_flow_boot_banner, CONFIG_SPINALI_OPTICAL_FLOW_LOG_LEVEL);

static int optical_flow_boot_banner_sys_init(void)
{
	LOG_INF("Spinali Optical Flow %d.%d.%d", CONFIG_SPINALI_VERSION_MAJOR,
		CONFIG_SPINALI_VERSION_MINOR, CONFIG_SPINALI_VERSION_PATCH);
#if defined(CONFIG_SPINALI_CORE_COMMON_BOOT_BANNER)
	printf("%s", banner_spinali);
#endif
	return 0;
};

SYS_INIT(optical_flow_boot_banner_sys_init, APPLICATION, 0);

// vi: ts=4 sw=4 et

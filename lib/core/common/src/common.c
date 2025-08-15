/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/reboot.h>

#include <stdio.h>
#include <stdlib.h>

#include <spinali/core/perf_duration.h>

LOG_MODULE_REGISTER(core_common, CONFIG_SPINALI_CORE_COMMON_LOG_LEVEL);

#if defined(CONFIG_REBOOT)
void do_reboot()
{
	sys_reboot(SYS_REBOOT_WARM);
};

SHELL_CMD_REGISTER(reboot, &do_reboot, "reboot autopilot", NULL);
#endif

const struct device *get_device(const struct device *const dev)
{
	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		LOG_ERR("no device found");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		LOG_ERR("device %s is not ready, check the driver initialization logs for errors",
			dev->name);
		return NULL;
	}
	return dev;
}

#if defined(CONFIG_SPINALI_CORE_COMMON_BOOT_BANNER)

const char *banner_spinali =
	"\n"
	"          ▓▓▓▓▓\n"
	"         ▓▓▓▓▓▓▓▓   ╔═══╗╔═══╗╔═══╗╔═╗ ╔╗╔══╗╔═══╗╔══╗╔╗   ╔═══╗╔════╗\n"
	"          ▓▓▓▓▓▓    ║╔═╗║║╔═╗║║╔═╗║║║║ ║║╚╣╠╝║╔═╗║╚╣╠╝║║   ║╔═╗║║╔╗╔╗║\n"
	"         ▒▒▒▒▓▓     ║║ ╚╝║║ ║║║║ ╚╝║║║ ║║ ║║ ║║ ║║ ║║ ║║   ║║ ║║╚╝║║╚╝\n"
	"        ▓▓▓▓▒▒▒     ║║   ║║ ║║║║   ║║╚╗║║ ║║ ║║ ║║ ║║ ║║   ║║ ║║  ║║\n"
	"       ▒▒▒▓▓▓▓▓▓    ║║   ║║ ║║║║   ║╔╗║║║ ║║ ║╚═╝║ ║║ ║║   ║║ ║║  ║║\n"
	"     ▓▓▓▓▒▒▒▒       ║║   ║║ ║║║║╔═╗║║║╚╝║ ║║ ║╔══╝ ║║ ║║   ║║ ║║  ║║\n"
	"    ▒▒▒▓▓▓▓▓▓▓▓     ║║   ║║ ║║║║╚╗║║║╚╗║║ ║║ ║║    ║║ ║║   ║║ ║║  ║║\n"
	"   ▓▓▓▒▒▒▒▒▒        ║║ ╔╗║║ ║║║║ ║║║║ ║║║ ║║ ║║    ║║ ║║ ╔╗║║ ║║  ║║\n"
	"   ▓▓▓▓▓▓▓▓▓▓       ║╚═╝║║╚═╝║║╚═╝║║║ ║║║╔╣╠╗║║   ╔╣╠╗║╚═╝║║╚═╝║ ╔╝╚╗\n"
	"  ▒▒▒▒▒▒▒▒          ╚═══╝╚═══╝╚═══╝╚╝ ╚═╝╚══╝╚╝   ╚══╝╚═══╝╚═══╝ ╚══╝\n"
	" ▓▓▓▓▓▓▓▓▓\n"
	" ▓▓▓▓▓▓▓▓▓▓▓\n"
	" ▒▒▒▒▒▒▒▒\n"
	"▓▓▓▓▓▓▓▓▓▓▓\n"
	"▓▓▓▓▓▓▓▓▓▓▓▓             ┏━━━┓ ┏━━━┓ ┏━━┓ ┏┓ ┏┓  ┏━┓  ┏┓    ┏━━┓\n"
	" ▒▒▒▒▒▒▒▒▒               ┃┏━┓┃ ┃┏━┓┃ ┗┫┣┛ ┃┃ ┃┃ ┏┛ ┗┓ ┃┃    ┗┫┣┛\n"
	" ▒▓▓▓▓▓▓▓▓▓▓             ┃┃ ┗┛ ┃┃ ┃┃  ┃┃  ┃┗┓┃┃ ┃┏━┓┃ ┃┃     ┃┃\n"
	"▓▓▓▓▓▓▓▓▓▓▓              ┃┗┓   ┃┃ ┃┃  ┃┃  ┃ ┃┃┃ ┃┃ ┃┃ ┃┃     ┃┃\n"
	"▓▓▓▓▓▓▓▒▒▒               ┗┓┗┓  ┃┗━┛┃  ┃┃  ┃┃┗┫┃ ┃┃ ┃┃ ┃┃     ┃┃\n"
	"  ▒▒▒▒▒▒▓▓▓▓▓             ┗┓┗┓ ┃┏━━┛  ┃┃  ┃┣┓┃┃ ┃┗━┛┃ ┃┃     ┃┃\n"
	"  ▒▓▓▓▓▓▓▓▓▓               ┗┓┃ ┃┃     ┃┃  ┃┃┃ ┃ ┃┏━┓┃ ┃┃     ┃┃\n"
	" ▓▓▓▓▓▓▓▓▓▒▒             ┏┓ ┃┃ ┃┃     ┃┃  ┃┃┗┓┃ ┃┃ ┃┃ ┃┃ ┏┓  ┃┃\n"
	"  ▓▓▓▓▓▒▒▒▒▓▓▓▓▓         ┃┗━┛┃ ┃┃    ┏┫┣┓ ┃┃ ┃┃ ┃┃ ┃┃ ┃┗━┛┃ ┏┫┣┓\n"
	"    ▒▒▒▒▓▓▓▓▓▓           ┗━━━┛ ┗┛    ┗━━┛ ┗┛ ┗┛ ┗┛ ┗┛ ┗━━━┛ ┗━━┛\n"
	"    ▓▓▓▓▓▓▓▓▓▒▒ ▓▓▓\n"
	"     ▓▓▓▓▓▓▒▒▒▓▓▓▓▓▓\n"
	"       ▓▓▒▒▒▓▓▓▓▓▓▓▓\n"
	"         ▒▓▓▓▓▓▓ ▓▓\n";

#endif

// vi: ts=4 sw=4 et

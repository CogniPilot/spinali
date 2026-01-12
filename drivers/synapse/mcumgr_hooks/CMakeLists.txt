# Copyright (c) 2025 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SPINALI_SYNAPSE_MCUMGR_HOOKS src/mcumgr_hooks.c)
zephyr_library_sources_ifdef(CONFIG_MCUMGR_GRP_HCDF src/hcdf_mgmt.c)
zephyr_include_directories_ifdef(CONFIG_MCUMGR_GRP_HCDF include)

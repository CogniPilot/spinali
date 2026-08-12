# Copyright CogniPilot Foundation 2026
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(spinali_time_reference)
zephyr_library_sources(src/main.c)

# The served clock quality (grandmaster clockClass, announced UTC offset)
# lives in the gPTP stack's internal data sets, reached the same way the
# gPTP shell reaches them.
zephyr_library_include_directories(${ZEPHYR_BASE}/subsys/net/l2)

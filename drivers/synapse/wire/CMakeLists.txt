# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(spinali_synapse_wire)
zephyr_library_sources(src/main.c)
target_include_directories(spinali_synapse_wire PRIVATE ${ZEPHYR_BASE}/subsys/net/ip)
target_link_libraries(spinali_synapse_wire PUBLIC synapse_fbs::wire)

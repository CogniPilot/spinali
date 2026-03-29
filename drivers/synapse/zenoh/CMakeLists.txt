# Copyright CogniPilot Foundation 2025
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(spinali_synapse_zenoh)
zephyr_library_sources(
  src/main.c
  src/fb_pack.c
)
zephyr_include_directories(${SYNAPSE_FLATBUFFER_GENERATED_DIR})

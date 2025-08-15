# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(spinali_sense_accel)


zephyr_include_directories()

zephyr_library_sources(
  main.c
  )

add_dependencies(spinali_sense_accel synapse_pb)
add_dependencies(spinali_sense_accel spinali_core_common)

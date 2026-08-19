# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(spinali_core_common)

zephyr_include_directories(src
  ${CMAKE_BINARY_DIR}
  )

zephyr_library_include_directories(src
  ${CMAKE_BINARY_DIR}
  )

zephyr_library_sources(
  src/common.c
  )

add_dependencies(app spinali_core_common)

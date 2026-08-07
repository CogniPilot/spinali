# Copyright (c) 2026, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(spinali_timing)

set(flags
  -std=c11
  -Wall
  -Wextra
  -Werror
  -Wstrict-prototypes
  -Waggregate-return
  -Wbad-function-cast
  -Wcast-align
  -Wcast-qual
  -Wfloat-equal
  -Wformat-security
  -Wlogical-op
  -Wmissing-declarations
  -Wmissing-include-dirs
  -Wmissing-prototypes
  -Wnested-externs
  -Wpointer-arith
  -Wredundant-decls
  -Wsequence-point
  -Wshadow
  -Wstrict-prototypes
  -Wswitch
  -Wundef
  -Wunreachable-code
  -Wunused-but-set-parameter
  -Wwrite-strings
  )
string(JOIN " " flags ${flags})

set(SOURCE_FILES
  src/pps_capture.c
  src/pps_servo.c
  )

if(CONFIG_SPINALI_TIMING_TM_AUDIT)
  list(APPEND SOURCE_FILES src/tm_audit.c)
endif()

set_source_files_properties(
  ${SOURCE_FILES}
  PROPERTIES COMPILE_FLAGS
  "${flags}"
  )

zephyr_library_include_directories(src)

# pps_servo.h is the interface of this library: applications that stamp
# traffic from the disciplined PHC ask it whether the clock is on the GNSS
# timescale yet, so the directory is exported rather than kept private.
zephyr_include_directories(src)

zephyr_library_sources(${SOURCE_FILES})

# The strict warning set above is aimed at this library only. Zephyr and
# generated headers are pulled in as system includes so that diagnostics
# raised inside them are not attributed to these sources.
target_include_directories(spinali_timing SYSTEM BEFORE PRIVATE
  ${ZEPHYR_BASE}/include
  ${CMAKE_BINARY_DIR}
  )

add_dependencies(app spinali_timing)

# vi: ts=2 sw=2 et

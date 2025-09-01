#-------------------------------------------------------------------------------
# Zephyr Spinali Application
#
# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

#Specify a minimum Cmake version
cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

#Give the project a name
project(
  IMU_ICM45686_test
  VERSION 1.0
  DESCRIPTION "ICM45686 test code"
  LANGUAGES C
)

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
  src/main.c
  )

set_source_files_properties(
  ${SOURCE_FILES}
  PROPERTIES COMPILE_FLAGS
  "${flags}"
  )

target_sources(app PRIVATE ${SOURCE_FILES})

target_include_directories(app SYSTEM BEFORE PRIVATE ${ZEPHYR_BASE}/include ${CMAKE_BINARY_DIR})

# vi: ts=2 sw=2 et

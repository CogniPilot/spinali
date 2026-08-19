#-------------------------------------------------------------------------------
# Zephyr Spinali Application
#
# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

set(SPINALI_SYNAPSE_FBS_ROOT "" CACHE PATH "Generated Synapse C package")
if(NOT SPINALI_SYNAPSE_FBS_ROOT AND DEFINED ENV{SPINALI_SYNAPSE_FBS_ROOT})
  set(SPINALI_SYNAPSE_FBS_ROOT "$ENV{SPINALI_SYNAPSE_FBS_ROOT}" CACHE PATH
    "Generated Synapse C package" FORCE)
endif()
if(SPINALI_SYNAPSE_FBS_ROOT)
  list(APPEND ZEPHYR_EXTRA_MODULES ${SPINALI_SYNAPSE_FBS_ROOT})
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(optical_flow LANGUAGES C)

# Validate HCDF SHA at build time
if(DEFINED ZEPHYR_HCDF_MODELS_MODULE_DIR)
  include(${ZEPHYR_HCDF_MODELS_MODULE_DIR}/cmake/hcdf.cmake)
  hcdf_validate_sha(BOARD mr_mcxn_t1 DEVICE optical-flow)
endif()

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
  src/boot_banner.c
  )

set_source_files_properties(
  ${SOURCE_FILES}
  PROPERTIES COMPILE_FLAGS
  "${flags}"
  )

target_sources(app PRIVATE ${SOURCE_FILES})

if(CONFIG_SPINALI_SYNAPSE_WIRE)
  if(NOT TARGET synapse_fbs::wire)
    message(FATAL_ERROR
      "SPINALI_SYNAPSE_FBS_ROOT must provide synapse_fbs::wire"
    )
  endif()
  target_link_libraries(app PRIVATE synapse_fbs::wire)
endif()

if(CONFIG_SPINALI_SYNAPSE_ROS_CDR)
  if(NOT TARGET synapse_fbs::cdr)
    message(FATAL_ERROR
      "SPINALI_SYNAPSE_FBS_ROOT must provide synapse_fbs::cdr"
    )
  endif()
  target_link_libraries(app PRIVATE synapse_fbs::cdr)
endif()

target_include_directories(app SYSTEM BEFORE PRIVATE ${ZEPHYR_BASE}/include ${CMAKE_BINARY_DIR})

# vi: ts=2 sw=2 et

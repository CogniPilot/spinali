#-------------------------------------------------------------------------------
# Zephyr Spinali Application
#
# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(optical_flow LANGUAGES C)

# ---- FlatBuffer code generation ----
get_filename_component(SPINALI_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../.. ABSOLUTE)
get_filename_component(WS_ROOT ${SPINALI_ROOT}/.. ABSOLUTE)
set(SYNAPSE_FLATBUFFER_SCHEMA_DIR
  ${WS_ROOT}/modules/lib/synapse_msgs_fbs/fbs/synapse
)
set(SYNAPSE_FLATBUFFER_GENERATED_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/generated/flatbuffers
)

find_program(FLATCC_EXECUTABLE NAMES flatcc)

if(FLATCC_EXECUTABLE)
  get_filename_component(FLATCC_REALPATH ${FLATCC_EXECUTABLE} REALPATH)
  get_filename_component(FLATCC_BIN_DIR ${FLATCC_REALPATH} DIRECTORY)
  get_filename_component(FLATCC_PREFIX ${FLATCC_BIN_DIR} DIRECTORY)
endif()

find_path(FLATCC_INCLUDE_DIR
  NAMES flatcc/flatcc_builder.h
  HINTS ${FLATCC_PREFIX}/include
        $ENV{HOME}/git/flatcc/include
)

if(FLATCC_EXECUTABLE)
  file(MAKE_DIRECTORY ${SYNAPSE_FLATBUFFER_GENERATED_DIR})

  set(SYNAPSE_FBS_FILES
    ${SYNAPSE_FLATBUFFER_SCHEMA_DIR}/synapse_topics.fbs
    ${SYNAPSE_FLATBUFFER_SCHEMA_DIR}/synapse_optical_flow.fbs
  )

  set(SYNAPSE_FLATBUFFER_STAMP
    ${SYNAPSE_FLATBUFFER_GENERATED_DIR}/.flatcc.stamp
  )

  add_custom_command(
    OUTPUT ${SYNAPSE_FLATBUFFER_STAMP}
    COMMAND ${FLATCC_EXECUTABLE}
            -a
            -I${SYNAPSE_FLATBUFFER_SCHEMA_DIR}
            -o${SYNAPSE_FLATBUFFER_GENERATED_DIR}
            ${SYNAPSE_FLATBUFFER_SCHEMA_DIR}/synapse_optical_flow.fbs
    COMMAND ${CMAKE_COMMAND} -E touch ${SYNAPSE_FLATBUFFER_STAMP}
    DEPENDS ${SYNAPSE_FBS_FILES}
    COMMENT "Generating FlatBuffer headers for synapse optical flow"
    VERBATIM
  )

  add_custom_target(synapse_flatbuffer_codegen DEPENDS ${SYNAPSE_FLATBUFFER_STAMP})
  add_dependencies(app synapse_flatbuffer_codegen)
else()
  message(WARNING "flatcc not found - FlatBuffer headers will not be regenerated")
endif()

# Make generated headers available to all libraries
set(SYNAPSE_FLATBUFFER_GENERATED_DIR ${SYNAPSE_FLATBUFFER_GENERATED_DIR} CACHE INTERNAL "")
zephyr_include_directories(${SYNAPSE_FLATBUFFER_GENERATED_DIR})
if(FLATCC_INCLUDE_DIR)
  zephyr_include_directories(${FLATCC_INCLUDE_DIR})
endif()

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

target_include_directories(app SYSTEM BEFORE PRIVATE ${ZEPHYR_BASE}/include ${CMAKE_BINARY_DIR})

# vi: ts=2 sw=2 et

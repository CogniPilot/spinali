# Copyright (c) 2025, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

set(CYCLONEDDS_PATH ${CMAKE_SOURCE_DIR}/../../../modules/lib/cyclonedds)
set(SYNAPSE_MSGS_PATH ${CMAKE_SOURCE_DIR}/../../../modules/lib/synapse_msgs_idl)

# CycloneDDS CDR serializer
include(${CYCLONEDDS_PATH}/src/core/cdr/CMakeLists.txt)
target_compile_options(cdr PRIVATE ${ZEPHYR_CFLAGS})
target_link_libraries(cdr PUBLIC zephyr_interface)

target_compile_options(cdr PUBLIC
            -Wno-cast-align
            -Wno-double-promotion
            $<$<COMPILE_LANGUAGE:C>:-Wno-implicit-function-declaration -Wno-nested-externs>
            -DNDEBUG)
target_sources(cdr PRIVATE dds_serializer.c)
target_include_directories(cdr PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# CDR message generation

# CycloneDDS-tools doesn't ship with the cdrstream-desc feature thus we've to compile idlc from source
MESSAGE(STATUS "Configuring idlc :" ${CMAKE_CURRENT_BINARY_DIR}/idlc)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc)
execute_process(COMMAND ${CMAKE_COMMAND} ${CYCLONEDDS_PATH}
    -DCMAKE_C_COMPILER=/usr/bin/gcc
    -DBUILD_EXAMPLES=OFF
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc
    RESULT_VARIABLE CMD_ERROR
    OUTPUT_FILE CMD_OUTPUT )
MESSAGE(STATUS "Building idlc :" ${CMAKE_CURRENT_BINARY_DIR}/idlc)
execute_process(COMMAND ${CMAKE_COMMAND} --build . --target idlc
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc
    RESULT_VARIABLE CMD_ERROR
    OUTPUT_FILE CMD_OUTPUT )
list(APPEND CMAKE_PROGRAM_PATH "${CMAKE_CURRENT_BINARY_DIR}/idlc/bin")

# Generate code for msgs
file(GLOB_RECURSE idl_list "${SYNAPSE_MSGS_PATH}/*.idl")
set(IDLC_GENERATE_DIR ${CMAKE_CURRENT_BINARY_DIR}/idlc_generate)

# Generate C definitions from IDL
include("${CYCLONEDDS_PATH}/cmake/Modules/Generate.cmake")
idlc_generate(TARGET msgs_cdrstream
                FEATURES "cdrstream-desc"
                FILES ${idl_list}
                INCLUDES ${SYNAPSE_MSGS_PATH}
                BASE_DIR ${SYNAPSE_MSGS_PATH}
                WARNINGS no-implicit-extensibility)
                
target_include_directories(cdr INTERFACE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(cdr PRIVATE msgs_cdrstream)
add_dependencies(cdr msgs_cdrstream)

# Custom command to generate combined files
add_custom_command(
    OUTPUT  ${CMAKE_CURRENT_BINARY_DIR}/synapse_msgs.c ${CMAKE_CURRENT_BINARY_DIR}/synapse_msgs.h
    COMMAND ${CMAKE_COMMAND} -E echo "Generating synapse_msgs_idl core..."
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/synapse_msgs.py ${SYNAPSE_MSGS_PATH} ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/synapse_msgs.py ${SYNAPSE_MSGS_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/synapse_msgs.c.em ${CMAKE_CURRENT_SOURCE_DIR}/synapse_msgs.h.em
    COMMENT "Running Empy code generation for combined output"
)

# Add a library that uses the generated source
add_library(synapse_msgs_lib
    ${CMAKE_CURRENT_BINARY_DIR}/synapse_msgs.c
)

target_include_directories(synapse_msgs_lib PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(synapse_msgs_lib PRIVATE cdr )
add_dependencies(synapse_msgs_lib cdr)

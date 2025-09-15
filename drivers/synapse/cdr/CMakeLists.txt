# Copyright (c) 2025, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

set(CYCLONEDDS_PATH ${CMAKE_SOURCE_DIR}/../../../modules/lib/cyclonedds)
set(SYNAPSE_MSGS_PATH ${CMAKE_SOURCE_DIR}/../../../modules/lib/synapse_msgs)

# CycloneDDS CDR serializer
include(${CYCLONEDDS_PATH}/src/core/cdr/CMakeLists.txt)

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
file(GLOB idl_list "${SYNAPSE_MSGS_PATH}/rosidl/*.idl")
set(IDLC_GENERATE_DIR ${CMAKE_CURRENT_BINARY_DIR}/idlc_generate)

# Copy .idl files
set(cdr_idl)
foreach(idl_file ${idl_list})
	get_filename_component(msg ${idl_file} NAME_WE)
  configure_file(${idl_file} ${IDLC_GENERATE_DIR}/synapse_msgs/msg/${msg}.idl COPYONLY)
	list(APPEND cdr_idl ${IDLC_GENERATE_DIR}/synapse_msgs/msg/${msg}.idl)
endforeach()

# Generate C definitions from IDL
include("${CYCLONEDDS_PATH}/cmake/Modules/Generate.cmake")
idlc_generate(TARGET msgs_cdrstream
                FEATURES "cdrstream-desc"
                FILES ${cdr_idl}
                INCLUDES ${IDLC_GENERATE_DIR}
                BASE_DIR ${IDLC_GENERATE_DIR}
                WARNINGS no-implicit-extensibility)
                
add_dependencies(cdr msgs_cdrstream)
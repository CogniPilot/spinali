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
  bmm350_test
  VERSION 1.0
  DESCRIPTION "ICM45686 test code"
  LANGUAGES C
)

set(SOURCE_FILES
  src/main.c
  )

target_sources(app PRIVATE ${SOURCE_FILES})

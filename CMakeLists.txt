# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0
#
# This CMake file is picked by the Zephyr build system because it is defined
# as the module CMake entry point (see zephyr/module.yml).

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

zephyr_include_directories(include)

find_program(POETRY poetry)
if (NOT POETRY)
	message(FATAL_ERROR "install poetry via: apt install python3-poetry or pipx install poetry")
endif()
set(SPINALI_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(CYECCA_PYTHON  ${SPINALI_ROOT_DIR}/scripts/cyecca_python)

execute_process(
	COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/cyecca_install
	COMMAND_ECHO STDOUT
	)

add_subdirectory(lib)
add_subdirectory(drivers)

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/../install)

install(PROGRAMS ${CMAKE_BINARY_DIR}/zephyr/zephyr.bin
	DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
	RENAME spinali_${CONFIG_SPINALI_APP_NAME}_${BOARD} OPTIONAL)

install(PROGRAMS ${CMAKE_BINARY_DIR}/zephyr/zephyr.exe
	DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
	RENAME spinali_${CONFIG_SPINALI_APP_NAME}_${BOARD} OPTIONAL)

install(FILES ${CMAKE_SOURCE_DIR}/../../scripts/setup.sh
	DESTINATION ${CMAKE_INSTALL_PREFIX})

set(CPACK_PACKAGE_NAME spinali_${CONFIG_SPINALI_APP_NAME})
set(CPACK_GENERATOR "DEB;TGZ")
set(CPACK_DEBIAN_PACKAGE_DEPENDS)
set(CPACK_SYSTEM_NAME "${BOARD}")
set(CPACK_PACKAGE_VERSION_MAJOR "${CONFIG_SPINALI_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${CONFIG_SPINALI_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${CONFIG_SPINALI_VERSION_PATCH}")
set(CPACK_PACKAGE_CONTACT cogni@cognipilot.com)

include(CPack)

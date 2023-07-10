#
# Copyright (c) 2020 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(protec3)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/api-gateway.c)
target_include_directories(app PRIVATE ${ZEPHYR_BASE}/subsys/fs/nvs)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/src/tls_config)

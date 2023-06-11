# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(protect3_test)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/buzzer.c)

zephyr_include_directories(src)
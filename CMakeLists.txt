cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

target_include_directories(app PRIVATE /src/)
target_include_directories(app PRIVATE ./)
include(common.cmake)
project(bm_timer)
GET_DEVICE_CONFIG_FILES(${BOARD} ./boards)
target_sources(app PRIVATE 
src/main.c
src/spi.c)

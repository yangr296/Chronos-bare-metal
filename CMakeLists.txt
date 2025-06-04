# Merged CMakeLists.txt for chronos-bare-metal + peripheral_uart
# Copyright (c) 2018-2025 Nordic Semiconductor
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

cmake_minimum_required(VERSION 3.20.0)

# Find Zephyr - this must come before project() declaration
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Include the common cmake utilities from chronos-bare-metal
# This provides the GET_DEVICE_CONFIG_FILES macro
include(common.cmake)

# Define the project name - using a combined name to reflect both functionalities
project(chronos_ble_uart)

# Set up include directories for the chronos-bare-metal code structure
target_include_directories(app PRIVATE src/)
target_include_directories(app PRIVATE ./)

# Get board-specific configurations (from chronos-bare-metal approach)
# This will look for board-specific overlay and conf files in ./boards/
GET_DEVICE_CONFIG_FILES(${BOARD} ./boards)

# Add source files from both projects
target_sources(app PRIVATE
    src/main.c
    # Add any additional source files here as you modularize:
    # src/timing_control.c
    # src/ble_services.c
    # src/spi_dac.c
)
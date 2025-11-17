cmake_minimum_required(VERSION 3.13)

# Set board type to Pico W (must be before pico_sdk_import)
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(ptp_gps_project C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Common library (shared code)
add_library(common INTERFACE)
target_sources(common INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/common/discipline.c
    ${CMAKE_CURRENT_LIST_DIR}/common/ptp_protocol.c
)
target_include_directories(common INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/common
)

# Add subdirectories for each device
add_subdirectory(grandmaster)
add_subdirectory(wifi_test)  # WiFi testing/debugging
add_subdirectory(slave)      # Phase 3
# add_subdirectory(monitor)  # Phase 4

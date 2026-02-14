cmake_minimum_required(VERSION 3.13)

# =============================================================================
# Network Backend Selection
# =============================================================================
# USE_ETHERNET=OFF (default): Build for Pico W with WiFi (CYW43)
# USE_ETHERNET=ON: Build for W55RP20 or W5500-based boards with Ethernet
option(USE_ETHERNET "Use W5500 Ethernet instead of WiFi" OFF)

# Set board type based on network backend (must be before pico_sdk_import)
if(USE_ETHERNET)
    set(PICO_BOARD pico CACHE STRING "Board type")
    message(STATUS "Building for Ethernet (W5500) - PICO_BOARD=pico")
else()
    set(PICO_BOARD pico_w CACHE STRING "Board type")
    message(STATUS "Building for WiFi (CYW43) - PICO_BOARD=pico_w")
endif()

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(ptp_gps_project C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# =============================================================================
# Common library (shared code)
# =============================================================================
add_library(common INTERFACE)
target_sources(common INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/common/discipline.c
    ${CMAKE_CURRENT_LIST_DIR}/common/ptp_protocol.c
)
target_include_directories(common INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/common
)

# =============================================================================
# Network library (WiFi or Ethernet backend)
# =============================================================================
if(USE_ETHERNET)
    # ---------------------------------------------------------------------------
    # W5500 Ethernet Backend - ARCHIVED (now using MACRAW mode w5500_simple.c)
    # ---------------------------------------------------------------------------
    # WIZnet library ARCHIVED - No longer used
    # Now using MACRAW mode with w5500_simple.c and eth_simple.c
    # (lightweight, no lwIP, direct packet manipulation)

    message(STATUS "Network backend: W5500 Ethernet (MACRAW mode)")

else()
    # ---------------------------------------------------------------------------
    # WiFi Backend using CYW43
    # ---------------------------------------------------------------------------
    add_library(network INTERFACE)

    target_sources(network INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/common/network_wifi.c
    )

    target_include_directories(network INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/common
    )

    target_link_libraries(network INTERFACE
        pico_cyw43_arch_lwip_poll
    )

    message(STATUS "Network backend: WiFi (CYW43)")
endif()

# =============================================================================
# Add subdirectories for each device
# =============================================================================
add_subdirectory(grandmaster)
add_subdirectory(slave)
# add_subdirectory(logger)  # ARCHIVED - superseded by measurement device
add_subdirectory(measurement)
add_subdirectory(ltsp)
# add_subdirectory(final_phase_meter)  # ARCHIVED - superseded by measurement device
# add_subdirectory(calibration_test)  # TEST - uncomment if needed for calibration
#
# # WiFi test only makes sense for WiFi builds
# if(NOT USE_ETHERNET)
#     add_subdirectory(wifi_test)
# endif()
#
# # Minimal USB test
# add_subdirectory(usb_test)

# add_subdirectory(disciplined_oscillator)  # ARCHIVED
# add_subdirectory(monitor)  # EMPTY

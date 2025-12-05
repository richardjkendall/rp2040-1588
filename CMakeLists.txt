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
    # W5500 Ethernet Backend using WIZnet-PICO-LWIP-C
    # ---------------------------------------------------------------------------
    set(WIZNET_DIR ${CMAKE_CURRENT_LIST_DIR}/libraries/WIZnet-PICO-LWIP-C)
    set(WIZNET_PORT_DIR ${WIZNET_DIR}/port)
    set(WIZNET_IOLIB_DIR ${WIZNET_DIR}/libraries/ioLibrary_Driver)

    # WIZnet ioLibrary driver files
    add_library(WIZNET_IOLIB STATIC
        ${WIZNET_PORT_DIR}/ioLibrary_Driver/src/w5x00_spi.c
        ${WIZNET_PORT_DIR}/ioLibrary_Driver/src/w5x00_gpio_irq.c
        ${WIZNET_PORT_DIR}/ioLibrary_Driver/src/wiznet_spi_pio.c
        ${WIZNET_IOLIB_DIR}/Ethernet/wizchip_conf.c
        ${WIZNET_IOLIB_DIR}/Ethernet/W5500/w5500.c
        ${WIZNET_IOLIB_DIR}/Ethernet/socket.c
    )

    # Generate PIO header for SPI
    pico_generate_pio_header(WIZNET_IOLIB
        ${WIZNET_PORT_DIR}/ioLibrary_Driver/src/wiznet_spi_pio.pio
    )

    target_include_directories(WIZNET_IOLIB PUBLIC
        ${WIZNET_IOLIB_DIR}/Ethernet
        ${WIZNET_IOLIB_DIR}/Ethernet/W5500
        ${WIZNET_PORT_DIR}/ioLibrary_Driver/inc
        ${WIZNET_PORT_DIR}
    )

    target_compile_definitions(WIZNET_IOLIB PUBLIC
        _WIZCHIP_=W5500
    )

    target_link_libraries(WIZNET_IOLIB PUBLIC
        pico_stdlib
        hardware_spi
        hardware_dma
        hardware_pio
        hardware_clocks
    )

    # WIZnet lwIP integration files
    add_library(WIZNET_LWIP STATIC
        ${WIZNET_PORT_DIR}/lwip/w5x00_lwip.c
    )

    target_include_directories(WIZNET_LWIP PUBLIC
        ${WIZNET_IOLIB_DIR}/Ethernet
        ${WIZNET_PORT_DIR}
        ${WIZNET_PORT_DIR}/lwip
    )

    target_link_libraries(WIZNET_LWIP PUBLIC
        WIZNET_IOLIB
        pico_stdlib
        pico_lwip
        pico_lwip_nosys
    )

    # Network abstraction library
    add_library(network INTERFACE)

    target_sources(network INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/common/network_w5500.c
    )

    target_include_directories(network INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/common
        ${WIZNET_IOLIB_DIR}/Ethernet
        ${WIZNET_IOLIB_DIR}/Ethernet/W5500
        ${WIZNET_PORT_DIR}/ioLibrary_Driver/inc
        ${WIZNET_PORT_DIR}
        ${WIZNET_PORT_DIR}/lwip
    )

    target_compile_definitions(network INTERFACE
        USE_ETHERNET=1
        _WIZCHIP_=W5500
    )

    target_link_libraries(network INTERFACE
        WIZNET_LWIP
        pico_lwip
        pico_lwip_nosys
    )

    message(STATUS "Network backend: W5500 Ethernet (W5500-EVB-Pico)")

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
add_subdirectory(logger)
add_subdirectory(measurement)
add_subdirectory(final_phase_meter)

# WiFi test only makes sense for WiFi builds
if(NOT USE_ETHERNET)
    add_subdirectory(wifi_test)
endif()

# Minimal USB test
add_subdirectory(usb_test)

# add_subdirectory(disciplined_oscillator)
# add_subdirectory(monitor)

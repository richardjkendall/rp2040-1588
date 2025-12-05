add_executable(grandmaster
    main.c
    gps.c
    discipline_v2.c
    ptp_grandmaster.c
)

# Generate PIO headers from .pio files
# NOTE: gps_discipline.pio contains both counter and PPS capture programs
pico_generate_pio_header(grandmaster ${CMAKE_CURRENT_LIST_DIR}/pio/gps_discipline.pio)

target_include_directories(grandmaster PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}
)

target_compile_definitions(grandmaster PRIVATE
    BUILD_GRANDMASTER
    PICO_CORE1_STACK_SIZE=0x1000
)

target_link_libraries(grandmaster
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_dma
    hardware_uart
    hardware_timer
    hardware_irq
    hardware_clocks
    common
    network
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(grandmaster 1)
pico_enable_stdio_uart(grandmaster 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(grandmaster)

# ============================================================================
# MINIMAL TEST VERSION - No printf, network, or NMEA parsing
# V3: Uses PIO IRQ flags instead of DMA (eliminates bus contention)
# ============================================================================
add_executable(grandmaster_minimal_test
    main_minimal_test.c
    gps.c
    discipline_v3.c
)

target_include_directories(grandmaster_minimal_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}
)

target_compile_definitions(grandmaster_minimal_test PRIVATE
    BUILD_GRANDMASTER
)

target_link_libraries(grandmaster_minimal_test
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_uart
    hardware_timer
    hardware_irq
    hardware_clocks
    hardware_gpio
    common
)

# Enable USB output for startup messages only
pico_enable_stdio_usb(grandmaster_minimal_test 1)
pico_enable_stdio_uart(grandmaster_minimal_test 0)

# Generate PIO header (V3 - IRQ flag sync)
pico_generate_pio_header(grandmaster_minimal_test ${CMAKE_CURRENT_LIST_DIR}/pio/gps_discipline_v2.pio)
pico_generate_pio_header(grandmaster_minimal_test ${CMAKE_SOURCE_DIR}/pio/pps_capture.pio)

pico_add_extra_outputs(grandmaster_minimal_test)

# ============================================================================
# DUAL-CORE VERSION - Core 0: Timing, Core 1: Network/PTP
# Phase 1 implementation: Basic dual-core structure with test output
# ============================================================================
add_executable(grandmaster_dual_core
    main_dual_core.c
    gps.c
    discipline_v3.c
    ptp_grandmaster.c
)

target_include_directories(grandmaster_dual_core PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}
)

target_compile_definitions(grandmaster_dual_core PRIVATE
    BUILD_GRANDMASTER
    DUAL_CORE_BUILD
    PICO_CORE1_STACK_SIZE=0x1000
)

target_link_libraries(grandmaster_dual_core
    pico_stdlib
    pico_multicore          # CRITICAL: Multicore support
    hardware_pio
    hardware_uart
    hardware_timer
    hardware_irq
    hardware_clocks
    hardware_gpio
    common
    network                 # W5500 Ethernet and lwIP stack
)

# Enable USB output for both cores
pico_enable_stdio_usb(grandmaster_dual_core 1)
pico_enable_stdio_uart(grandmaster_dual_core 0)

# Generate PIO headers (V3 - GPIO pin sync)
pico_generate_pio_header(grandmaster_dual_core ${CMAKE_CURRENT_LIST_DIR}/pio/gps_discipline_v2.pio)
pico_generate_pio_header(grandmaster_dual_core ${CMAKE_SOURCE_DIR}/pio/pps_capture.pio)

pico_add_extra_outputs(grandmaster_dual_core)

# ============================================================================
# W5500 + PTP TEST - Standalone implementation with GPS discipline
# Tests simple W5500 driver with GPS-disciplined PTP grandmaster
# ============================================================================
add_executable(w5500_ptp_test
    main_w5500_ptp_test.c
    w5500_simple.c
    eth_simple.c
    gps.c
    discipline_v3.c
)

target_include_directories(w5500_ptp_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}
)

target_compile_definitions(w5500_ptp_test PRIVATE
    BUILD_GRANDMASTER
)

target_link_libraries(w5500_ptp_test
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_pio
    hardware_uart
    hardware_clocks
    hardware_irq
    common          # For ptp_protocol.c
)

# Generate PIO headers for GPS discipline
pico_generate_pio_header(w5500_ptp_test ${CMAKE_CURRENT_LIST_DIR}/pio/gps_discipline_v2.pio)
pico_generate_pio_header(w5500_ptp_test ${CMAKE_SOURCE_DIR}/pio/pps_capture.pio)

# Enable USB output
pico_enable_stdio_usb(w5500_ptp_test 1)
pico_enable_stdio_uart(w5500_ptp_test 0)

pico_add_extra_outputs(w5500_ptp_test)

# ============================================================================
# W5500 DUAL CORE - GPS-Disciplined PTP Grandmaster (Dual Core)
# Core 0: GPS discipline ONLY (minimal interference, ±200-400ns accuracy)
# Core 1: W5500 Ethernet + PTP protocol (heavy workload isolated)
# ============================================================================
add_executable(w5500_dual_core
    main_w5500_dual_core.c
    w5500_simple.c
    eth_simple.c
    gps.c
    discipline_v3.c
    pps_scheduler.c
)

target_include_directories(w5500_dual_core PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}
)

target_compile_definitions(w5500_dual_core PRIVATE
    BUILD_GRANDMASTER
    DUAL_CORE_BUILD
    PICO_CORE1_STACK_SIZE=0x1000
)

target_link_libraries(w5500_dual_core
    pico_stdlib
    pico_multicore          # Multicore support for dual-core architecture
    hardware_spi            # SPI for W5500
    hardware_pio
    hardware_dma
    hardware_uart
    hardware_timer
    hardware_irq
    hardware_clocks
    hardware_gpio
    common          # For ptp_protocol.c
)

# Generate PIO headers for GPS discipline and 1PPS output
pico_generate_pio_header(w5500_dual_core ${CMAKE_CURRENT_LIST_DIR}/pio/gps_discipline_v2.pio)
pico_generate_pio_header(w5500_dual_core ${CMAKE_SOURCE_DIR}/pio/pps_capture.pio)
pico_generate_pio_header(w5500_dual_core ${CMAKE_CURRENT_LIST_DIR}/pio/scheduled_1pps.pio)

# Enable USB output
pico_enable_stdio_usb(w5500_dual_core 1)
pico_enable_stdio_uart(w5500_dual_core 0)

pico_add_extra_outputs(w5500_dual_core)

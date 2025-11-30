add_executable(slave
    main.c
    ptp_slave.c
    discipline_core.c
)

target_include_directories(slave PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(slave
    pico_stdlib
    pico_multicore
    hardware_gpio
    common
    network
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(slave 1)
pico_enable_stdio_uart(slave 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(slave)

# ============================================================================
# W5500 PTP Slave - Dual Core Ethernet Slave
# Core 0: PTP discipline with PIO counter (sub-microsecond accuracy)
# Core 1: W5500 Ethernet + PTP slave protocol
# ============================================================================
add_executable(w5500_ptp_slave
    main_w5500_slave.c
    ptp_slave_w5500.c
    ptp_discipline.c
    pps_scheduler.c
    ../grandmaster/w5500_simple.c
    ../grandmaster/eth_simple.c
)

target_include_directories(w5500_ptp_slave PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}/grandmaster
    ${CMAKE_SOURCE_DIR}
)

target_compile_definitions(w5500_ptp_slave PRIVATE
    BUILD_SLAVE
    DUAL_CORE_BUILD
    PICO_CORE1_STACK_SIZE=0x1000
)

target_link_libraries(w5500_ptp_slave
    pico_stdlib
    pico_multicore          # Dual-core support
    hardware_spi            # SPI for W5500
    hardware_pio            # PIO counter
    hardware_timer
    hardware_irq
    hardware_clocks
    hardware_gpio
    common          # For ptp_protocol.c
)

# Generate PIO headers for counter, INT timestamp, and scheduled 1PPS
pico_generate_pio_header(w5500_ptp_slave ${CMAKE_CURRENT_LIST_DIR}/pio/counter_simple.pio)
pico_generate_pio_header(w5500_ptp_slave ${CMAKE_CURRENT_LIST_DIR}/pio/int_timestamp.pio)
pico_generate_pio_header(w5500_ptp_slave ${CMAKE_CURRENT_LIST_DIR}/pio/scheduled_1pps.pio)

# Enable USB output
pico_enable_stdio_usb(w5500_ptp_slave 1)
pico_enable_stdio_uart(w5500_ptp_slave 0)

pico_add_extra_outputs(w5500_ptp_slave)

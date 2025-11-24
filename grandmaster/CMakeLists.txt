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

# ============================================================================
# LTSP Grandmaster - GPS-Disciplined Time Source
# Core 0: GPS discipline + LTSP regression
# Core 1: W5500 Ethernet + LTSP PDU broadcast (1 Hz)
#
# Uses local copies of grandmaster files (not shared with PTP grandmaster)
# ============================================================================
add_executable(ltsp_gm
    main.c
    w5500_simple.c
    eth_simple.c
    gps.c
    discipline_v3.c
    pps_scheduler.c
    hw_timestamp.c
)

target_include_directories(ltsp_gm PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}/ltsp/common
    ${CMAKE_SOURCE_DIR}/ltsp
    ${CMAKE_SOURCE_DIR}
)

target_compile_definitions(ltsp_gm PRIVATE
    BUILD_GRANDMASTER
    DUAL_CORE_BUILD
    PICO_CORE1_STACK_SIZE=0x1000
)

target_link_libraries(ltsp_gm
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_pio
    hardware_dma
    hardware_uart
    hardware_timer
    hardware_irq
    hardware_clocks
    hardware_gpio
    common
    ltsp_common
)

# Generate PIO headers (reference originals — PIO programs are not modified)
pico_generate_pio_header(ltsp_gm ${CMAKE_SOURCE_DIR}/grandmaster/pio/gps_discipline_v2.pio)
pico_generate_pio_header(ltsp_gm ${CMAKE_SOURCE_DIR}/pio/pps_capture.pio)
pico_generate_pio_header(ltsp_gm ${CMAKE_SOURCE_DIR}/grandmaster/pio/scheduled_1pps.pio)
pico_generate_pio_header(ltsp_gm ${CMAKE_SOURCE_DIR}/grandmaster/pio/counter_simple.pio)
pico_generate_pio_header(ltsp_gm ${CMAKE_SOURCE_DIR}/grandmaster/pio/int_timestamp.pio)

# Enable USB output
pico_enable_stdio_usb(ltsp_gm 1)
pico_enable_stdio_uart(ltsp_gm 0)

pico_add_extra_outputs(ltsp_gm)

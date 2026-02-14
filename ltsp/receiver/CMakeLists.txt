# ============================================================================
# LTSP Receiver - Report-Only v0.1
# Core 0: PIO counter + HW timestamp FIFO drain
# Core 1: W5500 Ethernet + LTSP PDU reception + CSV output
#
# Uses local copies of shared files (not shared with PTP slave)
# ============================================================================
add_executable(ltsp_receiver
    main.c
    error_decomp.c
    w5500_simple.c
    eth_simple.c
)

target_include_directories(ltsp_receiver PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}/ltsp/common
    ${CMAKE_SOURCE_DIR}/ltsp
    ${CMAKE_SOURCE_DIR}
)

target_compile_definitions(ltsp_receiver PRIVATE
    BUILD_SLAVE
    DUAL_CORE_BUILD
    PICO_CORE1_STACK_SIZE=0x1000
)

target_link_libraries(ltsp_receiver
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_pio
    hardware_timer
    hardware_irq
    hardware_clocks
    hardware_gpio
    ltsp_common
)

# Generate PIO headers (reference originals — PIO programs are not modified)
pico_generate_pio_header(ltsp_receiver ${CMAKE_SOURCE_DIR}/slave/pio/counter_simple.pio)
pico_generate_pio_header(ltsp_receiver ${CMAKE_SOURCE_DIR}/slave/pio/int_timestamp.pio)

# Enable USB output
pico_enable_stdio_usb(ltsp_receiver 1)
pico_enable_stdio_uart(ltsp_receiver 0)

pico_add_extra_outputs(ltsp_receiver)

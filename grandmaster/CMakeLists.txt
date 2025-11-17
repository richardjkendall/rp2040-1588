add_executable(grandmaster
    main.c
    gps.c
    discipline_core.c
    wifi_init.c
    ptp_grandmaster.c
)

# Generate PIO header from .pio file
pico_generate_pio_header(grandmaster ${CMAKE_SOURCE_DIR}/pio/pps_capture.pio)

target_include_directories(grandmaster PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(grandmaster
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_uart
    hardware_timer
    hardware_irq
    pico_cyw43_arch_lwip_poll
    common
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(grandmaster 1)
pico_enable_stdio_uart(grandmaster 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(grandmaster)

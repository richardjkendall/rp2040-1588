add_executable(slave
    main.c
    wifi_init.c
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
    pico_cyw43_arch_lwip_poll
    common
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(slave 1)
pico_enable_stdio_uart(slave 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(slave)

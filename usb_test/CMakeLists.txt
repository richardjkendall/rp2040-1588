add_executable(usb_test
    main.c
)

target_link_libraries(usb_test
    pico_stdlib
    hardware_clocks
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(usb_test 1)
pico_enable_stdio_uart(usb_test 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(usb_test)

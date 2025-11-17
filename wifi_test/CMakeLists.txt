add_executable(wifi_test
    wifi_test.c
)

target_include_directories(wifi_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(wifi_test
    pico_stdlib
    pico_cyw43_arch_lwip_poll
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(wifi_test 1)
pico_enable_stdio_uart(wifi_test 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(wifi_test)

# WiFi Scanner
add_executable(wifi_scan
    wifi_scan.c
)

target_include_directories(wifi_scan PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(wifi_scan
    pico_stdlib
    pico_cyw43_arch_lwip_poll
)

pico_enable_stdio_usb(wifi_scan 1)
pico_enable_stdio_uart(wifi_scan 0)

pico_add_extra_outputs(wifi_scan)

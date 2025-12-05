# ============================================================================
# PTP Phase Offset Measurement Device
# GPS-calibrated independent measurement of GM vs Slave 1PPS phase offset
# ============================================================================
add_executable(measurement_device
    main_measurement_device.c
    ${CMAKE_SOURCE_DIR}/grandmaster/gps.c
    ${CMAKE_SOURCE_DIR}/grandmaster/discipline_v3.c
)

target_include_directories(measurement_device PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/grandmaster
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}
)

target_compile_definitions(measurement_device PRIVATE
    BUILD_MEASUREMENT
)

target_link_libraries(measurement_device
    pico_stdlib
    hardware_pio
    hardware_uart
    hardware_timer
    hardware_irq
    hardware_clocks
    hardware_gpio
    common
)

# Generate PIO headers for GPS discipline
pico_generate_pio_header(measurement_device ${CMAKE_SOURCE_DIR}/grandmaster/pio/gps_discipline_v2.pio)
pico_generate_pio_header(measurement_device ${CMAKE_SOURCE_DIR}/pio/pps_capture.pio)

# Generate PIO headers for phase measurement
pico_generate_pio_header(measurement_device ${CMAKE_CURRENT_LIST_DIR}/pio/gm_to_slave_counter.pio)
pico_generate_pio_header(measurement_device ${CMAKE_CURRENT_LIST_DIR}/pio/slave_to_gm_counter.pio)

# Enable USB output
pico_enable_stdio_usb(measurement_device 1)
pico_enable_stdio_uart(measurement_device 0)

pico_add_extra_outputs(measurement_device)

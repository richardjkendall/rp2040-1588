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

# ============================================================================
# PTP Phase Offset Measurement Device - WiFi Telemetry Version
# GPS-calibrated measurement with remote telemetry streaming (Pico W only)
# ============================================================================

# Only build WiFi version for Pico W (not Ethernet builds)
if(NOT USE_ETHERNET)
    add_executable(measurement_device_wifi
        main_measurement_device_wifi.c
        ring_buffer.c
        telemetry.c
        ${CMAKE_SOURCE_DIR}/grandmaster/gps.c
        ${CMAKE_SOURCE_DIR}/grandmaster/discipline_v3.c
    )

    target_include_directories(measurement_device_wifi PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_SOURCE_DIR}/grandmaster
        ${CMAKE_SOURCE_DIR}/common
        ${CMAKE_SOURCE_DIR}
    )

    target_compile_definitions(measurement_device_wifi PRIVATE
        BUILD_MEASUREMENT
        WIFI_TELEMETRY_ENABLED
    )

    target_link_libraries(measurement_device_wifi
        pico_stdlib
        pico_multicore              # CRITICAL: Dual-core support
        pico_cyw43_arch_lwip_poll   # WiFi + TCP stack
        hardware_pio
        hardware_uart
        hardware_timer
        hardware_irq
        hardware_clocks
        hardware_gpio
        common
    )

    # Generate PIO headers for GPS discipline
    pico_generate_pio_header(measurement_device_wifi ${CMAKE_SOURCE_DIR}/grandmaster/pio/gps_discipline_v2.pio)
    pico_generate_pio_header(measurement_device_wifi ${CMAKE_SOURCE_DIR}/pio/pps_capture.pio)

    # Generate PIO headers for phase measurement
    pico_generate_pio_header(measurement_device_wifi ${CMAKE_CURRENT_LIST_DIR}/pio/gm_to_slave_counter.pio)
    pico_generate_pio_header(measurement_device_wifi ${CMAKE_CURRENT_LIST_DIR}/pio/slave_to_gm_counter.pio)

    # Enable USB output for debugging
    pico_enable_stdio_usb(measurement_device_wifi 1)
    pico_enable_stdio_uart(measurement_device_wifi 0)

    pico_add_extra_outputs(measurement_device_wifi)

    message(STATUS "Building measurement_device_wifi with WiFi telemetry support")
endif()

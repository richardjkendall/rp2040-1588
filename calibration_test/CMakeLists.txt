cmake_minimum_required(VERSION 3.13)

# Calibration device executable
add_executable(calibration_device
    main_calibration_device.c
    pps_discipline.c
    ../slave/pps_scheduler.c
)

# Generate PIO headers
pico_generate_pio_header(calibration_device ${CMAKE_SOURCE_DIR}/grandmaster/pio/gps_discipline_v2.pio)
pico_generate_pio_header(calibration_device ${CMAKE_SOURCE_DIR}/slave/pio/scheduled_1pps.pio)

# Link libraries
target_link_libraries(calibration_device
    pico_stdlib
    hardware_pio
    hardware_spi
    hardware_dma
    hardware_clocks
)

# Include directories
target_include_directories(calibration_device PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/pio
    ${CMAKE_SOURCE_DIR}/slave
    ${CMAKE_SOURCE_DIR}/grandmaster
    ${CMAKE_SOURCE_DIR}/grandmaster/pio
)

# Enable USB output
pico_enable_stdio_usb(calibration_device 1)
pico_enable_stdio_uart(calibration_device 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(calibration_device)

# =============================================================================
# GPS Dual 1PPS Output - Second calibration device
# =============================================================================

# GPS dual PPS executable
add_executable(gps_dual_pps
    main_gps_dual_pps.c
    pps_discipline.c
    ../slave/pps_scheduler.c
)

# Generate PIO headers
pico_generate_pio_header(gps_dual_pps ${CMAKE_SOURCE_DIR}/grandmaster/pio/gps_discipline_v2.pio)
pico_generate_pio_header(gps_dual_pps ${CMAKE_SOURCE_DIR}/slave/pio/scheduled_1pps.pio)

# Link libraries
target_link_libraries(gps_dual_pps
    pico_stdlib
    hardware_pio
    hardware_spi
    hardware_dma
    hardware_clocks
)

# Include directories
target_include_directories(gps_dual_pps PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/pio
    ${CMAKE_SOURCE_DIR}/slave
    ${CMAKE_SOURCE_DIR}/grandmaster
    ${CMAKE_SOURCE_DIR}/grandmaster/pio
)

# Enable USB output
pico_enable_stdio_usb(gps_dual_pps 1)
pico_enable_stdio_uart(gps_dual_pps 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(gps_dual_pps)

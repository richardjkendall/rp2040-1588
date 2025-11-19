# Phase Logger Build Configuration

# Add executable first
add_executable(logger
    main.c
    timestamp_processor.c
    pio_timestamp.c
    gps_simple.c
)

# Generate PIO header from .pio file (must be after add_executable)
pico_generate_pio_header(logger ${CMAKE_CURRENT_LIST_DIR}/pio/timestamp_capture.pio)

# Include directories
target_include_directories(logger PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Link libraries
target_link_libraries(logger
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_uart
    hardware_irq
    hardware_clocks
    hardware_dma
)

# Enable USB serial output
pico_enable_stdio_usb(logger 1)
pico_enable_stdio_uart(logger 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(logger)

# Final Phase Meter Build Configuration

add_executable(final_phase_meter
    main.c
)

# Generate PIO headers from .pio files
pico_generate_pio_header(final_phase_meter ${CMAKE_CURRENT_LIST_DIR}/frequency_counter.pio)
pico_generate_pio_header(final_phase_meter ${CMAKE_CURRENT_LIST_DIR}/pulse_generator.pio)
pico_generate_pio_header(final_phase_meter ${CMAKE_CURRENT_LIST_DIR}/phase_diff.pio)

# Link libraries
target_link_libraries(final_phase_meter
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_sync
    pico_multicore
)

# Enable USB serial output
pico_enable_stdio_usb(final_phase_meter 1)
pico_enable_stdio_uart(final_phase_meter 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(final_phase_meter)

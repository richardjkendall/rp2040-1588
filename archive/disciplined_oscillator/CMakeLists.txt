# Disciplined Oscillator Build Configuration

add_executable(disciplined_oscillator
    main.c
)

# Generate PIO headers from .pio files
pico_generate_pio_header(disciplined_oscillator ${CMAKE_CURRENT_LIST_DIR}/frequency_counter.pio)
pico_generate_pio_header(disciplined_oscillator ${CMAKE_CURRENT_LIST_DIR}/pulse_generator.pio)


# Link libraries
target_link_libraries(disciplined_oscillator
    pico_stdlib
    hardware_pio
    pico_multicore
)

# Enable USB serial output
pico_enable_stdio_usb(disciplined_oscillator 1)
pico_enable_stdio_uart(disciplined_oscillator 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(disciplined_oscillator)

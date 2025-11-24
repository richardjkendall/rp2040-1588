/**
 * GPS-Disciplined 100pps Oscillator
 *
 * This application creates a highly stable 100Hz pulse train on a GPIO pin.
 * It uses a software-based Phase-Locked Loop (PLL) architecture running
 * across both RP2040 cores and two PIO state machines.
 *
 * Core 0: Measurement Engine
 *  - Runs a PIO frequency counter to measure the precise number of system
 *    clock cycles in one GPS second.
 *  - Calculates a correction factor for the system clock's drift.
 *  - Handles USB serial output.
 *
 * Core 1: Synthesis Engine
 *  - Runs a PIO pulse generator.
 *  - A 100Hz repeating timer ISR, running from SRAM, calculates a corrected
 *    delay and pushes it to the PIO to generate the next pulse. This keeps
 *    the output frequency locked to the GPS reference.
 *
 * Hardware:
 *  - GPS 1PPS Input on GPIO 16
 *  - Disciplined 100pps Output on GPIO 20
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/sync.h" // For critical_section
#include "frequency_counter.pio.h"
#include "pulse_generator.pio.h"

// --- Pin Configuration ---
#define GPS_1PPS_PIN         16
#define OUTPUT_100PPS_PIN    20

// --- PIO Configuration ---
#define DISCIPLINE_PIO pio0

// --- PLL Configuration ---
#define IDEAL_SYS_CLOCKS_PER_SECOND 125000000
#define IDEAL_CLOCKS_PER_100HZ_PERIOD (IDEAL_SYS_CLOCKS_PER_SECOND / 100)
#define PULSE_HIGH_CYCLES 1248 // approx 10us pulse width

// --- Shared State ---
// We share the raw clock measurement from Core 0 to Core 1.
// It is protected by a critical section to prevent race conditions.
volatile uint32_t g_measured_clocks_per_gps_second = IDEAL_SYS_CLOCKS_PER_SECOND;
critical_section_t g_correction_crit_sec;


// =============================================================================
//  Core 1 - The Synthesis Engine (Real-Time)
// =============================================================================

uint sm_pulse_generator; // The SM for the pulse generator

/**
 * @brief The ISR for the 100Hz synthesis timer.
 * This function uses integer-only math for determinism.
 */
bool __not_in_flash_func(synthesis_isr_callback)(struct repeating_timer *t) {
    uint32_t local_measured_clocks;
    critical_section_enter_blocking(&g_correction_crit_sec);
    local_measured_clocks = g_measured_clocks_per_gps_second;
    critical_section_exit(&g_correction_crit_sec);

    // Calculate the corrected total period using 64-bit integer arithmetic
    // to avoid overflow and floating point math.
    // Corrected Period = Ideal Period * (Ideal Clocks / Measured Clocks)
    uint32_t total_period_cycles = local_measured_clocks / 100;

    // Calculate the duration for the LOW phase
    uint32_t low_cycles = total_period_cycles - PULSE_HIGH_CYCLES;

    // Push both durations to the pulse generator state machine.
    // These are non-blocking puts. If the FIFO is full (which it shouldn't be
    // in normal operation), the values are dropped for that cycle.
    pio_sm_put(DISCIPLINE_PIO, sm_pulse_generator, PULSE_HIGH_CYCLES);
    pio_sm_put(DISCIPLINE_PIO, sm_pulse_generator, low_cycles);

    return true; // Keep the timer repeating
}

/**
 * @brief Entry point for Core 1.
 */
void core1_synthesis_engine(void) {
    // --- Setup the Pulse Generator PIO ---
    uint offset = pio_add_program(DISCIPLINE_PIO, &pulse_generator_program);
    sm_pulse_generator = pio_claim_unused_sm(DISCIPLINE_PIO, true);
    pio_sm_config c = pulse_generator_program_get_default_config(offset);
    pio_gpio_init(DISCIPLINE_PIO, OUTPUT_100PPS_PIN);
    pio_sm_set_consecutive_pindirs(DISCIPLINE_PIO, sm_pulse_generator, OUTPUT_100PPS_PIN, 1, true);
    sm_config_set_set_pins(&c, OUTPUT_100PPS_PIN, 1);
    sm_config_set_clkdiv(&c, 1.0f);
    pio_sm_init(DISCIPLINE_PIO, sm_pulse_generator, offset, &c);
    pio_sm_set_enabled(DISCIPLINE_PIO, sm_pulse_generator, true);

    // --- Setup the 100Hz Repeating Timer ---
    struct repeating_timer timer;
    add_repeating_timer_us(-10000, synthesis_isr_callback, NULL, &timer);

    while (true) {
        tight_loop_contents();
    }
}


// =============================================================================
//  Core 0 - The Measurement Engine (Supervisory)
// =============================================================================

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    // Initialize critical section for sharing the clock measurement
    critical_section_init(&g_correction_crit_sec);

    printf("\n=== GPS-Disciplined Corrective Oscillator (v2) ===\n");
    printf("1PPS Input: GPIO%d, 100pps Output: GPIO%d\n", GPS_1PPS_PIN, OUTPUT_100PPS_PIN);

    // --- Setup the Frequency Counter PIO on Core 0 ---
    uint offset = pio_add_program(DISCIPLINE_PIO, &frequency_counter_program);
    uint sm_freq_counter = pio_claim_unused_sm(DISCIPLINE_PIO, true);
    pio_sm_config c = frequency_counter_program_get_default_config(offset);
    pio_gpio_init(DISCIPLINE_PIO, GPS_1PPS_PIN);
    sm_config_set_in_pins(&c, GPS_1PPS_PIN);
    sm_config_set_jmp_pin(&c, GPS_1PPS_PIN);
    sm_config_set_clkdiv(&c, 1.0f);
    pio_sm_init(DISCIPLINE_PIO, sm_freq_counter, offset, &c);
    pio_sm_set_enabled(DISCIPLINE_PIO, sm_freq_counter, true);

    printf("Launched Measurement Engine on Core 0.\n");

    // --- Launch the Synthesis Engine on Core 1 ---
    multicore_launch_core1(core1_synthesis_engine);
    printf("Launched Synthesis Engine on Core 1.\n\n");
    printf("Running... outputting debug info every second.\n");
    printf("--------------------------------------------\n");
    printf("Measured Clocks per GPS Second | Status\n");
    printf("--------------------------------------------\n");


    // --- Main loop on Core 0 ---
    while (true) {
        uint32_t raw_value = pio_sm_get_blocking(DISCIPLINE_PIO, sm_freq_counter);

        // The PIO loop takes 2 cycles per tick.
        uint32_t measured_clocks = (0xFFFFFFFF - raw_value) * 2;

        // Sanity-check the measurement. A valid measurement should be
        // within ~10% of the ideal value. This prevents glitches on the
        // 1PPS line from stalling the synthesis engine.
        if (measured_clocks > (IDEAL_SYS_CLOCKS_PER_SECOND * 0.9) &&
            measured_clocks < (IDEAL_SYS_CLOCKS_PER_SECOND * 1.1)) {

            critical_section_enter_blocking(&g_correction_crit_sec);
            g_measured_clocks_per_gps_second = measured_clocks;
            critical_section_exit(&g_correction_crit_sec);
            printf("%-30u | OK\n", measured_clocks);

        } else {
            printf("%-30u | DISCARDED (out of range)\n", measured_clocks);
        }
    }

    return 0;
}

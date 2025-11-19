/**
 * Timestamp Processor (Core 1)
 *
 * Processes hardware timestamps from PIO and calculates phase offsets
 */

#include "shared_state.h"
#include "pio_timestamp.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <stdio.h>

// Debug flag
#define DEBUG_PHASE 0

// Global shared state instances
timestamp_data_t timestamp_data = {0};
phase_measurement_t phase_measurement = {0};
gps_status_t gps_status = {0};

// Local state
static uint32_t last_gps_ref = 0;
static bool gps_ref_initialized = false;

/**
 * Calculate phase offset from PIO synchronized counter timestamps
 *
 * The PIO counters are DECREMENTING 32-bit counters at 125 MHz (8ns per tick).
 * All SMs started simultaneously, counters stay synchronized.
 * We calculate how much time elapsed from reference to signal.
 *
 * @param reference GPS PPS timestamp (counter value at GPS edge)
 * @param signal Signal timestamp (GM or Slave PPS, counter value at edge)
 * @return Phase offset in nanoseconds (0 to 10ms for 100 PPS)
 */
static int64_t calculate_phase_offset_ns(uint32_t reference, uint32_t signal) {
    // Calculate elapsed ticks (decrementing counter: ref - signal)
    // Handles 32-bit wrap-around automatically
    uint32_t elapsed_ticks = reference - signal;

#if DEBUG_PHASE
    static uint32_t debug_count = 0;
    if (debug_count++ < 10) {
        printf("  [PHASE] ref=0x%08lx sig=0x%08lx elapsed_ticks=%lu\n",
               (unsigned long)reference, (unsigned long)signal,
               (unsigned long)elapsed_ticks);
    }
#endif

    // Convert to nanoseconds (8ns per tick)
    uint32_t ns_per_tick = pio_timestamp_get_ns_per_tick();
    uint64_t elapsed_ns = (uint64_t)elapsed_ticks * ns_per_tick;

    // Normalize to 0-10ms range (100 PPS period = 10,000,000 ns)
    elapsed_ns = elapsed_ns % 10000000ULL;

    return (int64_t)elapsed_ns;
}

/**
 * Process timestamps and calculate phase measurements
 */
static void process_timestamps(void) {
    // Update GPS reference when new GPS PPS arrives
    if (timestamp_data.gps_pps_valid) {
        last_gps_ref = timestamp_data.gps_pps_timestamp;
        gps_ref_initialized = true;
        timestamp_data.gps_pps_valid = false;  // Clear flag
    }

    // Need valid GPS reference to calculate phase offsets
    if (!gps_ref_initialized) {
        return;
    }

    // Process GM PPS if available
    if (timestamp_data.gm_pps_valid) {
        phase_measurement.gm_phase_offset_ns =
            calculate_phase_offset_ns(last_gps_ref, timestamp_data.gm_pps_timestamp);
        timestamp_data.gm_pps_valid = false;  // Clear flag
    }

    // Process Slave PPS if available
    if (timestamp_data.slave_pps_valid) {
        phase_measurement.slave_phase_offset_ns =
            calculate_phase_offset_ns(last_gps_ref, timestamp_data.slave_pps_timestamp);

        // Calculate direct GM-Slave phase difference
        // This is what the oscilloscope measures
        phase_measurement.gm_slave_phase_diff_ns =
            phase_measurement.slave_phase_offset_ns -
            phase_measurement.gm_phase_offset_ns;

        // Mark measurement as valid and increment sample count
        phase_measurement.data_valid = true;
        phase_measurement.sample_count++;
        phase_measurement.last_update_us = time_us_64();

        timestamp_data.slave_pps_valid = false;  // Clear flag
    }
}

/**
 * Core 1 entry point
 */
void core1_entry(void) {
    printf("Core 1: Timestamp processor starting\n");

    // Main processing loop
    while (true) {
        // Process any available timestamps
        process_timestamps();

        // Minimal delay to prevent busy-wait hogging
        // The IRQ handler populates timestamp_data, this loop processes it
        tight_loop_contents();
    }
}

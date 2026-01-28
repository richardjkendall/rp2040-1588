/**
 * Timestamp Processor (Core 1)
 *
 * Processes hardware timestamps from PIO/DMA and calculates phase offsets
 * using absolute system time.
 */

#include "shared_state.h"
#include "pio_timestamp.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/time.h" // For absolute_time_t functions
#include <stdio.h>

// --- Global State Instances (defined here) ---
// These are accessed by Core 0 for logging and by the DMA IRQ handler.
timestamp_data_t timestamp_data = {0};
phase_measurement_t phase_measurement = {0};
gps_status_t gps_status = {0};


// --- Core 1 Processing Logic ---

/**
 * @brief Processes available timestamps and calculates phase measurements.
 * This function runs on Core 1, checking for new data from the DMA IRQ handler.
 */
static void process_timestamps(void) {
    // Local storage for the latest valid timestamps for this processing cycle
    static absolute_time_t local_gps_abs_time;
    static absolute_time_t local_gm_abs_time;
    static absolute_time_t local_slave_abs_time;

    // Flags to track if we've received new data for each source in this cycle
    static bool new_gps_data = false;
    static bool new_gm_data = false;
    static bool new_slave_data = false;

    // Check if new GPS data is available
    if (timestamp_data.gps_pps_valid) {
        local_gps_abs_time = timestamp_data.gps_abs_time;
        timestamp_data.gps_pps_valid = false; // Consume the flag
        new_gps_data = true;
    }

    // Check if new GM data is available
    if (timestamp_data.gm_pps_valid) {
        local_gm_abs_time = timestamp_data.gm_abs_time;
        timestamp_data.gm_pps_valid = false; // Consume the flag
        new_gm_data = true;
    }

    // Check if new Slave data is available
    if (timestamp_data.slave_pps_valid) {
        local_slave_abs_time = timestamp_data.slave_abs_time;
        timestamp_data.slave_pps_valid = false; // Consume the flag
        new_slave_data = true;
    }

    // If we have at least one new GPS event, and have seen GM and Slave events,
    // we can calculate a new set of phase measurements.
    // We use GPS as the primary reference for a measurement "epoch".
    if (new_gps_data && new_gm_data && new_slave_data) {
        // --- ATOMIC UPDATE BLOCK ---
        // All writes to the shared phase_measurement struct happen here at once.

        // Convert absolute_time_t to nanoseconds since boot for calculations
        uint64_t gps_ns = to_us_since_boot(local_gps_abs_time) * 1000ULL;
        uint64_t gm_ns = to_us_since_boot(local_gm_abs_time) * 1000ULL;
        uint64_t slave_ns = to_us_since_boot(local_slave_abs_time) * 1000ULL;

        // Update the absolute nanosecond timestamps in the shared struct
        phase_measurement.gps_timestamp_ns = gps_ns;
        phase_measurement.gm_timestamp_ns = gm_ns;
        phase_measurement.slave_timestamp_ns = slave_ns;

        // Calculate phase offsets relative to GPS
        phase_measurement.gm_phase_offset_ns = gm_ns - gps_ns;
        phase_measurement.slave_phase_offset_ns = slave_ns - gps_ns;

        // Calculate direct GM-Slave phase difference
        phase_measurement.gm_slave_phase_diff_ns = slave_ns - gm_ns;

        // Increment counters
        phase_measurement.gps_pps_count = timestamp_data.gps_pps_count;
        phase_measurement.gm_pps_count = timestamp_data.gm_pps_count;
        phase_measurement.slave_pps_count = timestamp_data.slave_pps_count;

        // Mark data as valid and update metadata
        phase_measurement.data_valid = true;
        phase_measurement.sample_count++;
        phase_measurement.last_update_us = time_us_64();

        // Reset flags for the next measurement cycle
        new_gps_data = false;
        new_gm_data = false;
        new_slave_data = false;
    }
}


// --- Core 1 Entry Point ---

/**
 * @brief Core 1 entry point.
 * Initializes state and enters the main processing loop.
 */
void core1_entry(void) {
    printf("Core 1: Timestamp processor starting\n");

    // Main processing loop
    while (true) {
        process_timestamps();

        // Minimal delay to prevent busy-wait hogging
        tight_loop_contents();
    }
}
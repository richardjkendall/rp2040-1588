/**
 * Shared State between Core 0 and Core 1 (Phase Logger)
 *
 * Core 0: GPS NMEA parsing, serial output, data formatting
 * Core 1: PIO timestamp processing, phase calculations
 */

#ifndef LOGGER_SHARED_STATE_H
#define LOGGER_SHARED_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/time.h" // Needed for absolute_time_t

/**
 * Raw timestamp data from PIO state machines and CPU
 * Written by IRQ handler, read by Core 1
 */
typedef struct {
    // GPS 1 PPS data
    volatile absolute_time_t gps_abs_time;    // Absolute system time (1us resolution)
    volatile uint32_t gps_raw_pio_val;        // Raw PIO counter value (8ns resolution)
    volatile bool gps_pps_valid;

    // Grandmaster 100 PPS data
    volatile absolute_time_t gm_abs_time;
    volatile uint32_t gm_raw_pio_val;
    volatile bool gm_pps_valid;

    // Slave 100 PPS data
    volatile absolute_time_t slave_abs_time;
    volatile uint32_t slave_raw_pio_val;
    volatile bool slave_pps_valid;

    // Pulse counters
    volatile uint32_t gps_pps_count;
    volatile uint32_t gm_pps_count;
    volatile uint32_t slave_pps_count;
} timestamp_data_t;

/**
 * Processed phase measurements
 * Written by Core 1, read by Core 0 for logging
 */
typedef struct {
    // Absolute nanosecond timestamps (derived from absolute_time_t and raw PIO)
    volatile uint64_t gps_timestamp_ns;
    volatile uint64_t gm_timestamp_ns;
    volatile uint64_t slave_timestamp_ns;

    // Phase offsets relative to GPS epoch
    volatile int64_t gm_phase_offset_ns;
    volatile int64_t slave_phase_offset_ns;
    volatile int64_t gm_slave_phase_diff_ns;

    // Counters and validity
    volatile uint32_t gps_pps_count;
    volatile uint32_t gm_pps_count;
    volatile uint32_t slave_pps_count;
    volatile bool data_valid;
    volatile uint32_t sample_count;
    volatile uint64_t last_update_us;
} phase_measurement_t;

/**
 * GPS status information
 * Written by Core 0 GPS parser, read by Core 1 for validation
 */
typedef struct {
    volatile bool gps_fix;
    volatile uint8_t gps_sats;
    volatile char gps_utc_time[16];
    volatile char gps_date[12];
    volatile bool time_valid;
} gps_status_t;

// Global instances
extern timestamp_data_t timestamp_data;
extern phase_measurement_t phase_measurement;
extern gps_status_t gps_status;

#endif // LOGGER_SHARED_STATE_H
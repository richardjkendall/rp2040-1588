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

/**
 * Raw timestamp data from PIO state machines
 * Written by IRQ handler, read by Core 1
 */
typedef struct {
    // PIO timestamps (32-bit, decrementing counter @ 125 MHz = 8ns resolution)
    volatile uint32_t gps_pps_timestamp;      // GPS 1 PPS edge timestamp
    volatile uint32_t gm_pps_timestamp;       // Grandmaster 100 PPS edge timestamp
    volatile uint32_t slave_pps_timestamp;    // Slave 100 PPS edge timestamp

    // Validity flags
    volatile bool gps_pps_valid;
    volatile bool gm_pps_valid;
    volatile bool slave_pps_valid;

    // Pulse counters
    volatile uint32_t gps_pps_count;          // GPS pulse counter (increments by 1)
    volatile uint32_t gm_pps_count;           // GM pulse counter (0-99, wraps)
    volatile uint32_t slave_pps_count;        // Slave pulse counter (0-99, wraps)
} timestamp_data_t;

/**
 * Processed phase measurements
 * Written by Core 1, read by Core 0 for logging
 */
typedef struct {
    // Phase offsets relative to GPS epoch (0-10ms for 100 PPS)
    volatile int64_t gm_phase_offset_ns;      // GM phase vs GPS reference
    volatile int64_t slave_phase_offset_ns;   // Slave phase vs GPS reference
    volatile int64_t gm_slave_phase_diff_ns;  // Direct GM-Slave comparison

    // Data validity
    volatile bool data_valid;                 // True when measurements are ready
    volatile uint32_t sample_count;           // Total samples processed

    // Last update time for monitoring
    volatile uint64_t last_update_us;
} phase_measurement_t;

/**
 * GPS status information
 * Written by Core 0 GPS parser, read by Core 1 for validation
 */
typedef struct {
    volatile bool gps_fix;                    // GPS has valid fix
    volatile uint8_t gps_sats;                // Number of satellites
    volatile char gps_utc_time[16];           // "HH:MM:SS.sss" format
    volatile char gps_date[12];               // "YYYY-MM-DD" format
    volatile bool time_valid;                 // True when UTC time is valid
} gps_status_t;

// Global instances (defined in timestamp_processor.c)
extern timestamp_data_t timestamp_data;
extern phase_measurement_t phase_measurement;
extern gps_status_t gps_status;

#endif // LOGGER_SHARED_STATE_H

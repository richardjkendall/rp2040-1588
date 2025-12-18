/**
 * PPS Discipline Module - Calibration Device
 *
 * Disciplines local crystal to external GM 1PPS reference.
 * Simplified version of GPS discipline - no NMEA, just count PPS edges.
 */

#ifndef PPS_DISCIPLINE_H
#define PPS_DISCIPLINE_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/types.h"  // For uint typedef

// Discipline statistics
typedef struct {
    bool locked;                    // Lock status
    int64_t crystal_error_ns;       // Crystal error vs reference
    double crystal_ppm;             // Crystal error in PPM
    double scale_factor;            // Crystal scale factor
    uint64_t pps_count;             // PPS pulses received
    uint64_t discipline_updates;    // Discipline loop updates
    uint64_t reference_time_ns;     // Current reference time
} pps_discipline_stats_t;

// Global stats instance (for external access)
extern pps_discipline_stats_t pps_stats;

/**
 * Initialize PPS discipline system
 *
 * @param pps_input_pin GPIO pin for external PPS input
 * @return true if successful, false on error
 */
bool pps_discipline_init(uint pps_input_pin);

/**
 * Handle PPS edge event (called from ISR/main loop)
 *
 * @param counter_value PIO counter value at PPS edge
 */
void pps_discipline_on_edge(uint32_t counter_value);

/**
 * Get current reference time in nanoseconds
 *
 * Interpolates between PPS edges using characterized crystal scale factor.
 * Equivalent to get_gps_time_ns() but for PPS reference.
 *
 * @return Time in nanoseconds since first PPS edge
 */
uint64_t get_reference_time_ns(void);

/**
 * Get current scale factor
 *
 * @return Crystal scale factor (1.0 = perfect, >1.0 = fast, <1.0 = slow)
 */
double pps_discipline_get_scale_factor(void);

#endif // PPS_DISCIPLINE_H

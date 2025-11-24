/**
 * GPS Disciplined Clock - V3 with PIO IRQ Flag Synchronization
 */

#ifndef DISCIPLINE_V3_H
#define DISCIPLINE_V3_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize GPS discipline system V3
 * Uses PIO IRQ flags instead of continuous DMA
 *
 * @return true on success
 */
bool discipline_init_v3(void);

/**
 * Get disciplined time in microseconds
 */
uint64_t get_disciplined_time_us(void);

/**
 * Generate 100 PPS output pulses
 */
void discipline_generate_100pps(void);

/**
 * Update discipline statistics
 */
void discipline_update_stats(void);

/**
 * Get debug stats for 100 PPS generation
 */
void discipline_get_100pps_stats(uint32_t *fired, uint32_t *out_of_range, uint32_t *too_early, uint32_t *too_late, uint32_t *guard_blocked);

/**
 * Get current time correction in microseconds
 */
int64_t discipline_get_correction_us(void);

/**
 * Get disciplined time error in nanoseconds (PTP accuracy metric)
 * This is the residual error in disciplined time after correction
 */
int64_t discipline_get_disciplined_error_ns(void);

/**
 * Get debug info about frequency compensation
 */
void discipline_get_debug_info(int64_t *elapsed_raw, double *freq_ppm, int64_t *freq_correction_calc);

#endif // DISCIPLINE_V3_H

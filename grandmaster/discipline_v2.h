/**
 * GPS Disciplined Clock - PIO + DMA Architecture V2
 *
 * Hardware-accelerated GPS discipline using frequency counter architecture
 */

#ifndef DISCIPLINE_V2_H
#define DISCIPLINE_V2_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize GPS discipline system
 * Sets up PIO counter, DMA chain, and PPS capture
 *
 * @return true on success, false on failure
 */
bool discipline_init_v2(void);

/**
 * Get disciplined time in microseconds
 * Applies software correction to hardware timer
 *
 * @return Disciplined time in microseconds since boot
 */
uint64_t get_disciplined_time_us(void);

/**
 * Generate 100 PPS output pulses (GPS-disciplined)
 * Call from main loop - polls disciplined time and generates pulses
 */
void discipline_generate_100pps(void);

/**
 * Update discipline statistics
 * Optional - call from main loop to publish current disciplined time
 */
void discipline_update_stats(void);

#endif // DISCIPLINE_V2_H

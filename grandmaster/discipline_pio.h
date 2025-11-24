/**
 * PIO-based Clock Discipline
 *
 * Hardware-accelerated GPS discipline using PIO + DMA
 * No tight loops, no Core 1 - interrupt/DMA driven
 */

#ifndef DISCIPLINE_PIO_H
#define DISCIPLINE_PIO_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize PIO-based discipline system
 * Sets up PIO counter, DMA broadcast, and prepares for GPS PPS
 *
 * @return true on success, false on failure
 */
bool discipline_pio_init(void);

/**
 * GPS PPS callback - called from GPS IRQ handler
 * Captures counter value and runs PI controller
 *
 * @param pps_timestamp_us Hardware timestamp of PPS edge
 * @param valid Whether GPS has valid fix
 */
void discipline_pio_gps_pps_callback(uint64_t pps_timestamp_us, bool valid);

/**
 * Update disciplined clock time
 * Call periodically (e.g., in main loop) to publish current time
 * Much less critical than Core 1 version - just for stats
 */
void discipline_pio_update(void);

#endif // DISCIPLINE_PIO_H

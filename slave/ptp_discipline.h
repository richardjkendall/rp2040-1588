/**
 * PTP Discipline Module - IEEE 1588 Slave Clock Discipline
 *
 * Disciplines local clock to PTP grandmaster using:
 * - PIO counter for crystal characterization
 * - IEEE 1588 offset and path delay calculation
 * - PI servo controller
 */

#ifndef PTP_DISCIPLINE_H
#define PTP_DISCIPLINE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize PTP discipline system
 *
 * Sets up:
 * - PIO counter (free-running at 83.33 MHz)
 * - Discipline state variables
 * - Servo controller
 *
 * @return true on success, false on failure
 */
bool ptp_discipline_init(void);

/**
 * Get current PTP time (interpolated between Sync messages)
 *
 * Uses:
 * - Last known master time (from Sync/Follow_Up)
 * - Elapsed time since last Sync
 * - Crystal characterization (scale factor)
 * - Servo controller frequency offset
 *
 * @return Current PTP time in nanoseconds
 */
uint64_t get_ptp_time_ns(void);

/**
 * Update discipline based on new Sync/Follow_Up and Delay_Req/Resp cycle
 *
 * Called by Core 0 main loop when complete timestamp set available.
 * Performs:
 * 1. Mean path delay calculation (IEEE 1588 Eq. 3)
 * 2. Offset from master calculation (IEEE 1588 Eq. 4)
 * 3. Crystal characterization from PIO counter
 * 4. PI servo controller update
 * 5. Time base adjustment
 *
 * This function should be called frequently (100ms period recommended)
 */
void ptp_discipline_update(void);

#endif // PTP_DISCIPLINE_H

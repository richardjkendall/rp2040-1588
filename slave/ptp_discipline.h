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

/**
 * Read hardware timestamp from W5500 INT pin (via PIO)
 *
 * Reads PIO FIFO for INT pin edge detection timestamp.
 * Non-blocking - returns false if FIFO empty.
 *
 * @param counter_value Output: PIO counter value at INT edge
 * @return true if timestamp available, false if FIFO empty
 */
bool read_int_hardware_timestamp(uint32_t *counter_value);

/**
 * Convert PIO counter value to nanoseconds
 *
 * @param counter_value PIO counter snapshot
 * @return Elapsed time in nanoseconds
 */
uint64_t counter_to_ns(uint32_t counter_value);

/**
 * Convert PIO counter delta to nanoseconds with crystal correction
 *
 * Counter counts DOWN, so earlier_counter should be > later_counter
 * Applies crystal characterization for sub-nanosecond accuracy
 *
 * @param earlier_counter Counter value at earlier time
 * @param later_counter Counter value at later time
 * @return Delta in nanoseconds (positive = time elapsed)
 */
int64_t counter_delta_to_ns(uint32_t earlier_counter, uint32_t later_counter);

/**
 * Find most recent hardware timestamp for RX event correlation
 *
 * Searches buffer for hardware timestamp that occurred just before software timestamp
 * Used to calculate RX latency and correct t2 to wire time
 *
 * @param counter_sw Software counter snapshot
 * @param system_us_sw System time when software captured timestamp
 * @param counter_hw Output: Hardware counter value if found
 * @param latency_ns Output: Calculated RX latency in nanoseconds
 * @return true if suitable hardware timestamp found
 */
bool find_hw_timestamp_for_rx(uint32_t counter_sw, uint64_t system_us_sw,
                              uint32_t *counter_hw, int64_t *latency_ns);

/**
 * Get count of outliers rejected by Kalman filter
 *
 * @return Number of measurements rejected due to being > 3σ from prediction
 */
uint32_t ptp_discipline_get_outliers_rejected(void);

#endif // PTP_DISCIPLINE_H

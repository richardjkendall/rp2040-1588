/**
 * Hardware Timestamp Correlation for Grandmaster
 *
 * Correlates software timestamps with W5500 INT pin hardware timestamps
 * to accurately determine packet wire times.
 */

#ifndef HW_TIMESTAMP_H
#define HW_TIMESTAMP_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize hardware timestamp system
 * Must be called before using timestamp correlation
 */
bool hw_timestamp_init(void);

/**
 * Poll HW timestamp FIFO and store any pending timestamps
 * Call this frequently in main loop to minimize capture latency
 */
void hw_timestamp_poll_fifo(void);

/**
 * Store a timestamp directly into the circular buffer
 * Used when timestamps are read outside normal polling flow
 */
void hw_timestamp_store_in_buffer(uint32_t counter_value);

/**
 * Read current PIO counter value (for software timestamps)
 * Call this BEFORE sending/receiving packet
 */
uint32_t hw_timestamp_read_counter(void);

/**
 * Find hardware timestamp for recent TX event
 *
 * @param counter_before Counter value captured before TX
 * @param latency_ns Output: TX latency in nanoseconds
 * @return true if hardware timestamp found and correlated
 */
bool hw_timestamp_find_tx(uint32_t counter_before, int64_t *latency_ns);

/**
 * Find hardware timestamp for recent RX event
 *
 * @param counter_after Counter value captured after RX
 * @param latency_ns Output: RX latency in nanoseconds (negative, time from wire to SW)
 * @return true if hardware timestamp found and correlated
 */
bool hw_timestamp_find_rx(uint32_t counter_after, int64_t *latency_ns);

/**
 * Convert counter delta to nanoseconds
 * Counter counts DOWN, so delta = counter_before - counter_after
 */
int64_t hw_timestamp_counter_to_ns(uint32_t counter_before, uint32_t counter_after);

/**
 * Get TX latency running mean in nanoseconds (EMA over successful correlations)
 */
int64_t hw_timestamp_get_tx_latency_mean_ns(void);

/**
 * Get debug statistics for HW timestamp system
 *
 * @param poll_count Output: Number of polling calls
 * @param fifo_hits Output: Number of times FIFO had data
 * @param buffer_count Output: Current number of timestamps in buffer
 */
void hw_timestamp_get_debug_stats(uint32_t *poll_count, uint32_t *fifo_hits, uint32_t *buffer_count);

#endif // HW_TIMESTAMP_H

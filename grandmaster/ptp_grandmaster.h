/**
 * PTP Grandmaster Protocol Implementation
 *
 * Implements IEEE 1588-2008 grandmaster functionality:
 * - Announce messages (declare grandmaster status)
 * - Sync messages (time synchronization events)
 * - Follow_Up messages (precise timestamps)
 */

#ifndef PTP_GRANDMASTER_H
#define PTP_GRANDMASTER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Initialize PTP grandmaster
 *
 * Sets up UDP sockets, multicast groups, and PTP clock identity.
 *
 * @return true if successful, false otherwise
 */
bool ptp_grandmaster_init(void);

/**
 * PTP grandmaster periodic processing
 *
 * Call this regularly (e.g., every 100ms) to handle PTP message transmission.
 * Messages are sent at 1 Hz intervals.
 */
void ptp_grandmaster_process(void);

/**
 * Get PTP statistics
 *
 * @param announce_count Output: Number of Announce messages sent
 * @param sync_count Output: Number of Sync messages sent
 * @param followup_count Output: Number of Follow_Up messages sent
 */
void ptp_grandmaster_get_stats(uint32_t *announce_count, uint32_t *sync_count, uint32_t *followup_count);

#endif // PTP_GRANDMASTER_H

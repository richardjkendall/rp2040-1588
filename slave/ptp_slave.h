/**
 * PTP Slave - Receives PTP packets from grandmaster
 */

#ifndef PTP_SLAVE_H
#define PTP_SLAVE_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Initialize PTP slave
 * Sets up UDP sockets to receive PTP packets
 *
 * Returns: true on success, false on error
 */
bool ptp_slave_init(void);

/**
 * Process received PTP packets
 * Call regularly from main loop
 */
void ptp_slave_process(void);

/**
 * Get statistics
 */
void ptp_slave_get_stats(uint32_t *sync_count, uint32_t *announce_count);

#endif // PTP_SLAVE_H

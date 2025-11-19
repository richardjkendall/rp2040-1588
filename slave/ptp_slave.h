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

/**
 * Get PDV rejection count (measurements rejected due to excessive jitter)
 */
uint32_t ptp_slave_get_pdv_rejected(void);

/**
 * Get extended statistics including two-way timing
 */
void ptp_slave_get_timing(int64_t *offset_ns, int64_t *path_delay_ns, int64_t *pdv_ns, bool *offset_valid);

#endif // PTP_SLAVE_H

#ifndef LTSP_ERROR_DECOMP_H
#define LTSP_ERROR_DECOMP_H

#include <stdint.h>
#include <stdbool.h>

/**
 * LTSP Error Decomposition (Section 7 of draft-user-ltsp-rp2040-04)
 *
 * D_total = D_gm_local + D_wire + D_rx_local
 *
 * All values in nanoseconds.
 */

typedef struct {
    int64_t t_rx_ns;            // Receiver PIO timestamp (ns, local crystal domain)
    int64_t t_gps_tx_ns;        // GM TX timestamp (GPS ns, from PDU Prev_Tx_Timestamp)
    int64_t d_total_ns;         // T_rx_ns - T_gps_tx_ns
    int64_t d_gm_local_ns;      // GM local processing delay (from PDU)
    int64_t d_wire_ns;          // Estimated wire propagation delay
    int64_t d_rx_local_ns;      // Receiver local processing delay (approx)
} ltsp_error_decomp_t;

/**
 * Compute error decomposition for one deferred timestamp pair.
 *
 * @param t_pio_rx_ticks  Receiver PIO counter at INTn edge (64-bit extended), in PIO ticks
 * @param t_gps_tx_ns     GM's Prev_Tx_Timestamp (GPS nanoseconds)
 * @param d_gm_local_ns   GM's local processing mean (from PDU field, nanoseconds)
 * @param t_sys_spi_rx_us Receiver's system timer at SPI read (microseconds, for D_rx_local estimate)
 * @param t_pio_rx_ns_approx Approximate nanosecond value of PIO RX (for D_rx_local: T_sys_spi - T_pio_rx)
 * @param result          Output decomposition
 */
void ltsp_error_decompose(int64_t t_pio_rx_ticks,
                           int64_t t_gps_tx_ns,
                           uint32_t d_gm_local_ns,
                           uint64_t t_sys_spi_rx_us,
                           int64_t t_pio_rx_ns_approx,
                           ltsp_error_decomp_t *result);

#endif /* LTSP_ERROR_DECOMP_H */

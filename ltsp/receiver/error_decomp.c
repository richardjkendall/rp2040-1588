#include "error_decomp.h"
#include "../common/ltsp_pio_timestamp.h"

void ltsp_error_decompose(int64_t t_pio_rx_ticks,
                           int64_t t_gps_tx_ns,
                           uint32_t d_gm_local_ns,
                           uint64_t t_sys_spi_rx_us,
                           int64_t t_pio_rx_ns_approx,
                           ltsp_error_decomp_t *result) {
    // Convert receiver PIO ticks to nanoseconds
    result->t_rx_ns = ltsp_pio_ticks_to_ns(t_pio_rx_ticks);
    result->t_gps_tx_ns = t_gps_tx_ns;

    // D_total = T_rx_ns - T_gps_tx_ns
    // Positive means receiver sees frame arrive after GM sent it (expected)
    result->d_total_ns = result->t_rx_ns - t_gps_tx_ns;

    // D_gm_local from PDU
    result->d_gm_local_ns = (int64_t)d_gm_local_ns;

    // D_rx_local approximation: T_sys_spi_rx - T_pio_rx
    // This overestimates because it includes CPU SPI read time
    int64_t t_sys_spi_rx_ns = (int64_t)t_sys_spi_rx_us * 1000;
    result->d_rx_local_ns = t_sys_spi_rx_ns - t_pio_rx_ns_approx;
    if (result->d_rx_local_ns < 0) {
        result->d_rx_local_ns = 0;  // Shouldn't go negative
    }

    // D_wire = D_total - D_gm_local - D_rx_local
    result->d_wire_ns = result->d_total_ns - result->d_gm_local_ns - result->d_rx_local_ns;
}

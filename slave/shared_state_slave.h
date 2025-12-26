/**
 * Shared State for PTP Slave - Inter-Core Communication
 *
 * Core 0: PTP discipline (timing critical)
 * Core 1: W5500 network + PTP protocol
 */

#ifndef SHARED_STATE_SLAVE_H
#define SHARED_STATE_SLAVE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * PTP Sync Data - Core 1 → Core 0
 *
 * Core 1 captures timestamps and passes to Core 0 for discipline
 */
typedef struct {
    // Sync + Follow_Up timestamps
    volatile uint64_t t1_master_ns;       // Master TX time (from Follow_Up)
    volatile uint64_t t2_ptp_ns;          // Slave RX time (from PTP clock)
    volatile uint64_t t2_slave_us;        // Slave RX time (system timer - for reference)
    volatile int64_t correction_sync_ns;  // From Sync/Follow_Up header
    volatile uint32_t counter_at_sync;    // PIO counter at Sync RX (for crystal characterization)
    volatile uint16_t sync_sequence;      // Sequence ID
    volatile bool sync_followup_ready;    // Both Sync+Follow_Up received

    // Hardware timestamp correlation for RX
    volatile int64_t rx_latency_ns;       // RX latency (HW to SW), 0 if no HW timestamp found
    volatile bool rx_hw_timestamp_valid;  // Whether HW timestamp was correlated
    volatile bool rx_used_average;        // Whether average was used instead of actual HW

    // Delay_Req + Delay_Resp timestamps
    volatile uint64_t t3_ptp_ns;          // Slave TX time (from PTP clock)
    volatile uint64_t t3_slave_us;        // Slave TX time (system timer - for reference)
    volatile uint64_t t4_master_ns;       // Master RX time (from Delay_Resp)
    volatile int64_t correction_delay_resp_ns; // From Delay_Resp header
    volatile uint16_t delay_req_sequence; // Sequence ID
    volatile bool delay_resp_ready;       // Delay_Resp received

    // Hardware timestamp correlation for TX
    volatile int64_t tx_latency_ns;       // TX latency (SW to HW), 0 if no HW timestamp found
    volatile bool tx_hw_timestamp_valid;  // Whether HW timestamp was correlated
} ptp_sync_data_t;

/**
 * PTP Discipline Statistics - Core 0 → Core 1
 *
 * Core 0 provides discipline stats for Core 1 to display
 */
typedef struct {
    // Timing discipline
    volatile bool locked;                 // Locked to grandmaster
    volatile int64_t offset_from_master_ns; // Offset from GM (ns)
    volatile int64_t mean_path_delay_ns;  // One-way path delay (ns)
    volatile double freq_offset_ppb;      // Frequency offset (ppb)
    volatile double scale_factor;         // Crystal correction scale

    // Crystal characterization
    volatile int64_t crystal_error_ns;    // Crystal error per second
    volatile double crystal_ppm;          // Crystal error in PPM

    // Counters
    volatile uint32_t sync_count;         // Sync messages processed
    volatile uint32_t discipline_updates; // Discipline updates performed
    volatile uint64_t ptp_time_ns;        // Current PTP time estimate

    // Status
    volatile uint64_t last_update_us;     // Last discipline update time
    volatile bool discipline_running;     // Discipline system active
} ptp_discipline_stats_t;

// Global instances (defined in discipline module)
extern ptp_sync_data_t ptp_sync_data;
extern ptp_discipline_stats_t ptp_stats;

#endif // SHARED_STATE_SLAVE_H

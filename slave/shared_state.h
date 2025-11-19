/**
 * Shared State between Core 0 and Core 1 (PTP Slave)
 *
 * Core 0: Handles WiFi, PTP packet reception
 * Core 1: Disciplines clock to PTP, generates 100 PPS output
 */

#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * PTP sync data received from grandmaster
 * Written by Core 0, read by Core 1
 */
typedef struct {
    volatile bool valid;              // True when new timestamp available
    volatile uint64_t ptp_time_ns;    // PTP time from grandmaster (nanoseconds)
    volatile uint64_t local_time_us;  // Local time when packet received (microseconds)
    volatile uint32_t sequence;       // Sync sequence number

    // Two-way PTP offset (calculated from Delay_Req/Delay_Resp exchange)
    volatile int64_t offset_ns;       // Clock offset from two-way measurement
    volatile bool offset_valid;       // True if offset has been calculated
} ptp_sync_data_t;

extern ptp_sync_data_t ptp_sync_data;

/**
 * Core 1 discipline statistics
 * Written by Core 1, read by Core 0
 */
typedef struct {
    volatile bool discipline_running;
    volatile bool first_sync_received;
    volatile bool locked;              // True when disciplined to PTP

    volatile uint32_t sync_count;      // Number of PTP syncs processed
    volatile int64_t phase_error_ns;   // Phase difference from PTP time (filtered)
    volatile int64_t raw_phase_error_ns;  // Raw phase error before filtering (shows network jitter)
    volatile int32_t freq_offset_ppb;  // Frequency offset (parts per billion)

    volatile uint64_t disciplined_time_ns;  // Current disciplined clock value (0-999ms cycle)
    volatile uint64_t continuous_time_ns;   // Continuous PTP timestamp (never resets)
    volatile uint64_t ptp_estimate_ns;      // Smooth PTP time estimate for packet timestamping (no phase jumps)
    volatile uint64_t last_update_us;       // When last updated

    // Lock/unlock event counters
    volatile uint32_t lock_event_count;
    volatile uint32_t unlock_event_count;
} core1_stats_t;

extern core1_stats_t core1_stats;

#endif // SHARED_STATE_H

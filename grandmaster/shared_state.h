/**
 * Shared State Between Core 0 and Core 1
 *
 * Core 1 updates these variables (timing-critical, no printf)
 * Core 0 reads these variables and prints to serial
 *
 * volatile ensures compiler doesn't optimize away reads/writes
 */

#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include <stdint.h>
#include <stdbool.h>

// Statistics shared from Core 1 to Core 0
typedef struct {
    // Discipline statistics
    volatile uint32_t pps_count;
    volatile int64_t phase_error_ns;
    volatile int32_t freq_offset_ppb;
    volatile bool locked;

    // Status flags
    volatile bool first_pps_received;
    volatile bool discipline_running;

    // Lock event counters (for detecting state changes)
    volatile uint32_t lock_event_count;
    volatile uint32_t unlock_event_count;
} core1_stats_t;

// Global shared state (defined in discipline_core.c)
extern core1_stats_t core1_stats;

#endif // SHARED_STATE_H

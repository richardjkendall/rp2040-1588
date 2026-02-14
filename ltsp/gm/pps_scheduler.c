/**
 * PPS Scheduler - Scheduled 1PPS Generation
 */

#include "pps_scheduler.h"
#include "scheduled_1pps.pio.h"
#include <stdio.h>

// Expected ticks per second at 83.33 MHz (system clock / 3)
#define EXPECTED_TICKS_PER_SECOND 83333333ULL

// Software delay compensation (measured empirically)
// Time between reading time and PIO starting countdown
// Typical: 5-20 microseconds depending on system load
#define SOFTWARE_DELAY_NS 15000

/**
 * Initialize 1PPS scheduler
 */
bool pps_scheduler_init(pps_scheduler_t *sched) {
    if (!sched || !sched->get_time_ns || !sched->get_scale_factor) {
        return false;
    }

    printf("Initializing 1PPS scheduler on GPIO %d...\n", sched->pin);

    // Load PIO program
    uint offset = pio_add_program(sched->pio, &scheduled_1pps_program);
    printf("  PIO program loaded at offset %d\n", offset);

    // Initialize state machine
    scheduled_1pps_program_init(sched->pio, sched->sm, offset, sched->pin);
    printf("  SM%d initialized (83.33 MHz, 12ns per tick)\n", sched->sm);

    // Initialize state
    sched->initialized = true;
    sched->pps_count = 0;
    sched->last_schedule_time_ns = 0;
    sched->last_ticks_scheduled = 0;

    printf("1PPS scheduler ready\n");
    return true;
}

/**
 * Calculate ticks until next second boundary
 */
uint32_t pps_scheduler_calculate_ticks_to_next_second(pps_scheduler_t *sched) {
    // 1. Get current time in nanoseconds
    uint64_t now_ns = sched->get_time_ns();

    // Store for statistics
    sched->last_schedule_time_ns = now_ns;

    // 2. Calculate time within current second (0 to 999,999,999)
    uint64_t ns_into_second = now_ns % 1000000000ULL;

    // 3. Calculate time remaining until next second boundary
    uint64_t ns_until_second = 1000000000ULL - ns_into_second;

    // 4. Compensate for software delay
    // We need to account for the time it takes to:
    // - Return from this function
    // - Push value to PIO FIFO
    // - PIO to start counting
    if (ns_until_second > SOFTWARE_DELAY_NS) {
        ns_until_second -= SOFTWARE_DELAY_NS;
    } else {
        // Too close to boundary, target next second
        ns_until_second += 1000000000ULL - SOFTWARE_DELAY_NS;
    }

    // 5. Convert nanoseconds to ticks with crystal correction
    // Base conversion: 83.33 MHz = 12ns per tick
    // Apply scale_factor to correct for crystal drift
    double scale_factor = sched->get_scale_factor();

    // Ticks = ns / (12 ns/tick) / scale_factor
    // scale_factor corrects for crystal running fast/slow
    double ticks_precise = ((double)ns_until_second / 12.0) / scale_factor;

    // 6. Round to nearest tick (±6ns uncertainty)
    uint32_t ticks = (uint32_t)(ticks_precise + 0.5);

    // Sanity check: valid range is 0 to ~166M ticks (0 to 2 seconds)
    // We call scheduler every 800ms, so we can be at any point in the second
    // Only warn on truly unusual values (> 2 seconds or clearly invalid)
    if (ticks > 166000000) {
        printf("WARNING: Unusual tick count: %lu (sf=%.6f, ns=%llu)\n",
               ticks, scale_factor, ns_until_second);
    }

    sched->last_ticks_scheduled = ticks;
    return ticks;
}

/**
 * Schedule next 1PPS pulse
 */
void pps_scheduler_schedule_next(pps_scheduler_t *sched) {
    if (!sched->initialized) {
        printf("ERROR: Scheduler not initialized\n");
        return;
    }

    // Calculate ticks to next second
    uint32_t ticks = pps_scheduler_calculate_ticks_to_next_second(sched);

    // Push to PIO FIFO (blocking if FIFO full, but it should always be empty)
    pio_sm_put_blocking(sched->pio, sched->sm, ticks);

    // Increment pulse count
    sched->pps_count++;

    // Debug output for first few pulses
    if (sched->pps_count <= 5) {
        printf("1PPS #%lu scheduled: %lu ticks (%.6f ms, sf=%.6f)\n",
               sched->pps_count, ticks,
               (double)ticks * 12.0 / 1000000.0,
               sched->get_scale_factor());
    }
}

/**
 * Get scheduler statistics
 */
void pps_scheduler_get_stats(pps_scheduler_t *sched,
                             uint32_t *pps_count,
                             uint64_t *last_schedule_time_ns) {
    if (pps_count) {
        *pps_count = sched->pps_count;
    }
    if (last_schedule_time_ns) {
        *last_schedule_time_ns = sched->last_schedule_time_ns;
    }
}

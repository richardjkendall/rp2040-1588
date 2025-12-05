/**
 * PPS Scheduler - Scheduled 1PPS Generation
 *
 * Generates a 1 pulse-per-second output aligned to an absolute time reference
 * (GPS time on GM, PTP time on Slave) by scheduling PIO to count down to the
 * next second boundary.
 *
 * This allows external measurement of timing accuracy by comparing 1PPS signals
 * from multiple devices.
 */

#ifndef PPS_SCHEDULER_H
#define PPS_SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

/**
 * PPS Scheduler Configuration
 */
typedef struct {
    PIO pio;                         // PIO instance to use
    uint sm;                          // State machine number
    uint pin;                         // GPIO pin for 1PPS output
    uint64_t (*get_time_ns)(void);   // Function to get current time in nanoseconds
    double (*get_scale_factor)(void); // Function to get crystal scale factor

    // Internal state
    bool initialized;
    uint32_t pps_count;              // Number of pulses generated
    uint64_t last_schedule_time_ns;  // Last time we scheduled a pulse
    uint32_t last_ticks_scheduled;   // Last tick count scheduled
} pps_scheduler_t;

/**
 * Initialize 1PPS scheduler
 *
 * @param sched Scheduler configuration
 * @return true on success, false on failure
 */
bool pps_scheduler_init(pps_scheduler_t *sched);

/**
 * Calculate ticks until next second boundary
 *
 * This function:
 * 1. Gets current time from time source
 * 2. Calculates time remaining in current second
 * 3. Compensates for software delay
 * 4. Converts to PIO ticks with crystal correction
 *
 * @param sched Scheduler state
 * @return Number of ticks to countdown (12ns per tick @ 83.33MHz)
 */
uint32_t pps_scheduler_calculate_ticks_to_next_second(pps_scheduler_t *sched);

/**
 * Schedule next 1PPS pulse
 *
 * Calculates and loads the tick count for the next second boundary.
 * Should be called approximately once per second.
 *
 * @param sched Scheduler state
 */
void pps_scheduler_schedule_next(pps_scheduler_t *sched);

/**
 * Get scheduler statistics
 *
 * @param sched Scheduler state
 * @param pps_count Output: Number of pulses generated
 * @param last_schedule_time_ns Output: Time of last schedule
 */
void pps_scheduler_get_stats(pps_scheduler_t *sched,
                             uint32_t *pps_count,
                             uint64_t *last_schedule_time_ns);

#endif // PPS_SCHEDULER_H

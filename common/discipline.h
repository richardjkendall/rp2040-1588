/**
 * Clock Discipline Algorithm
 *
 * Implements a PI (Proportional-Integral) controller for disciplining
 * a local clock to an external reference (GPS PPS).
 *
 * The disciplined clock maintains a 64-bit nanosecond counter that is
 * continuously corrected based on phase error measurements.
 *
 * IMPORTANT: No printf() in this module - it's called from Core 1.
 * Use event counters in shared state for status changes.
 */

#ifndef DISCIPLINE_H
#define DISCIPLINE_H

#include <stdint.h>
#include <stdbool.h>

// Disciplined clock state
typedef struct {
    uint64_t nanoseconds;           // Current disciplined time (ns since epoch)
    int64_t phase_error_ns;         // Last measured phase error (ns)
    int32_t frequency_offset_ppb;   // Frequency correction (parts per billion)
    uint64_t last_update_time_us;   // Hardware timer value at last update
    uint64_t last_reference_time_us; // Hardware timer value at last reference pulse
    bool locked;                    // Are we locked to reference?

    // PI controller state
    double integral;                // Integral term accumulator
    uint32_t lock_counter;          // Consecutive good measurements

    // PI controller gains (tunable)
    double kp;                      // Proportional gain
    double ki;                      // Integral gain
} disciplined_clock_t;

/**
 * Initialize disciplined clock
 *
 * @param clock Pointer to clock structure
 * @param kp Proportional gain (default: 0.1)
 * @param ki Integral gain (default: 0.001)
 */
void discipline_init(disciplined_clock_t *clock, double kp, double ki);

/**
 * Update disciplined clock based on reference pulse
 *
 * Call this when a reference pulse (e.g., GPS PPS) occurs.
 *
 * @param clock Pointer to clock structure
 * @param reference_time_us Hardware timer value when reference pulse occurred
 * @param expected_interval_ns Expected time between pulses (e.g., 1000000000 for 1 Hz)
 * @param lock_event_counter Pointer to counter to increment on lock events (can be NULL)
 * @param unlock_event_counter Pointer to counter to increment on unlock events (can be NULL)
 * @return Phase error in nanoseconds
 */
int64_t discipline_update_reference(disciplined_clock_t *clock,
                                    uint64_t reference_time_us,
                                    uint64_t expected_interval_ns,
                                    volatile uint32_t *lock_event_counter,
                                    volatile uint32_t *unlock_event_counter);

/**
 * Update the disciplined clock time based on elapsed hardware time
 *
 * Call this regularly (e.g., every loop iteration) to advance the
 * disciplined clock with frequency correction applied.
 *
 * @param clock Pointer to clock structure
 */
void discipline_update_time(disciplined_clock_t *clock);

/**
 * Get current disciplined time
 *
 * @param clock Pointer to clock structure
 * @return Current time in nanoseconds
 */
static inline uint64_t discipline_get_time_ns(const disciplined_clock_t *clock) {
    return clock->nanoseconds;
}

/**
 * Check if clock is locked to reference
 *
 * @param clock Pointer to clock structure
 * @return true if locked, false otherwise
 */
static inline bool discipline_is_locked(const disciplined_clock_t *clock) {
    return clock->locked;
}

/**
 * Get current phase error
 *
 * @param clock Pointer to clock structure
 * @return Phase error in nanoseconds
 */
static inline int64_t discipline_get_phase_error_ns(const disciplined_clock_t *clock) {
    return clock->phase_error_ns;
}

/**
 * Get current frequency offset
 *
 * @param clock Pointer to clock structure
 * @return Frequency offset in parts per billion
 */
static inline int32_t discipline_get_frequency_offset_ppb(const disciplined_clock_t *clock) {
    return clock->frequency_offset_ppb;
}

#endif // DISCIPLINE_H

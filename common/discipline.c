/**
 * Clock Discipline Algorithm Implementation
 *
 * IMPORTANT: No printf() in this file - it's called from Core 1.
 */

#include "discipline.h"
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include <math.h>

// Lock thresholds
#define LOCK_THRESHOLD_NS 1000000     // 1ms - consider locked if within this
#define LOCK_SAMPLES_REQUIRED 5        // Consecutive samples needed for lock
#define UNLOCK_THRESHOLD_NS 10000000   // 10ms - lose lock if error exceeds this

void discipline_init(disciplined_clock_t *clock, double kp, double ki) {
    clock->nanoseconds = 0;
    clock->phase_error_ns = 0;
    clock->frequency_offset_ppb = 0;
    clock->last_update_time_us = time_us_64();
    clock->last_reference_time_us = 0;
    clock->locked = false;
    clock->integral = 0.0;
    clock->lock_counter = 0;
    clock->kp = kp;
    clock->ki = ki;
}

int64_t discipline_update_reference(disciplined_clock_t *clock,
                                    uint64_t reference_time_us,
                                    uint64_t expected_interval_ns,
                                    volatile uint32_t *lock_event_counter,
                                    volatile uint32_t *unlock_event_counter) {
    // First reference pulse - initialize
    if (clock->last_reference_time_us == 0) {
        clock->last_reference_time_us = reference_time_us;
        clock->last_update_time_us = reference_time_us;
        clock->nanoseconds = 0; // Start counting from zero
        return 0;
    }

    // Update our disciplined clock to the current moment (when GPS PPS arrived)
    discipline_update_time(clock);

    // Phase error: where is our disciplined clock vs. where should it be?
    // - Clock started at 0 when last GPS PPS arrived
    // - Clock should be at exactly expected_interval_ns (1,000,000,000) when this GPS PPS arrives
    // - Clock is actually at clock->nanoseconds
    // - If clock > expected: we're running FAST (positive error)
    // - If clock < expected: we're running SLOW (negative error)
    int64_t phase_error = (int64_t)clock->nanoseconds - (int64_t)expected_interval_ns;
    clock->phase_error_ns = phase_error;

    // PI Controller
    double error_seconds = (double)phase_error / 1e9;

    // Proportional term
    double p_term = clock->kp * error_seconds;

    // Integral term (with anti-windup)
    clock->integral += error_seconds;
    // Clamp integral to prevent windup (±100ms accumulated error)
    if (clock->integral > 0.1) clock->integral = 0.1;
    if (clock->integral < -0.1) clock->integral = -0.1;
    double i_term = clock->ki * clock->integral;

    // Calculate frequency offset correction (in ppb)
    // IMPORTANT: Negate because positive phase error (running fast) needs negative correction (slow down)
    double correction = -(p_term + i_term);
    clock->frequency_offset_ppb = (int32_t)(correction * 1e9);

    // Clamp frequency offset to reasonable range (±1000 ppm = ±1,000,000 ppb)
    if (clock->frequency_offset_ppb > 1000000) clock->frequency_offset_ppb = 1000000;
    if (clock->frequency_offset_ppb < -1000000) clock->frequency_offset_ppb = -1000000;

    // Update lock status
    int64_t abs_error = phase_error < 0 ? -phase_error : phase_error;
    if (abs_error < LOCK_THRESHOLD_NS) {
        clock->lock_counter++;
        if (clock->lock_counter >= LOCK_SAMPLES_REQUIRED) {
            if (!clock->locked) {
                // Transition to locked state
                if (lock_event_counter) {
                    (*lock_event_counter)++;
                }
            }
            clock->locked = true;
        }
    } else {
        if (abs_error > UNLOCK_THRESHOLD_NS) {
            if (clock->locked) {
                // Transition to unlocked state
                if (unlock_event_counter) {
                    (*unlock_event_counter)++;
                }
            }
            clock->locked = false;
        }
        clock->lock_counter = 0;
    }

    // Reset our nanosecond counter to zero at each reference pulse
    // This allows the clock to count from 0 to expected_interval_ns repeatedly
    clock->nanoseconds = 0;
    clock->last_reference_time_us = reference_time_us;
    clock->last_update_time_us = reference_time_us;

    return phase_error;
}

void discipline_update_time(disciplined_clock_t *clock) {
    uint64_t now_us = time_us_64();
    uint64_t elapsed_us = now_us - clock->last_update_time_us;

    // Convert elapsed microseconds to nanoseconds
    uint64_t elapsed_ns = elapsed_us * 1000ULL;

    // Apply frequency correction
    // corrected_time = elapsed_ns * (1 + frequency_offset/1e9)
    int64_t correction_ns = ((int64_t)elapsed_ns * (int64_t)clock->frequency_offset_ppb) / 1000000000LL;
    int64_t corrected_elapsed_ns = (int64_t)elapsed_ns + correction_ns;

    // Update clock
    if (corrected_elapsed_ns > 0) {
        clock->nanoseconds += (uint64_t)corrected_elapsed_ns;
    }

    clock->last_update_time_us = now_us;
}

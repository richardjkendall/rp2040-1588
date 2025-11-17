/**
 * Slave Discipline Core (Core 1)
 * Disciplines clock to PTP timestamps and generates 100 PPS output
 */

#include "shared_state.h"
#include "discipline.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"

// 100 PPS output pin (same as grandmaster for scope comparison)
#define PPS_OUTPUT_PIN 3

// Discipline parameters
#define LOCK_THRESHOLD_NS 1000000     // 1ms - consider locked if error < 1ms
#define UNLOCK_THRESHOLD_NS 10000000  // 10ms - lose lock if error > 10ms
#define LOCK_HYSTERESIS_COUNT 5       // Must be stable for 5 samples to lock

// 100 PPS output timing
#define OUTPUT_PPS_INTERVAL_NS 10000000ULL  // 10ms between pulses

// Shared state
ptp_sync_data_t ptp_sync_data = {0};
core1_stats_t core1_stats = {0};

// Local state
static disciplined_clock_t disciplined_clock;
static uint32_t lock_stable_count = 0;
static uint64_t last_ptp_time_ns = 0;
static uint64_t next_output_pulse_ns = 0;
static bool first_sync = true;
static uint64_t ptp_seconds = 0;  // Track seconds portion of PTP time

/**
 * Core 1 entry point
 */
void core1_entry(void) {
    // Initialize PPS output pin
    gpio_init(PPS_OUTPUT_PIN);
    gpio_set_dir(PPS_OUTPUT_PIN, GPIO_OUT);
    gpio_put(PPS_OUTPUT_PIN, 0);

    // Initialize discipline algorithm with PI gains
    // Use same gains as GPS discipline: kp=0.1, ki=0.001
    discipline_init(&disciplined_clock, 0.1, 0.001);

    core1_stats.discipline_running = true;

    uint32_t last_processed_sequence = 0;
    uint64_t last_ptp_rx_time_us = 0;
    uint64_t last_ptp_timestamp_ns = 0;

    while (true) {
        // Update clock based on frequency offset (runs continuously)
        discipline_update_time(&disciplined_clock);

        // Handle second rollover
        if (disciplined_clock.nanoseconds >= 1000000000ULL) {
            ptp_seconds++;
            disciplined_clock.nanoseconds -= 1000000000ULL;
        }

        // Check for new PTP timestamp from Core 0
        // Read sequence first to avoid race condition
        uint32_t current_sequence = ptp_sync_data.sequence;
        bool data_valid = ptp_sync_data.valid;

        if (data_valid && current_sequence != last_processed_sequence) {
            // New PTP timestamp available - read data atomically
            uint64_t ptp_time_ns = ptp_sync_data.ptp_time_ns;
            uint64_t rx_time_us = ptp_sync_data.local_time_us;

            // Update sequence BEFORE clearing valid to prevent reprocessing
            last_processed_sequence = current_sequence;

            // Memory barrier to ensure sequence update completes before clearing valid
            __compiler_memory_barrier();

            // Clear valid flag to signal Core 0 that we've consumed this data
            ptp_sync_data.valid = false;

            if (!core1_stats.first_sync_received) {
                // First sync - initialize our clock to match PTP time
                uint64_t now_us = time_us_64();
                uint64_t elapsed_since_rx_us = now_us - rx_time_us;

                // Estimate current PTP time
                uint64_t estimated_ptp_now_ns = ptp_time_ns + (elapsed_since_rx_us * 1000);

                // Split into seconds and nanoseconds (like grandmaster does)
                ptp_seconds = estimated_ptp_now_ns / 1000000000ULL;
                disciplined_clock.nanoseconds = estimated_ptp_now_ns % 1000000000ULL;
                disciplined_clock.last_update_time_us = now_us;
                disciplined_clock.last_reference_time_us = now_us;
                core1_stats.first_sync_received = true;

                // Initialize output pulse timing (use nanoseconds portion)
                next_output_pulse_ns = disciplined_clock.nanoseconds;
                // Round to next 10ms boundary
                next_output_pulse_ns = ((next_output_pulse_ns / OUTPUT_PPS_INTERVAL_NS) + 1) * OUTPUT_PPS_INTERVAL_NS;
                first_sync = false;

                last_ptp_rx_time_us = rx_time_us;
                last_ptp_timestamp_ns = ptp_time_ns;
            } else {
                // Subsequent syncs - discipline our clock to PTP time
                uint64_t now_us = time_us_64();
                uint64_t elapsed_since_rx_us = now_us - rx_time_us;

                // Calculate estimated current PTP time (continuous)
                uint64_t estimated_ptp_now_ns = ptp_time_ns + (elapsed_since_rx_us * 1000);

                // Calculate our continuous clock time (seconds + nanoseconds)
                uint64_t our_continuous_time_ns = (ptp_seconds * 1000000000ULL) + disciplined_clock.nanoseconds;

                // Calculate phase error using continuous timestamps
                // Positive error = we're behind master (master time > our time)
                // Negative error = we're ahead of master (master time < our time)
                int64_t phase_error_ns = (int64_t)estimated_ptp_now_ns - (int64_t)our_continuous_time_ns;

                // For large errors (>100ms), make a phase jump to get close quickly
                // For small errors, use PI controller to fine-tune frequency
                int64_t abs_error = (phase_error_ns < 0) ? -phase_error_ns : phase_error_ns;

                if (abs_error > 100000000LL) {
                    // Phase error > 100ms - do a phase jump
                    ptp_seconds = estimated_ptp_now_ns / 1000000000ULL;
                    disciplined_clock.nanoseconds = estimated_ptp_now_ns % 1000000000ULL;
                    disciplined_clock.last_update_time_us = now_us;
                    disciplined_clock.integral = 0;  // Reset integral on phase jump
                    lock_stable_count = 0;
                } else {
                    // Phase error < 100ms - use PI controller for smooth discipline
                    double error_sec = (double)phase_error_ns / 1e9;
                    double p_term = disciplined_clock.kp * error_sec;

                    disciplined_clock.integral += error_sec;

                    // Anti-windup (limit to ±0.1 seconds accumulated)
                    if (disciplined_clock.integral > 0.1) disciplined_clock.integral = 0.1;
                    if (disciplined_clock.integral < -0.1) disciplined_clock.integral = -0.1;

                    double i_term = disciplined_clock.ki * disciplined_clock.integral;

                    // Calculate frequency correction in ppb
                    // Negative because positive error (we're behind) needs positive correction (speed up)
                    // Wait, let's think: if phase_error > 0, we're behind, so we need to speed up (positive freq offset)
                    disciplined_clock.frequency_offset_ppb = (int32_t)((p_term + i_term) * 1e9);

                    // Limit to reasonable range (±50 ppm)
                    if (disciplined_clock.frequency_offset_ppb > 50000)
                        disciplined_clock.frequency_offset_ppb = 50000;
                    if (disciplined_clock.frequency_offset_ppb < -50000)
                        disciplined_clock.frequency_offset_ppb = -50000;
                }

                // Lock detection based on phase error
                if (abs_error < LOCK_THRESHOLD_NS) {
                    lock_stable_count++;
                    if (lock_stable_count >= LOCK_HYSTERESIS_COUNT && !disciplined_clock.locked) {
                        disciplined_clock.locked = true;
                        core1_stats.lock_event_count++;
                    }
                } else if (abs_error > UNLOCK_THRESHOLD_NS) {
                    lock_stable_count = 0;
                    if (disciplined_clock.locked) {
                        disciplined_clock.locked = false;
                        core1_stats.unlock_event_count++;
                    }
                }

                core1_stats.phase_error_ns = phase_error_ns;
                core1_stats.freq_offset_ppb = disciplined_clock.frequency_offset_ppb;
                core1_stats.locked = disciplined_clock.locked;
                core1_stats.sync_count++;

                last_ptp_rx_time_us = rx_time_us;
                last_ptp_timestamp_ns = ptp_time_ns;
            }
        }

        // Publish disciplined time for monitoring
        core1_stats.disciplined_time_ns = disciplined_clock.nanoseconds;
        core1_stats.last_update_us = time_us_64();

        // Generate 100 PPS output when disciplined clock crosses thresholds
        if (!first_sync) {
            uint64_t current_ns = discipline_get_time_ns(&disciplined_clock);

            if (current_ns >= next_output_pulse_ns) {
                // Generate output pulse (short pulse for precision)
                gpio_put(PPS_OUTPUT_PIN, 1);
                busy_wait_us(10);  // 10μs pulse - short enough for minimal blocking
                gpio_put(PPS_OUTPUT_PIN, 0);

                // Schedule next pulse (10ms later for 100 PPS)
                next_output_pulse_ns += OUTPUT_PPS_INTERVAL_NS;
            }
        }

        // Minimal delay to prevent busy-wait hogging
        tight_loop_contents();
    }
}

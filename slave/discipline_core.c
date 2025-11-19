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

// Discipline parameters for WiFi PTP
// WiFi has inherent jitter of 1-10ms, so thresholds must be realistic
#define LOCK_THRESHOLD_NS 5000000     // 5ms - consider locked if error < 5ms (realistic for WiFi)
#define UNLOCK_THRESHOLD_NS 20000000  // 20ms - lose lock if error > 20ms
#define LOCK_HYSTERESIS_COUNT 10      // Must be stable for 10 samples to lock (more stringent)

// Phase adjustment gain (5% of phase error applied per sync)
// This allows fast convergence: 100ms error → <10ms in ~20 syncs (~20 seconds)
#define PHASE_ADJUSTMENT_GAIN 0.05

// Jitter filter coefficient for exponential moving average
// Lower value = more filtering (slower response but smoother)
// 0.1 means: 10% new sample, 90% previous filtered value (more filtering for WiFi)
#define JITTER_FILTER_ALPHA 0.1

// 100 PPS output timing
#define OUTPUT_PPS_INTERVAL_NS 10000000ULL  // 10ms between pulses

// Shared state
ptp_sync_data_t ptp_sync_data = {0};
core1_stats_t core1_stats = {0};

// Local state
static disciplined_clock_t disciplined_clock;  // Fast-converging clock for PPS output (can have phase jumps)
static disciplined_clock_t ptp_estimate_clock;  // Smooth PTP estimate for timestamps (frequency-only, no phase jumps)
static uint32_t lock_stable_count = 0;
static uint64_t last_ptp_time_ns = 0;
static uint64_t next_output_pulse_ns = 0;
static bool first_sync = true;
static uint64_t ptp_seconds = 0;  // Track seconds portion of PTP time (for output clock)
static uint64_t ptp_estimate_seconds = 0;  // Track seconds portion of PTP estimate (for timestamping)
static int64_t filtered_phase_error_ns = 0;  // Filtered phase error for jitter reduction

/**
 * Core 1 entry point
 */
void core1_entry(void) {
    // Initialize PPS output pin
    gpio_init(PPS_OUTPUT_PIN);
    gpio_set_dir(PPS_OUTPUT_PIN, GPIO_OUT);
    gpio_put(PPS_OUTPUT_PIN, 0);

    // Initialize output clock (can use phase jumps for fast PPS alignment)
    // kp=0.001: 10ms residual error → 10ppm frequency adjustment
    // ki=0.00001: very slow integral buildup for long-term stability
    discipline_init(&disciplined_clock, 0.001, 0.00001);

    // Initialize PTP estimate clock (frequency-only, for stable packet timestamps)
    // Slightly more aggressive since it has no phase corrections to help
    // kp=0.01: 10ms residual error → 100ppm frequency adjustment
    // ki=0.0001: faster integral for convergence
    discipline_init(&ptp_estimate_clock, 0.01, 0.0001);

    core1_stats.discipline_running = true;

    uint32_t last_processed_sequence = 0;
    uint64_t last_ptp_rx_time_us = 0;
    uint64_t last_ptp_timestamp_ns = 0;

    while (true) {
        // Update our local clock based on elapsed time and frequency offset
        uint64_t now_us = time_us_64();
        uint64_t elapsed_us = now_us - disciplined_clock.last_update_time_us;

        if (elapsed_us > 0) {
            // Convert elapsed time to nanoseconds
            uint64_t elapsed_ns = elapsed_us * 1000ULL;

            // Apply frequency correction
            // corrected_time = elapsed_ns * (1 + frequency_offset/1e9)
            int64_t correction_ns = ((int64_t)elapsed_ns * (int64_t)disciplined_clock.frequency_offset_ppb) / 1000000000LL;
            int64_t corrected_elapsed_ns = (int64_t)elapsed_ns + correction_ns;

            if (corrected_elapsed_ns > 0) {
                disciplined_clock.nanoseconds += (uint64_t)corrected_elapsed_ns;

                // Handle second rollover
                while (disciplined_clock.nanoseconds >= 1000000000ULL) {
                    ptp_seconds++;
                    disciplined_clock.nanoseconds -= 1000000000ULL;
                }
            }

            disciplined_clock.last_update_time_us = now_us;
        }

        // Update PTP estimate clock (frequency-only, for stable timestamping)
        uint64_t estimate_elapsed_us = now_us - ptp_estimate_clock.last_update_time_us;
        if (estimate_elapsed_us > 0) {
            uint64_t elapsed_ns = estimate_elapsed_us * 1000ULL;

            // Apply frequency correction (same formula as output clock)
            int64_t correction_ns = ((int64_t)elapsed_ns * (int64_t)ptp_estimate_clock.frequency_offset_ppb) / 1000000000LL;
            int64_t corrected_elapsed_ns = (int64_t)elapsed_ns + correction_ns;

            if (corrected_elapsed_ns > 0) {
                ptp_estimate_clock.nanoseconds += (uint64_t)corrected_elapsed_ns;

                // Handle second rollover
                while (ptp_estimate_clock.nanoseconds >= 1000000000ULL) {
                    ptp_estimate_seconds++;
                    ptp_estimate_clock.nanoseconds -= 1000000000ULL;
                }
            }

            ptp_estimate_clock.last_update_time_us = now_us;
        }

        // Check for new PTP timestamp from Core 0
        // Read sequence first to avoid race condition
        uint32_t current_sequence = ptp_sync_data.sequence;
        bool data_valid = ptp_sync_data.valid;

        if (data_valid && current_sequence != last_processed_sequence) {
            // New PTP timestamp available - read data atomically
            uint64_t ptp_time_ns = ptp_sync_data.ptp_time_ns;
            uint64_t rx_time_us = ptp_sync_data.local_time_us;
            int64_t offset_ns = ptp_sync_data.offset_ns;
            bool offset_valid = ptp_sync_data.offset_valid;

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

                // Initialize PTP estimate clock (same initial time, but will diverge)
                ptp_estimate_seconds = estimated_ptp_now_ns / 1000000000ULL;
                ptp_estimate_clock.nanoseconds = estimated_ptp_now_ns % 1000000000ULL;
                ptp_estimate_clock.last_update_time_us = now_us;
                ptp_estimate_clock.last_reference_time_us = now_us;

                core1_stats.first_sync_received = true;

                // Initialize output pulse timing using continuous PTP estimate
                uint64_t continuous_ptp_ns = (ptp_estimate_seconds * 1000000000ULL) + ptp_estimate_clock.nanoseconds;
                // Round to next 10ms boundary
                next_output_pulse_ns = ((continuous_ptp_ns / OUTPUT_PPS_INTERVAL_NS) + 1) * OUTPUT_PPS_INTERVAL_NS;
                first_sync = false;

                last_ptp_rx_time_us = rx_time_us;
                last_ptp_timestamp_ns = ptp_time_ns;
            } else {
                // Subsequent syncs - discipline our clock to PTP time
                uint64_t now_us = time_us_64();
                uint64_t elapsed_since_rx_us = now_us - rx_time_us;

                // Determine phase error using one-way PTP only
                // (Two-way offset disabled - causes circular dependency with disciplined clock)
                uint64_t estimated_ptp_now_ns = ptp_time_ns + (elapsed_since_rx_us * 1000);
                uint64_t our_continuous_time_ns = (ptp_seconds * 1000000000ULL) + disciplined_clock.nanoseconds;
                int64_t raw_phase_error_ns = (int64_t)estimated_ptp_now_ns - (int64_t)our_continuous_time_ns;

                // === JITTER FILTERING ===
                // Apply exponential moving average to reduce network jitter
                // This prevents the discipline loop from chasing WiFi packet delay variations
                if (core1_stats.sync_count == 0) {
                    // First sync - initialize filter with first measurement
                    filtered_phase_error_ns = raw_phase_error_ns;
                } else {
                    // Exponential moving average: filtered = alpha*new + (1-alpha)*old
                    filtered_phase_error_ns = (int64_t)(
                        JITTER_FILTER_ALPHA * raw_phase_error_ns +
                        (1.0 - JITTER_FILTER_ALPHA) * filtered_phase_error_ns
                    );
                }

                // Use filtered error for discipline (smooth), report raw error for monitoring (shows jitter)
                int64_t phase_error_ns = filtered_phase_error_ns;

                // === PHASE DISCIPLINE ===
                // Apply a gradual phase adjustment (5% of error per sync)
                int64_t phase_adjustment_ns = (int64_t)(phase_error_ns * PHASE_ADJUSTMENT_GAIN);

                // Apply phase adjustment to our clock
                int64_t new_nanoseconds = (int64_t)disciplined_clock.nanoseconds + phase_adjustment_ns;

                // Handle rollovers when adjusting phase
                while (new_nanoseconds >= 1000000000LL) {
                    ptp_seconds++;
                    new_nanoseconds -= 1000000000LL;
                }
                while (new_nanoseconds < 0) {
                    if (ptp_seconds > 0) {
                        ptp_seconds--;
                        new_nanoseconds += 1000000000LL;
                    } else {
                        new_nanoseconds = 0;
                        break;
                    }
                }

                disciplined_clock.nanoseconds = (uint64_t)new_nanoseconds;

                // === FREQUENCY DISCIPLINE ===
                // Use PI controller to adjust frequency based on remaining phase error
                // The phase adjustment above handles the bulk of the error quickly
                // The PI controller fine-tunes frequency for long-term stability
                double error_sec = (double)phase_error_ns / 1e9;
                double p_term = disciplined_clock.kp * error_sec;

                disciplined_clock.integral += error_sec;

                // Anti-windup (limit to ±0.1 seconds accumulated)
                if (disciplined_clock.integral > 0.1) disciplined_clock.integral = 0.1;
                if (disciplined_clock.integral < -0.1) disciplined_clock.integral = -0.1;

                double i_term = disciplined_clock.ki * disciplined_clock.integral;

                // Calculate frequency correction in ppb
                // Positive error (behind) → positive correction (speed up)
                // Negative error (ahead) → negative correction (slow down)
                disciplined_clock.frequency_offset_ppb = (int32_t)((p_term + i_term) * 1e9);

                // Limit to reasonable range (±50 ppm for typical crystal tolerance)
                if (disciplined_clock.frequency_offset_ppb > 50000)
                    disciplined_clock.frequency_offset_ppb = 50000;
                if (disciplined_clock.frequency_offset_ppb < -50000)
                    disciplined_clock.frequency_offset_ppb = -50000;

                // === PTP ESTIMATE CLOCK DISCIPLINE (FREQUENCY-ONLY) ===
                // This clock is used for packet timestamping (t2/t3)
                // NO phase jumps - frequency adjustments only for stable timestamps
                // Uses same phase error but more aggressive gains since no phase help
                double estimate_error_sec = (double)phase_error_ns / 1e9;
                double estimate_p_term = ptp_estimate_clock.kp * estimate_error_sec;

                ptp_estimate_clock.integral += estimate_error_sec;

                // Anti-windup (limit to ±1.0 seconds accumulated - more tolerance for freq-only)
                if (ptp_estimate_clock.integral > 1.0) ptp_estimate_clock.integral = 1.0;
                if (ptp_estimate_clock.integral < -1.0) ptp_estimate_clock.integral = -1.0;

                double estimate_i_term = ptp_estimate_clock.ki * ptp_estimate_clock.integral;

                // Calculate frequency correction
                ptp_estimate_clock.frequency_offset_ppb = (int32_t)((estimate_p_term + estimate_i_term) * 1e9);

                // Limit to reasonable range
                if (ptp_estimate_clock.frequency_offset_ppb > 50000)
                    ptp_estimate_clock.frequency_offset_ppb = 50000;
                if (ptp_estimate_clock.frequency_offset_ppb < -50000)
                    ptp_estimate_clock.frequency_offset_ppb = -50000;

                // Lock detection based on FILTERED phase error
                int64_t abs_error = (phase_error_ns < 0) ? -phase_error_ns : phase_error_ns;
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

                // Report both raw and filtered errors for diagnostics
                core1_stats.phase_error_ns = phase_error_ns;  // Filtered (what we discipline to)
                core1_stats.raw_phase_error_ns = raw_phase_error_ns;  // Raw (shows network jitter)
                core1_stats.freq_offset_ppb = disciplined_clock.frequency_offset_ppb;
                core1_stats.locked = disciplined_clock.locked;
                core1_stats.sync_count++;

                last_ptp_rx_time_us = rx_time_us;
                last_ptp_timestamp_ns = ptp_time_ns;
            }
        }

        // Publish disciplined time for monitoring
        core1_stats.disciplined_time_ns = disciplined_clock.nanoseconds;
        core1_stats.continuous_time_ns = (ptp_seconds * 1000000000ULL) + disciplined_clock.nanoseconds;
        core1_stats.ptp_estimate_ns = (ptp_estimate_seconds * 1000000000ULL) + ptp_estimate_clock.nanoseconds;
        core1_stats.last_update_us = time_us_64();

        // Generate 100 PPS output using continuous PTP estimate (frequency-only, no phase jumps)
        // Continuous timestamp never wraps, so no special handling needed
        if (!first_sync) {
            // Use continuous PTP estimate timestamp (always increasing, never wraps)
            uint64_t current_continuous_ns = (ptp_estimate_seconds * 1000000000ULL) + ptp_estimate_clock.nanoseconds;

            if (current_continuous_ns >= next_output_pulse_ns) {
                // Generate output pulse (short pulse for precision)
                gpio_put(PPS_OUTPUT_PIN, 1);
                busy_wait_us(10);  // 10μs pulse - short enough for minimal blocking
                gpio_put(PPS_OUTPUT_PIN, 0);

                // Schedule next pulse (10ms later for 100 PPS)
                // No wrapping needed - continuous timestamp keeps increasing
                next_output_pulse_ns += OUTPUT_PPS_INTERVAL_NS;
            }
        }

        // Minimal delay to prevent busy-wait hogging
        tight_loop_contents();
    }
}

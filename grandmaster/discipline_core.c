/**
 * Clock Discipline Core (Core 1)
 *
 * This runs on Core 1, isolated from network stack and other processing.
 * Maintains a GPS-disciplined clock and generates 1 PPS output.
 *
 * IMPORTANT: NO printf() calls in this file!
 * All output is via shared variables read by Core 0.
 */

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "gps.h"
#include "shared_state.h"
#include "../common/discipline.h"

// External pin definitions (from main.c)
extern const uint LED_OUTPUT_PIN;
#define LED_OUTPUT_PIN 3

// GPS PPS interval (1 Hz = 1,000,000,000 ns)
#define GPS_PPS_INTERVAL_NS 1000000000ULL

// 100 PPS output interval (10 ms = 10,000,000 ns) - Phase 2a
#define OUTPUT_PPS_INTERVAL_NS 10000000ULL

// Disciplined clock instance
static disciplined_clock_t disciplined_clock;

// Shared state for Core 0 to read (defined globally)
core1_stats_t core1_stats = {0};

void core1_entry() {
    // Signal to Core 0 that we're running
    core1_stats.discipline_running = true;

    // Initialize disciplined clock with PI controller gains
    // kp = 0.1, ki = 0.001 (starting values, can be tuned)
    discipline_init(&disciplined_clock, 0.1, 0.001);

    uint64_t next_output_pulse_ns = 0;
    bool first_pulse = true;
    uint64_t ptp_seconds = 0;  // Continuous PTP timestamp (seconds portion)

    // LED blink counter for testing
    uint32_t led_counter = 0;

    while (true) {
        // Update disciplined clock time (apply frequency correction)
        discipline_update_time(&disciplined_clock);

        // Publish disciplined clock for PTP timestamping (Phase 2b)
        core1_stats.disciplined_time_ns = disciplined_clock.nanoseconds;

        // Continuous PTP timestamp = seconds * 1e9 + nanoseconds
        core1_stats.continuous_time_ns = (ptp_seconds * 1000000000ULL) + disciplined_clock.nanoseconds;
        core1_stats.last_update_us = time_us_64();

        // Check for GPS PPS event
        gps_pps_t pps;
        if (gps_get_pps(&pps)) {
            if (pps.valid) {
                // Update discipline based on GPS PPS
                int64_t phase_error = discipline_update_reference(
                    &disciplined_clock,
                    pps.timestamp_us,
                    GPS_PPS_INTERVAL_NS,
                    &core1_stats.lock_event_count,
                    &core1_stats.unlock_event_count
                );

                // Update shared statistics for Core 0 to read
                core1_stats.pps_count++;
                core1_stats.phase_error_ns = phase_error;
                core1_stats.freq_offset_ppb = disciplined_clock.frequency_offset_ppb;
                core1_stats.locked = disciplined_clock.locked;

                // Increment continuous PTP timestamp seconds counter
                ptp_seconds++;

                // Reset output pulse timing on each GPS pulse (since clock resets to 0)
                // Start at 0 to get 100 pulses: 0ms, 10ms, 20ms, ..., 990ms
                next_output_pulse_ns = 0;

                if (first_pulse) {
                    first_pulse = false;
                    core1_stats.first_pps_received = true;
                }
            }
            // Note: Silently ignore invalid PPS - no printf allowed on Core 1
        }

        // TESTING: Blink LED
        if (++led_counter >= 100000) {
            gpio_put(LED_OUTPUT_PIN, !gpio_get(LED_OUTPUT_PIN));
            led_counter = 0;
        }

        // Critical delay for Core 0 - increased from 1us due to heavier workload
        sleep_us(10);
    }

    // FULL DISCIPLINE CODE DISABLED FOR TESTING
    #if 0
    // Initialize disciplined clock with PI controller gains
    // kp = 0.1, ki = 0.001 (starting values, can be tuned)
    discipline_init(&disciplined_clock, 0.1, 0.001);

    uint64_t next_output_pulse_ns = 0;
    bool first_pulse = true;
    uint64_t ptp_seconds = 0;  // Continuous PTP timestamp (seconds portion)

    while (true) {
        // Update disciplined clock time (apply frequency correction)
        discipline_update_time(&disciplined_clock);

        // Publish disciplined clock for PTP timestamping (Phase 2b)
        core1_stats.disciplined_time_ns = disciplined_clock.nanoseconds;

        // Continuous PTP timestamp = seconds * 1e9 + nanoseconds
        core1_stats.continuous_time_ns = (ptp_seconds * 1000000000ULL) + disciplined_clock.nanoseconds;
        core1_stats.last_update_us = time_us_64();

        // Check for GPS PPS event
        gps_pps_t pps;
        if (gps_get_pps(&pps)) {
            if (pps.valid) {
                // Update discipline based on GPS PPS
                int64_t phase_error = discipline_update_reference(
                    &disciplined_clock,
                    pps.timestamp_us,
                    GPS_PPS_INTERVAL_NS,
                    &core1_stats.lock_event_count,
                    &core1_stats.unlock_event_count
                );

                // Update shared statistics for Core 0 to read
                core1_stats.pps_count++;
                core1_stats.phase_error_ns = phase_error;
                core1_stats.freq_offset_ppb = disciplined_clock.frequency_offset_ppb;
                core1_stats.locked = disciplined_clock.locked;

                // Increment continuous PTP timestamp seconds counter
                ptp_seconds++;

                // Reset output pulse timing on each GPS pulse (since clock resets to 0)
                // Start at 0 to get 100 pulses: 0ms, 10ms, 20ms, ..., 990ms
                next_output_pulse_ns = 0;

                if (first_pulse) {
                    first_pulse = false;
                    core1_stats.first_pps_received = true;
                }
            }
            // Note: Silently ignore invalid PPS - no printf allowed on Core 1
        }

        // Generate 100 PPS output when disciplined clock crosses thresholds
        if (!first_pulse) {
            uint64_t current_ns = discipline_get_time_ns(&disciplined_clock);

            if (current_ns >= next_output_pulse_ns) {
                // Generate output pulse (short pulse for precision)
                gpio_put(LED_OUTPUT_PIN, 1);
                busy_wait_us(10);  // 10μs pulse - short enough for minimal blocking
                gpio_put(LED_OUTPUT_PIN, 0);

                // Schedule next pulse (10ms later for 100 PPS)
                next_output_pulse_ns += OUTPUT_PPS_INTERVAL_NS;
            }
        }

        // Small delay to give Core 0 CPU time (critical for USB/Ethernet/printf)
        // WiFi doesn't need this because CYW43 driver naturally yields
        sleep_us(1);  // 1 microsecond - minimal impact on timing precision
    }
    #endif
}

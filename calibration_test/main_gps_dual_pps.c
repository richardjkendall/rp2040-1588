/**
 * GPS Dual 1PPS Output - Calibration Device
 *
 * Generates two GPS-disciplined 1PPS outputs with a precise, known offset
 * for validating measurement device accuracy.
 *
 * HARDWARE:
 *   - GPIO 2:  Input from GPS 1PPS (shared with measurement device)
 *   - GPIO 10: Output Reference 1PPS (to measurement device "GM" input)
 *   - GPIO 11: Output Offset 1PPS (to measurement device "Slave" input)
 *
 * USAGE:
 *   1. Edit TEST_OFFSET_NS below
 *   2. Build and flash to Pico W
 *   3. Connect GPS 1PPS to GPIO 2 (shared with measurement device GPS)
 *   4. Connect GPIO 10 to measurement device GPIO 10
 *   5. Connect GPIO 11 to measurement device GPIO 11
 *   6. Measurement should read TEST_OFFSET_NS ± accuracy
 *
 * ADVANTAGES:
 *   - Pure GPS reference (no GM variability)
 *   - Both signals from same disciplined crystal
 *   - Shared GPS with measurement device (simpler setup)
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "pps_discipline.h"
#include "../slave/pps_scheduler.h"  // Reuse PPS scheduler from slave

// Use GM's interrupt-driven PIO programs (read-only, no modification)
#include "gps_discipline_v2.pio.h"

// =============================================================================
// CONFIGURATION - EDIT THIS TO TEST DIFFERENT OFFSETS
// =============================================================================

// Test offset in nanoseconds (applied between reference and offset outputs)
const int64_t TEST_OFFSET_NS = 100000;  // 100µs

// Uncomment one of these presets or use custom value above:
//const int64_t TEST_OFFSET_NS = 0;         // Zero reference (both aligned)
//const int64_t TEST_OFFSET_NS = 10000;     // 10µs
//const int64_t TEST_OFFSET_NS = 50000;     // 50µs
//const int64_t TEST_OFFSET_NS = 100000;    // 100µs
//const int64_t TEST_OFFSET_NS = 500000;    // 500µs
//const int64_t TEST_OFFSET_NS = 1000000;   // 1ms
//const int64_t TEST_OFFSET_NS = -100000;   // -100µs (test negative polarity)

// =============================================================================

// Pin configuration
#define GPS_PPS_INPUT_PIN    2   // Input from GPS 1PPS (shared with measurement)
#define REFERENCE_PPS_OUTPUT_PIN 10  // Output reference 1PPS (measurement "GM")
#define OFFSET_PPS_OUTPUT_PIN    11  // Output offset 1PPS (measurement "Slave")
#define TRIGGER_PIN         16  // Internal coordination between SM0 and SM1

// PIO configuration
#define DISCIPLINE_PIO pio0
#define COUNTER_SM 0
#define PPS_CAPTURE_SM 1
#define REFERENCE_SCHEDULER_SM 2
#define OFFSET_SCHEDULER_SM 3

/**
 * Get reference time WITH offset for second output
 *
 * Adds TEST_OFFSET_NS to the pure GPS time from get_reference_time_ns()
 */
static uint64_t get_offset_time_ns(void) {
    uint64_t base_time = get_reference_time_ns();  // Pure GPS time

    // Add the configured offset
    if (TEST_OFFSET_NS >= 0) {
        return base_time + (uint64_t)TEST_OFFSET_NS;
    } else {
        uint64_t abs_offset = (uint64_t)(-TEST_OFFSET_NS);
        if (base_time >= abs_offset) {
            return base_time - abs_offset;
        } else {
            return 0;
        }
    }
}

int main() {
    // Overclock to 250 MHz (same as GM/slave for consistency)
    set_sys_clock_khz(250000, true);

    // Initialize stdio
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== GPS Dual 1PPS Output - Calibration Device ===\n");
    printf("System clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);
    printf("Test offset: %+lld ns\n", (long long)TEST_OFFSET_NS);
    printf("GPS 1PPS input: GPIO%d (shared with measurement device)\n", GPS_PPS_INPUT_PIN);
    printf("Reference 1PPS output: GPIO%d\n", REFERENCE_PPS_OUTPUT_PIN);
    printf("Offset 1PPS output: GPIO%d\n\n", OFFSET_PPS_OUTPUT_PIN);

    // Load PIO programs (same as GM - interrupt-driven architecture)
    uint counter_offset = pio_add_program(DISCIPLINE_PIO, &counter_with_pin_trigger_program);
    uint pps_offset = pio_add_program(DISCIPLINE_PIO, &pps_edge_with_pin_signal_program);

    printf("PIO programs loaded (counter @ %d, pps @ %d)\n", counter_offset, pps_offset);

    // Initialize counter SM (SM0) - triggered push via GPIO pin
    counter_with_pin_trigger_program_init(DISCIPLINE_PIO, COUNTER_SM, counter_offset, TRIGGER_PIN);
    printf("Counter SM%d initialized (250 MHz, pin-triggered push on GPIO%d)\n", COUNTER_SM, TRIGGER_PIN);

    // Initialize PPS capture SM (SM1) - pulses trigger pin and triggers IRQ
    pps_edge_with_pin_signal_program_init(DISCIPLINE_PIO, PPS_CAPTURE_SM, pps_offset,
                                          GPS_PPS_INPUT_PIN, TRIGGER_PIN);
    printf("PPS capture SM%d initialized (GPIO%d -> GPIO%d trigger + IRQ)\n",
           PPS_CAPTURE_SM, GPS_PPS_INPUT_PIN, TRIGGER_PIN);

    // Initialize GPS PPS discipline
    if (!pps_discipline_init(GPS_PPS_INPUT_PIN)) {
        printf("FATAL: GPS PPS discipline init failed\n");
        while (1) { sleep_ms(1000); }
    }

    // Initialize Reference PPS scheduler (SM2)
    pps_scheduler_t reference_sched = {
        .pio = DISCIPLINE_PIO,
        .sm = REFERENCE_SCHEDULER_SM,
        .pin = REFERENCE_PPS_OUTPUT_PIN,
        .get_time_ns = get_reference_time_ns,  // Pure GPS time
        .get_scale_factor = pps_discipline_get_scale_factor
    };

    if (!pps_scheduler_init(&reference_sched)) {
        printf("FATAL: Reference PPS scheduler init failed\n");
        while (1) { sleep_ms(1000); }
    }

    printf("Reference PPS scheduler SM%d initialized (output on GPIO%d)\n",
           REFERENCE_SCHEDULER_SM, REFERENCE_PPS_OUTPUT_PIN);

    // Initialize Offset PPS scheduler (SM3)
    pps_scheduler_t offset_sched = {
        .pio = DISCIPLINE_PIO,
        .sm = OFFSET_SCHEDULER_SM,
        .pin = OFFSET_PPS_OUTPUT_PIN,
        .get_time_ns = get_offset_time_ns,  // GPS time + offset
        .get_scale_factor = pps_discipline_get_scale_factor
    };

    if (!pps_scheduler_init(&offset_sched)) {
        printf("FATAL: Offset PPS scheduler init failed\n");
        while (1) { sleep_ms(1000); }
    }

    printf("Offset PPS scheduler SM%d initialized (output on GPIO%d)\n",
           OFFSET_SCHEDULER_SM, OFFSET_PPS_OUTPUT_PIN);

    printf("\nWaiting for GPS 1PPS signal...\n");
    printf("(Shared GPS with measurement device on GPIO %d)\n\n", GPS_PPS_INPUT_PIN);

    uint64_t last_reference_schedule_us = 0;
    uint64_t last_offset_schedule_us = 0;
    uint64_t last_stats_us = 0;
    bool pps_output_enabled = false;

    // Main loop (PPS edges handled in IRQ - see pps_discipline.c)
    while (true) {
        uint64_t now_us = time_us_64();

        // Enable dual PPS output after achieving lock (10 PPS edges received)
        if (!pps_output_enabled && pps_stats.locked && pps_stats.pps_count >= 10) {
            pps_output_enabled = true;
            printf("\n=== DUAL PPS OUTPUT ENABLED ===\n");
            printf("GPS discipline locked\n");
            printf("Reference 1PPS on GPIO%d (GPS time)\n", REFERENCE_PPS_OUTPUT_PIN);
            printf("Offset 1PPS on GPIO%d (GPS time %+lldns)\n\n",
                   OFFSET_PPS_OUTPUT_PIN, (long long)TEST_OFFSET_NS);
        }

        // Schedule dual 1PPS outputs
        // Only when locked, output enabled, and near second boundary
        if (pps_output_enabled && pps_stats.locked) {
            uint64_t ref_time_ns = get_reference_time_ns();
            uint64_t ns_in_second = ref_time_ns % 1000000000ULL;

            // Schedule when in last 100ms of second AND at least 800ms since last schedule
            if (ns_in_second > 900000000ULL) {
                // Schedule reference output
                if (now_us - last_reference_schedule_us >= 800000) {
                    pps_scheduler_schedule_next(&reference_sched);
                    last_reference_schedule_us = now_us;
                }

                // Schedule offset output (slightly staggered to avoid PIO contention)
                if (now_us - last_offset_schedule_us >= 800000) {
                    pps_scheduler_schedule_next(&offset_sched);
                    last_offset_schedule_us = now_us;
                }
            }
        }

        // Print stats every 30 seconds
        if (now_us - last_stats_us >= 30000000) {
            printf("\n=== Status ===\n");
            printf("GPS PPS Count: %llu\n", pps_stats.pps_count);
            printf("Discipline Updates: %llu\n", pps_stats.discipline_updates);
            printf("Locked: %s\n", pps_stats.locked ? "YES" : "NO");
            printf("Crystal Error: %+lld ns (%+.3f ppm)\n",
                   (long long)pps_stats.crystal_error_ns, pps_stats.crystal_ppm);
            printf("Scale Factor: %.9f\n", pps_stats.scale_factor);
            printf("Reference Time: %llu ns\n", pps_stats.reference_time_ns);
            printf("Output Enabled: %s\n", pps_output_enabled ? "YES" : "NO");
            if (pps_output_enabled) {
                printf("Reference Output: GPIO%d (GPS time)\n", REFERENCE_PPS_OUTPUT_PIN);
                printf("Offset Output: GPIO%d (GPS time %+lldns)\n",
                       OFFSET_PPS_OUTPUT_PIN, (long long)TEST_OFFSET_NS);
            }
            printf("\n");

            last_stats_us = now_us;
        }

        // Small sleep to avoid busy-wait
        sleep_ms(10);
    }

    return 0;
}

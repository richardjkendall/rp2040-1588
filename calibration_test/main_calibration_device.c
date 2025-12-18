/**
 * PTP Measurement Calibration Device
 *
 * Disciplines to external GM 1PPS reference and generates offset 1PPS output
 * for validating measurement device accuracy.
 *
 * HARDWARE:
 *   - GPIO 2:  Input from GM 1PPS (reference)
 *   - GPIO 15: Output offset 1PPS (to measurement device)
 *
 * USAGE:
 *   1. Edit TEST_OFFSET_NS below
 *   2. Build and flash to Pico W
 *   3. Connect GM 1PPS to GPIO 2
 *   4. Connect GPIO 15 to measurement device
 *   5. Measurement should read TEST_OFFSET_NS ± accuracy
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

// Test offset in nanoseconds
// Change this value, rebuild, and reflash to test different offsets
const int64_t TEST_OFFSET_NS = 100000;  // 100µs

// Uncomment one of these presets or use custom value above:
//const int64_t TEST_OFFSET_NS = 0;         // Zero reference
//const int64_t TEST_OFFSET_NS = 10000;     // 10µs
//const int64_t TEST_OFFSET_NS = 50000;     // 50µs
//const int64_t TEST_OFFSET_NS = 100000;    // 100µs
//const int64_t TEST_OFFSET_NS = 500000;    // 500µs
//const int64_t TEST_OFFSET_NS = 1000000;   // 1ms
//const int64_t TEST_OFFSET_NS = -100000;   // -100µs (test negative polarity)

// =============================================================================

// Pin configuration
#define GM_PPS_INPUT_PIN    2   // Input from GM 1PPS reference
#define TEST_PPS_OUTPUT_PIN 15  // Output offset 1PPS to measurement device
#define TRIGGER_PIN         16  // Internal coordination between SM0 and SM1 (same as GM)

// PIO configuration
#define DISCIPLINE_PIO pio0
#define COUNTER_SM 0
#define PPS_CAPTURE_SM 1
#define PPS_SCHEDULER_SM 2

// PPS edge handling now done in IRQ handler (see pps_discipline.c)

/**
 * Get reference time WITH configured offset
 *
 * Wrapper around get_reference_time_ns() that adds TEST_OFFSET_NS
 */
static uint64_t get_offset_reference_time_ns(void) {
    uint64_t base_time = get_reference_time_ns();

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

    printf("\n=== PTP Measurement Calibration Device ===\n");
    printf("System clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);
    printf("Test offset: %+lld ns\n", (long long)TEST_OFFSET_NS);
    printf("GM 1PPS input: GPIO%d\n", GM_PPS_INPUT_PIN);
    printf("Test 1PPS output: GPIO%d\n\n", TEST_PPS_OUTPUT_PIN);

    // Load PIO programs (same as GM - interrupt-driven architecture)
    uint counter_offset = pio_add_program(DISCIPLINE_PIO, &counter_with_pin_trigger_program);
    uint pps_offset = pio_add_program(DISCIPLINE_PIO, &pps_edge_with_pin_signal_program);

    printf("PIO programs loaded (counter @ %d, pps @ %d)\n", counter_offset, pps_offset);

    // Initialize counter SM (SM0) - triggered push via GPIO pin
    counter_with_pin_trigger_program_init(DISCIPLINE_PIO, COUNTER_SM, counter_offset, TRIGGER_PIN);
    printf("Counter SM%d initialized (250 MHz, pin-triggered push on GPIO%d)\n", COUNTER_SM, TRIGGER_PIN);

    // Initialize PPS capture SM (SM1) - pulses trigger pin and triggers IRQ
    pps_edge_with_pin_signal_program_init(DISCIPLINE_PIO, PPS_CAPTURE_SM, pps_offset,
                                          GM_PPS_INPUT_PIN, TRIGGER_PIN);
    printf("PPS capture SM%d initialized (GPIO%d -> GPIO%d trigger + IRQ)\n",
           PPS_CAPTURE_SM, GM_PPS_INPUT_PIN, TRIGGER_PIN);

    // Initialize PPS discipline
    if (!pps_discipline_init(GM_PPS_INPUT_PIN)) {
        printf("FATAL: PPS discipline init failed\n");
        while (1) { sleep_ms(1000); }
    }

    // Initialize PPS scheduler for offset 1PPS output
    pps_scheduler_t pps_sched = {
        .pio = DISCIPLINE_PIO,
        .sm = PPS_SCHEDULER_SM,
        .pin = TEST_PPS_OUTPUT_PIN,
        .get_time_ns = get_offset_reference_time_ns,  // Use wrapper with offset
        .get_scale_factor = pps_discipline_get_scale_factor
    };

    if (!pps_scheduler_init(&pps_sched)) {
        printf("FATAL: PPS scheduler init failed\n");
        while (1) { sleep_ms(1000); }
    }

    printf("PPS scheduler SM%d initialized (output on GPIO%d)\n", PPS_SCHEDULER_SM, TEST_PPS_OUTPUT_PIN);
    printf("\nWaiting for GM 1PPS reference...\n");
    printf("(Connect GM GPIO 15 to this device GPIO %d)\n\n", GM_PPS_INPUT_PIN);

    uint64_t last_pps_schedule_us = 0;
    uint64_t last_stats_us = 0;
    bool pps_output_enabled = false;

    // Main loop (PPS edges handled in IRQ - see pps_discipline.c)
    while (true) {
        uint64_t now_us = time_us_64();

        // Enable PPS output after achieving lock (10 PPS edges received)
        if (!pps_output_enabled && pps_stats.locked && pps_stats.pps_count >= 10) {
            pps_output_enabled = true;
            printf("\n=== PPS OUTPUT ENABLED ===\n");
            printf("Discipline locked (offset=%+lldns)\n", (long long)TEST_OFFSET_NS);
            printf("Output 1PPS on GPIO%d with %+lldns offset from GM\n\n",
                   TEST_PPS_OUTPUT_PIN, (long long)TEST_OFFSET_NS);
        }

        // Schedule offset 1PPS output
        // Only when locked, output enabled, and near second boundary
        if (pps_output_enabled && pps_stats.locked) {
            uint64_t ref_time_ns = get_reference_time_ns();
            uint64_t ns_in_second = ref_time_ns % 1000000000ULL;

            // Schedule when in last 100ms of second AND at least 800ms since last schedule
            if (ns_in_second > 900000000ULL && (now_us - last_pps_schedule_us >= 800000)) {
                // Schedule PPS (will use get_reference_time_ns which includes offset)
                pps_scheduler_schedule_next(&pps_sched);

                last_pps_schedule_us = now_us;
            }
        }

        // Print stats every 30 seconds
        if (now_us - last_stats_us >= 30000000) {
            printf("\n=== Status ===\n");
            printf("PPS Count: %llu\n", pps_stats.pps_count);
            printf("Discipline Updates: %llu\n", pps_stats.discipline_updates);
            printf("Locked: %s\n", pps_stats.locked ? "YES" : "NO");
            printf("Crystal Error: %+lld ns (%+.3f ppm)\n",
                   (long long)pps_stats.crystal_error_ns, pps_stats.crystal_ppm);
            printf("Scale Factor: %.9f\n", pps_stats.scale_factor);
            printf("Reference Time: %llu ns\n", pps_stats.reference_time_ns);
            printf("Output Enabled: %s\n", pps_output_enabled ? "YES" : "NO");
            if (pps_output_enabled) {
                printf("Output Offset: %+lld ns\n", (long long)TEST_OFFSET_NS);
            }
            printf("\n");

            last_stats_us = now_us;
        }

        // Small sleep to avoid busy-wait
        sleep_ms(10);
    }

    return 0;
}

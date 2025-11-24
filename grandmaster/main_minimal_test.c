/**
 * PTP Grandmaster - MINIMAL TEST VERSION V3
 *
 * V3: PIO IRQ flag synchronization (no continuous DMA)
 *
 * NO printf in main loop
 * NO network polling
 * NO GPS NMEA parsing
 *
 * Only GPS PPS discipline and GPIO signals for scope measurement
 *
 * GPIO Signals:
 * - GPIO 3: 100 PPS output (LED) - measure precision on scope
 * - GPIO 4: GPS PPS IRQ toggle - verify 1 Hz
 * - GPIO 5: Discipline lock status (high = locked)
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "gps.h"
#include "shared_state.h"
#include "discipline_v3.h"

// Pin definitions
#define GPS_UART_ID uart0
#define GPS_TX_PIN 0
#define GPS_RX_PIN 1
#define GPS_PPS_PIN 2
#define LED_OUTPUT_PIN 3      // 100 PPS output
#define DEBUG_PPS_PIN 4       // Toggle on GPS PPS IRQ
#define DEBUG_LOCK_PIN 5      // High when locked

// PIO configuration
#define GPS_PIO pio1
#define GPS_SM 0

// Export lock pin for discipline module
uint debug_lock_pin = DEBUG_LOCK_PIN;
uint debug_pps_pin = DEBUG_PPS_PIN;

int main() {
    // Overclock to 250 MHz
    set_sys_clock_khz(250000, true);

    // Initialize stdio (for startup messages only)
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== MINIMAL TEST - GPS Discipline ===\n");
    printf("System clock: 250 MHz\n\n");

    // Initialize debug GPIOs
    gpio_init(LED_OUTPUT_PIN);
    gpio_set_dir(LED_OUTPUT_PIN, GPIO_OUT);
    gpio_put(LED_OUTPUT_PIN, 0);

    gpio_init(DEBUG_PPS_PIN);
    gpio_set_dir(DEBUG_PPS_PIN, GPIO_OUT);
    gpio_put(DEBUG_PPS_PIN, 0);

    gpio_init(DEBUG_LOCK_PIN);
    gpio_set_dir(DEBUG_LOCK_PIN, GPIO_OUT);
    gpio_put(DEBUG_LOCK_PIN, 0);

    printf("Debug GPIOs initialized:\n");
    printf("  GPIO %d: 100 PPS output\n", LED_OUTPUT_PIN);
    printf("  GPIO %d: GPS PPS IRQ toggle\n", DEBUG_PPS_PIN);
    printf("  GPIO %d: Lock status\n\n", DEBUG_LOCK_PIN);

    // Initialize GPS module (NMEA parsing + PPS on PIO1)
    gps_init(GPS_UART_ID, GPS_TX_PIN, GPS_RX_PIN, GPS_PPS_PIN, GPS_PIO, GPS_SM);

    // Initialize GPS discipline system
    if (!discipline_init_v3()) {
        printf("FATAL: Discipline init failed\n");
        while (1) {
            sleep_ms(1000);
        }
    }

    printf("\n*** ENTERING TEST LOOP WITH STATS ***\n\n");
    sleep_ms(1000);

    uint64_t last_stats_time_us = time_us_64();
    uint32_t last_pps_count = 0;
    uint32_t last_pulse_count = 0;
    uint32_t last_out_of_range = 0;
    uint32_t last_too_late = 0;

    // TEST LOOP - Periodic stats output
    while (true) {
        // Generate 100 PPS output (GPS-disciplined)
        discipline_generate_100pps();

        // Update lock status GPIO
        if (core1_stats.locked) {
            gpio_put(DEBUG_LOCK_PIN, 1);
        } else {
            gpio_put(DEBUG_LOCK_PIN, 0);
        }

        // Print stats every 5 seconds
        uint64_t now_us = time_us_64();
        if (now_us - last_stats_time_us >= 5000000) {
            // Check if we're receiving PPS
            bool pps_active = (core1_stats.pps_count > last_pps_count);
            last_pps_count = core1_stats.pps_count;

            // Get 100 PPS debug stats
            uint32_t fired, out_of_range, too_early, too_late, guard_blocked;
            discipline_get_100pps_stats(&fired, &out_of_range, &too_early, &too_late, &guard_blocked);

            // Calculate deltas in last 5 seconds
            uint32_t pulses_this_interval = fired - last_pulse_count;
            uint32_t out_of_range_delta = out_of_range - last_out_of_range;
            uint32_t too_late_delta = too_late - last_too_late;
            last_pulse_count = fired;
            last_out_of_range = out_of_range;
            last_too_late = too_late;

            // Get discipline correction and times
            int64_t correction_us = discipline_get_correction_us();
            int64_t disciplined_error_ns = discipline_get_disciplined_error_ns();

            // Get debug info
            int64_t elapsed_raw, freq_correction_calc;
            double freq_ppm;
            discipline_get_debug_info(&elapsed_raw, &freq_ppm, &freq_correction_calc);

            printf("PPS:%lu PTP_accuracy:%lldns crystal_err:%lldns | freq_ppm:%.3f elapsed:%lldus corr_calc:%lldus | 100PPS:%luHz lock:%s\n",
                   core1_stats.pps_count,
                   (long long)disciplined_error_ns,
                   (long long)core1_stats.phase_error_ns,
                   freq_ppm,
                   (long long)elapsed_raw,
                   (long long)freq_correction_calc,
                   pulses_this_interval / 5,
                   core1_stats.locked ? "YES" : "NO ");

            last_stats_time_us = now_us;
        }

        // Small delay to prevent tight loop
        sleep_us(10);
    }

    return 0;
}

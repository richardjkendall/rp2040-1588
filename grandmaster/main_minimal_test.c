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
#include <stdlib.h>
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

    // Initialize debug GPIOs (minimal set)
    gpio_init(DEBUG_PPS_PIN);
    gpio_set_dir(DEBUG_PPS_PIN, GPIO_OUT);
    gpio_put(DEBUG_PPS_PIN, 0);

    printf("Debug GPIOs initialized:\n");
    printf("  GPIO %d: GPS PPS IRQ toggle (1 Hz)\n\n", DEBUG_PPS_PIN);

    // Initialize GPS module (NMEA parsing + PPS on PIO1)
    gps_init(GPS_UART_ID, GPS_TX_PIN, GPS_RX_PIN, GPS_PPS_PIN, GPS_PIO, GPS_SM);

    // Initialize GPS discipline system
    if (!discipline_init_v3()) {
        printf("FATAL: Discipline init failed\n");
        while (1) {
            sleep_ms(1000);
        }
    }

    printf("\n*** ULTRA-MINIMAL TEST - ZERO INTERFERENCE ***\n");
    printf("*** Removed: 100 PPS generation, GPIO updates, tight loop ***\n\n");
    sleep_ms(1000);

    uint64_t last_stats_time_us = time_us_64();
    uint32_t last_pps_count = 0;

    // ULTRA-MINIMAL LOOP - Only periodic stats, NO continuous processing
    while (true) {
        // Print stats every 10 seconds
        uint64_t now_us = time_us_64();
        if (now_us - last_stats_time_us >= 10000000) {
            // Check if we're receiving PPS
            bool pps_active = (core1_stats.pps_count > last_pps_count);
            last_pps_count = core1_stats.pps_count;

            // Get GPS nanosecond counter metrics
            extern volatile uint64_t gps_ns_counter;
            extern volatile int32_t crystal_error_ns;
            extern volatile int64_t interpolation_error_ns;

            uint64_t gps_seconds = gps_ns_counter / 1000000000ULL;

            // Convert crystal error to ppm (parts per million)
            // crystal_error_ns is error over 1 second (1,000,000,000 ns)
            // ppm = error / 1e9 * 1e6 = error_ns / 1000
            double crystal_ppm = (double)crystal_error_ns / 1000.0;

            printf("PPS:%lu GPS:%llus crystal_err:%+ldns (%+.3fppm) interp_err:%+lldns lock:%s\n",
                   core1_stats.pps_count,
                   (unsigned long long)gps_seconds,
                   (long)crystal_error_ns,
                   crystal_ppm,
                   (long long)interpolation_error_ns,
                   core1_stats.locked ? "YES" : "NO ");

            last_stats_time_us = now_us;
        }

        // Sleep to minimize CPU interference (wake every 100ms for stats check)
        sleep_ms(100);
    }

    return 0;
}

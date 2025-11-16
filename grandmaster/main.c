/**
 * PTP Grandmaster - Main Program (Core 0)
 *
 * Phase 1: GPS-Disciplined Clock with 1 PPS Output
 *
 * Hardware connections:
 * - GPS PPS: GPIO 2
 * - GPS TX: GPIO 1 (to Pico RX)
 * - GPS RX: GPIO 0 (to Pico TX)
 * - LED Output: GPIO 25 (onboard LED) or GPIO 3
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "gps.h"
#include "shared_state.h"

// Pin definitions
#define GPS_UART_ID uart0
#define GPS_TX_PIN 0
#define GPS_RX_PIN 1
#define GPS_PPS_PIN 2
#define LED_OUTPUT_PIN 3  // External LED (GPIO 3) - Pico W onboard LED (25) needs CYW43 driver

// PIO configuration
#define GPS_PIO pio0
#define GPS_SM 0

// Forward declaration of Core 1 entry point
void core1_entry();

int main() {
    // Initialize stdio for debug output
    stdio_init_all();

    // Wait a moment for USB serial to connect
    sleep_ms(2000);

    printf("\n=== PTP Grandmaster - Phase 2a ===\n");
    printf("GPS-Disciplined Clock with 100 PPS Output\n\n");

    // Initialize LED output
    gpio_init(LED_OUTPUT_PIN);
    gpio_set_dir(LED_OUTPUT_PIN, GPIO_OUT);
    gpio_put(LED_OUTPUT_PIN, 0);

    // Initialize GPS module
    printf("Initializing GPS module...\n");
    gps_init(GPS_UART_ID, GPS_TX_PIN, GPS_RX_PIN, GPS_PPS_PIN, GPS_PIO, GPS_SM);

    printf("Waiting for GPS fix...\n");
    printf("(This may take 30-60 seconds outdoors with clear sky view)\n\n");

    // Launch Core 1 for time discipline
    printf("Launching Core 1 for clock discipline...\n");
    multicore_launch_core1(core1_entry);

    // Core 0 main loop
    uint32_t last_gps_status_ms = 0;
    uint32_t last_core1_stats_ms = 0;
    uint32_t last_lock_event_count = 0;
    uint32_t last_unlock_event_count = 0;

    while (true) {
        // Process GPS NMEA data
        gps_process();

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        // Print GPS status every 2 seconds
        if (now_ms - last_gps_status_ms >= 2000) {
            last_gps_status_ms = now_ms;

            gps_data_t gps_data;
            gps_get_data(&gps_data);

            printf("GPS Status: ");
            if (gps_has_fix()) {
                printf("FIX - %d sats - Time: %02d:%02d:%02d.%03d UTC\n",
                       gps_data.satellites,
                       gps_data.hours, gps_data.minutes, gps_data.seconds,
                       gps_data.milliseconds);
            } else {
                printf("NO FIX - %d sats\n", gps_data.satellites);
            }
        }

        // Print Core 1 statistics every 5 seconds
        if (now_ms - last_core1_stats_ms >= 5000) {
            last_core1_stats_ms = now_ms;

            if (core1_stats.discipline_running) {
                if (core1_stats.first_pps_received) {
                    printf("Core 1 Stats: PPS count=%lu, Phase error=%lld ns, Freq offset=%ld ppb, Locked=%s\n",
                           (unsigned long)core1_stats.pps_count,
                           (long long)core1_stats.phase_error_ns,
                           (long)core1_stats.freq_offset_ppb,
                           core1_stats.locked ? "YES" : "NO");
                } else {
                    printf("Core 1: Running, waiting for first GPS PPS...\n");
                }
            }
        }

        // Detect and print lock/unlock events
        if (core1_stats.lock_event_count != last_lock_event_count) {
            last_lock_event_count = core1_stats.lock_event_count;
            printf("Discipline: LOCKED (phase_error=%lld ns, freq_offset=%ld ppb)\n",
                   (long long)core1_stats.phase_error_ns,
                   (long)core1_stats.freq_offset_ppb);
            printf("  -> Clock disciplined to GPS. Phase error < 1ms for 5 consecutive samples.\n");
        }

        if (core1_stats.unlock_event_count != last_unlock_event_count) {
            last_unlock_event_count = core1_stats.unlock_event_count;
            printf("Discipline: LOST LOCK (phase_error=%lld ns exceeds 10ms threshold)\n",
                   (long long)core1_stats.phase_error_ns);
        }

        // First PPS event
        static bool first_pps_printed = false;
        if (core1_stats.first_pps_received && !first_pps_printed) {
            first_pps_printed = true;
            printf("Core 1: First GPS PPS received, starting output generation\n");
        }

        // Small delay to avoid busy-waiting
        sleep_ms(10);
    }

    return 0;
}

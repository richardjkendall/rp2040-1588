/**
 * PTP Grandmaster - Main Program (Core 0)
 *
 * Phase 2b: GPS-Disciplined PTP Grandmaster with WiFi
 *
 * Hardware connections:
 * - GPS PPS: GPIO 2
 * - GPS TX: GPIO 1 (to Pico RX)
 * - GPS RX: GPIO 0 (to Pico TX)
 * - LED Output: GPIO 3
 * - WiFi: Built-in CYW43 on Pico W
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "gps.h"
#include "shared_state.h"
#include "network_interface.h"
#include "ptp_grandmaster.h"
#include "discipline_v2.h"

// Pin definitions
#define GPS_UART_ID uart0
#define GPS_TX_PIN 0
#define GPS_RX_PIN 1
#define GPS_PPS_PIN 2
#define LED_OUTPUT_PIN 3  // External LED (GPIO 3) - Pico W onboard LED (25) needs CYW43 driver

// PIO configuration
// NOTE: Discipline uses PIO0 SM0+SM1, so GPS uses PIO1
#define GPS_PIO pio1
#define GPS_SM 0

int main() {
    // Overclock to 250 MHz for better timing precision
    // Reduces interrupt latency and WiFi overhead impact on discipline
    // MUST be done before stdio_init_all() to avoid USB timing issues
    set_sys_clock_khz(250000, true);

    // Initialize stdio for debug output (after clock change)
    stdio_init_all();

    // Wait a moment for USB serial to connect
    sleep_ms(2000);

    printf("\n=== PTP Grandmaster - Phase 2b ===\n");
    printf("GPS-Disciplined PTP Grandmaster with Ethernet\n");
    printf("System clock: 250 MHz\n\n");

    // Initialize network (WiFi or Ethernet depending on build)
    if (!network_init()) {
        printf("FATAL: Network init failed - check network_config.h\n");
        while (1) {
            sleep_ms(1000);
        }
    }

    // Initialize PTP Grandmaster (Phase 2b)
    if (!ptp_grandmaster_init()) {
        printf("FATAL: PTP init failed\n");
        while (1) {
            sleep_ms(1000);
        }
    }

    // Initialize LED output
    gpio_init(LED_OUTPUT_PIN);
    gpio_set_dir(LED_OUTPUT_PIN, GPIO_OUT);
    gpio_put(LED_OUTPUT_PIN, 0);

    // Initialize GPS module (NMEA parsing + PPS on PIO1)
    gps_init(GPS_UART_ID, GPS_TX_PIN, GPS_RX_PIN, GPS_PPS_PIN, GPS_PIO, GPS_SM);

    // Initialize GPS discipline system (250 MHz counter on PIO0, software discipline)
    if (!discipline_init_v2()) {
        printf("FATAL: Discipline init failed\n");
        while (1) {
            sleep_ms(1000);
        }
    }

    // Core 0 main loop
    uint32_t last_status_ms = 0;
    uint32_t last_lock_event_count = 0;
    uint32_t last_unlock_event_count = 0;

    printf("System ready. Status every 10s, events shown immediately.\n\n");

    while (true) {
        // Poll network stack (required for poll mode)
        network_poll();

        // Process GPS NMEA data
        gps_process();

        // Process PTP grandmaster (send messages at 1 Hz)
        ptp_grandmaster_process();

        // Generate 100 PPS output (GPS-disciplined)
        discipline_generate_100pps();

        // Update discipline stats (for PTP timestamping)
        discipline_update_stats();

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        // Print consolidated status every 10 seconds
        if (now_ms - last_status_ms >= 10000) {
            last_status_ms = now_ms;

            gps_data_t gps_data;
            gps_get_data(&gps_data);

            char ip_addr[16];
            network_get_ip_str(ip_addr, sizeof(ip_addr));

            uint32_t announce_count, sync_count, followup_count, delay_resp_count, active_slaves;
            ptp_grandmaster_get_stats_extended(&announce_count, &sync_count, &followup_count,
                                               &delay_resp_count, &active_slaves);

            // Single-line status summary
            printf("Status: GPS=%s(%dsats) Lock=%s Phase=%lldns Freq=%ldppb WiFi=%s PTP=%lu/%lu/%lu/%lu Slaves=%lu PPS_IRQ=%lu PPS_Core1=%lu\n",
                   gps_has_fix() ? "FIX" : "NOFIX",
                   gps_data.satellites,
                   core1_stats.locked ? "YES" : "NO",
                   (long long)core1_stats.phase_error_ns,
                   (long)core1_stats.freq_offset_ppb,
                   network_is_connected() ? "OK" : "DOWN",
                   (unsigned long)announce_count,
                   (unsigned long)sync_count,
                   (unsigned long)followup_count,
                   (unsigned long)delay_resp_count,
                   (unsigned long)active_slaves,
                   (unsigned long)gps_get_pps_irq_count(),
                   (unsigned long)core1_stats.pps_count);
        }

        // Detect and print lock/unlock events
        if (core1_stats.lock_event_count != last_lock_event_count) {
            last_lock_event_count = core1_stats.lock_event_count;
            printf("EVENT: Discipline LOCKED (phase=%lld ns, freq=%ld ppb)\n",
                   (long long)core1_stats.phase_error_ns,
                   (long)core1_stats.freq_offset_ppb);
        }

        if (core1_stats.unlock_event_count != last_unlock_event_count) {
            last_unlock_event_count = core1_stats.unlock_event_count;
            printf("EVENT: Discipline LOST LOCK (phase=%lld ns)\n",
                   (long long)core1_stats.phase_error_ns);
        }

        // First PPS event
        static bool first_pps_printed = false;
        if (core1_stats.first_pps_received && !first_pps_printed) {
            first_pps_printed = true;
            printf("EVENT: First GPS PPS received\n");
        }

        // Very small delay - WiFi needs frequent polling
        sleep_ms(1);
    }

    return 0;
}

/**
 * PTP Slave - Main Program (Core 0)
 *
 * Phase 3: PTP Slave synchronized to Grandmaster
 *
 * Hardware connections:
 * - 100 PPS Output: GPIO 3 (for scope comparison with grandmaster)
 * - WiFi: Built-in CYW43 on Pico W
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "shared_state.h"
#include "wifi_init.h"
#include "ptp_slave.h"

// Forward declaration of Core 1 entry point
void core1_entry();

int main() {
    // Initialize stdio for debug output
    stdio_init_all();

    // Overclock to 250 MHz for better timing precision
    // Matches grandmaster clock speed for consistent performance
    set_sys_clock_khz(250000, true);

    // Wait a moment for USB serial to connect
    sleep_ms(2000);

    printf("\n=== PTP Slave - Phase 3 ===\n");
    printf("PTP Slave synchronized to Grandmaster\n");
    printf("System clock: 250 MHz\n\n");

    // Initialize WiFi
    if (!wifi_init_and_connect()) {
        printf("FATAL: WiFi init failed - check wifi_config.h\n");
        while (1) {
            sleep_ms(1000);
        }
    }

    // Initialize PTP Slave
    if (!ptp_slave_init()) {
        printf("FATAL: PTP slave init failed\n");
        while (1) {
            sleep_ms(1000);
        }
    }

    // Launch Core 1 for clock discipline and 100 PPS generation
    multicore_launch_core1(core1_entry);

    // Core 0 main loop
    uint32_t last_status_ms = 0;
    uint32_t last_lock_event_count = 0;
    uint32_t last_unlock_event_count = 0;

    printf("System ready. Status every 10s, events shown immediately.\n\n");

    while (true) {
        // Poll WiFi/lwIP stack (required for poll mode)
        wifi_poll();

        // Process PTP slave (callback-based, but keep this for future expansion)
        ptp_slave_process();

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        // Print consolidated status every 10 seconds
        if (now_ms - last_status_ms >= 10000) {
            last_status_ms = now_ms;

            char ip_addr[16];
            wifi_get_ip_address(ip_addr, sizeof(ip_addr));

            uint32_t sync_count, announce_count;
            ptp_slave_get_stats(&sync_count, &announce_count);

            // Single-line status summary
            printf("Status: PTP_RX=%lu/%lu Lock=%s Phase=%lldns Freq=%ldppb WiFi=%s\n",
                   (unsigned long)sync_count,
                   (unsigned long)announce_count,
                   core1_stats.locked ? "YES" : "NO",
                   (long long)core1_stats.phase_error_ns,
                   (long)core1_stats.freq_offset_ppb,
                   wifi_is_connected() ? "OK" : "DOWN");
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

        // First Sync event
        static bool first_sync_printed = false;
        if (core1_stats.first_sync_received && !first_sync_printed) {
            first_sync_printed = true;
            printf("EVENT: First PTP Sync received\n");
        }

        // Very small delay - WiFi needs frequent polling
        sleep_ms(1);
    }

    return 0;
}

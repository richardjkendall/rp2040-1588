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
#include "network_interface.h"
#include "ptp_slave.h"

// Forward declaration of Core 1 entry point
void core1_entry();

int main() {
    // Overclock to 250 MHz for better timing precision
    // Matches grandmaster clock speed for consistent performance
    // MUST be done before stdio_init_all() to avoid USB timing issues
    set_sys_clock_khz(250000, true);

    // Initialize stdio for debug output (after clock change)
    stdio_init_all();

    // Wait a moment for USB serial to connect
    sleep_ms(2000);

    printf("\n=== PTP Slave - Phase 3 ===\n");
    printf("PTP Slave synchronized to Grandmaster\n");
    printf("System clock: 250 MHz\n\n");

    // Initialize network (WiFi or Ethernet depending on build)
    if (!network_init()) {
        printf("FATAL: Network init failed - check network_config.h\n");
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
        // Poll network stack (required for poll mode)
        network_poll();

        // Process PTP slave (callback-based, but keep this for future expansion)
        ptp_slave_process();

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        // Print consolidated status every 10 seconds
        if (now_ms - last_status_ms >= 10000) {
            last_status_ms = now_ms;

            char ip_addr[16];
            network_get_ip_str(ip_addr, sizeof(ip_addr));

            uint32_t sync_count, announce_count;
            ptp_slave_get_stats(&sync_count, &announce_count);

            int64_t offset_ns, path_delay_ns, pdv_ns;
            bool offset_valid;
            ptp_slave_get_timing(&offset_ns, &path_delay_ns, &pdv_ns, &offset_valid);

            uint32_t pdv_rejected = ptp_slave_get_pdv_rejected();

            // Single-line status summary with two-way PTP timing
            printf("Status: PTP_RX=%lu/%lu Lock=%s PhaseRaw=%lldns PhaseFlt=%lldns Freq=%ldppb PathDelay=%0.2fms PDV=%0.2fms Mode=%s Rejected=%lu WiFi=%s\n",
                   (unsigned long)sync_count,
                   (unsigned long)announce_count,
                   core1_stats.locked ? "YES" : "NO",
                   (long long)core1_stats.raw_phase_error_ns,
                   (long long)core1_stats.phase_error_ns,
                   (long)core1_stats.freq_offset_ppb,
                   (double)path_delay_ns / 1000000.0,
                   (double)pdv_ns / 1000000.0,
                   offset_valid ? "2WAY" : "1WAY",
                   (unsigned long)pdv_rejected,
                   network_is_connected() ? "OK" : "DOWN");
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

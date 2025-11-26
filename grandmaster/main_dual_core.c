/**
 * PTP Grandmaster - Dual-Core Implementation
 *
 * Architecture:
 * - Core 0: GPS discipline, timing (time-critical, minimal load)
 * - Core 1: Network (W5500), PTP protocol (heavy workload)
 *
 * Thread Safety:
 * - Core 0 WRITES timing variables (gps_ns_counter, etc.) in GPS PPS IRQ
 * - Core 1 READS timing variables via get_gps_time_ns()
 * - No locks needed (atomic reads, single writer)
 */

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "gps.h"
#include "shared_state.h"
#include "discipline_v3.h"
#include "network_interface.h"
#include "ptp_grandmaster.h"

// Pin definitions
#define GPS_UART_ID uart0
#define GPS_TX_PIN 0
#define GPS_RX_PIN 1
#define GPS_PPS_PIN 2
#define DEBUG_PPS_PIN 4       // Toggle on GPS PPS IRQ

// PIO configuration
#define GPS_PIO pio1
#define GPS_SM 0

// Export lock pin for discipline module
uint debug_lock_pin = 5;  // Not used in dual-core version
uint debug_pps_pin = DEBUG_PPS_PIN;

// Core synchronization
volatile bool core0_ready = false;

/**
 * Core 1 Entry Point: Network and PTP Processing
 */
void core1_network_entry(void) {
    // Wait for Core 0 to finish timing initialization
    while (!core0_ready) {
        tight_loop_contents();
    }

    printf("[Core 1] Network core started\n");

    // Initialize network (W5500 Ethernet)
    printf("[Core 1] Initializing network...\n");
    if (!network_init()) {
        printf("[Core 1] FATAL: Network init failed\n");
        while (1) {
            sleep_ms(1000);
        }
    }

    // Initialize PTP protocol
    printf("[Core 1] Initializing PTP grandmaster...\n");
    if (!ptp_grandmaster_init()) {
        printf("[Core 1] FATAL: PTP init failed\n");
        while (1) {
            sleep_ms(1000);
        }
    }

    printf("[Core 1] Network and PTP ready\n");
    printf("[Core 1] Entering main loop...\n");

    uint64_t last_stats_time = time_us_64();

    uint32_t loop_counter = 0;

    while (true) {
        loop_counter++;

        // Print loop counter every 100 iterations for debugging
        if (loop_counter % 100 == 0) {
            printf("[Core 1] Loop iteration: %lu\n", (unsigned long)loop_counter);
        }

        // DEBUG: Test which function is hanging
        // network_poll();
        // ptp_grandmaster_process();

        // Print PTP stats every 10 seconds
        uint64_t now_us = time_us_64();
        if (now_us - last_stats_time >= 10000000) {  // Every 10 seconds
            uint32_t sync_count, followup_count, delay_resp_count, active_slaves;
            ptp_grandmaster_get_stats_extended(NULL, &sync_count, &followup_count,
                                               &delay_resp_count, &active_slaves);

            char ip_addr[16];
            network_get_ip_str(ip_addr, sizeof(ip_addr));

            printf("[Core 1] Loop:%lu Network:%s IP:%s PTP: Sync=%lu FollowUp=%lu DelayResp=%lu Slaves=%lu\n",
                   (unsigned long)loop_counter,
                   network_is_connected() ? "UP" : "DOWN",
                   ip_addr,
                   (unsigned long)sync_count,
                   (unsigned long)followup_count,
                   (unsigned long)delay_resp_count,
                   (unsigned long)active_slaves);

            last_stats_time = now_us;
        }

        // Small delay - balance network responsiveness with Core 0 timing
        sleep_ms(100);  // Increase to 100ms to reduce contention
    }
}

/**
 * Core 0 Main: Timing and Discipline
 */
void core0_timing_main(void) {
    printf("[Core 0] Timing core running\n");

    uint64_t last_stats_time_us = time_us_64();
    uint32_t last_pps_count = 0;

    while (true) {
        // Print timing stats every 10 seconds
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
            double crystal_ppm = (double)crystal_error_ns / 1000.0;

            // TEMPORARILY DISABLED: Remove printf to eliminate spinlock contention
            // printf("[Core 0] PPS:%lu GPS:%llus crystal_err:%+ldns (%+.3fppm) interp_err:%+lldns lock:%s\n",
            //        core1_stats.pps_count,
            //        (unsigned long long)gps_seconds,
            //        (long)crystal_error_ns,
            //        crystal_ppm,
            //        (long long)interpolation_error_ns,
            //        core1_stats.locked ? "YES" : "NO ");

            last_stats_time_us = now_us;
        }

        // Minimal sleep to yield CPU
        sleep_ms(100);
    }
}

/**
 * Main: Initialize and launch dual-core system
 */
int main() {
    // Overclock to 250 MHz (both cores)
    set_sys_clock_khz(250000, true);

    // Initialize stdio (for startup messages from both cores)
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== DUAL-CORE PTP GRANDMASTER ===\n");
    printf("Core 0: Timing/Discipline (GPS-locked)\n");
    printf("Core 1: Network/PTP (packet processing)\n");
    printf("System clock: 250 MHz\n\n");

    // Initialize debug GPIOs
    gpio_init(DEBUG_PPS_PIN);
    gpio_set_dir(DEBUG_PPS_PIN, GPIO_OUT);
    gpio_put(DEBUG_PPS_PIN, 0);

    printf("[Core 0] Initializing GPS discipline...\n");

    // Initialize GPS module (NMEA parsing + PPS on PIO1)
    gps_init(GPS_UART_ID, GPS_TX_PIN, GPS_RX_PIN, GPS_PPS_PIN, GPS_PIO, GPS_SM);

    // Initialize GPS discipline system (V3 - GPIO pin sync, no DMA)
    if (!discipline_init_v3()) {
        printf("[Core 0] FATAL: Discipline init failed\n");
        while (1) {
            sleep_ms(1000);
        }
    }

    printf("[Core 0] GPS discipline ready\n");
    printf("[Core 0] Launching Core 1...\n");

    // Signal that Core 0 is ready
    core0_ready = true;

    // Launch Core 1 for network processing
    multicore_launch_core1(core1_network_entry);

    printf("[Core 0] Core 1 launched successfully\n\n");

    // Core 0 continues with timing loop
    core0_timing_main();

    return 0;
}

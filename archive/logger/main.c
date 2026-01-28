/**
 * Phase Logger Main Program (Core 0)
 *
 * Measures phase offset between Grandmaster and Slave 100 PPS outputs
 * relative to GPS reference with nanosecond precision.
 *
 * Hardware:
 *   - GPS module on UART0 (GPIO 0/1)
 *   - GPS PPS on GPIO 16
 *   - Grandmaster 100 PPS on GPIO 17
 *   - Slave 100 PPS on GPIO 18
 *
 * Output: CSV format via USB serial
 */

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/uart.h"
#include "shared_state.h"
#include "pio_timestamp.h"
#include "gps_simple.h"
#include <stdio.h>
#include <string.h>

// GPS UART configuration
#define GPS_UART_ID uart0
#define GPS_UART_TX_PIN 0
#define GPS_UART_RX_PIN 1
#define GPS_UART_BAUD 9600

// Forward declaration for Core 1 entry
void core1_entry(void);

/**
 * Initialize GPS UART for NMEA parsing
 */
static void gps_uart_init(void) {
    // Initialize UART
    uart_init(GPS_UART_ID, GPS_UART_BAUD);

    // Set GPIO functions
    gpio_set_function(GPS_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_UART_RX_PIN, GPIO_FUNC_UART);

    // Set UART format: 8N1
    uart_set_format(GPS_UART_ID, 8, 1, UART_PARITY_NONE);

    // Enable UART FIFO
    uart_set_fifo_enabled(GPS_UART_ID, true);
}

/**
 * Process GPS NMEA data
 */
static void gps_process_uart(void) {
    while (uart_is_readable(GPS_UART_ID)) {
        char c = uart_getc(GPS_UART_ID);
        gps_process_char(c);
    }
}

/**
 * Update GPS status in shared state
 */
static void update_gps_status(void) {
    gps_status.gps_fix = gps_has_fix();
    gps_status.gps_sats = gps_get_satellites();

    // Get UTC time if available
    char time_str[32];
    char date_str[32];
    if (gps_get_utc_time(time_str, sizeof(time_str)) &&
        gps_get_date(date_str, sizeof(date_str))) {
        strncpy((char *)gps_status.gps_utc_time, time_str, sizeof(gps_status.gps_utc_time) - 1);
        strncpy((char *)gps_status.gps_date, date_str, sizeof(gps_status.gps_date) - 1);
        gps_status.time_valid = true;
    } else {
        gps_status.time_valid = false;
    }
}

/**
 * Print CSV header
 */
static void print_csv_header(void) {
    printf("# Phase Logger - Hardware Timestamp Capture\n");
    printf("# Resolution: %lu ns per tick\n", (unsigned long)pio_timestamp_get_ns_per_tick());
    printf("# PIO Clock: %lu Hz\n", (unsigned long)pio_timestamp_get_freq_hz());
    printf("#\n");
    printf("# CSV Format:\n");
    printf("#   Timestamp(UTC)   - GPS time when measurement was taken\n");
    printf("#   Fix              - GPS fix status (YES/NO)\n");
    printf("#   Sats             - Number of GPS satellites\n");
    printf("#   GM_Phase(ns)     - Grandmaster phase offset from GPS (0-10ms)\n");
    printf("#   Slave_Phase(ns)  - Slave phase offset from GPS (0-10ms)\n");
    printf("#   GM-Slave(ns)     - Direct phase difference (what scope shows)\n");
    printf("#   GM_Count         - GM pulse counter (continuous)\n");
    printf("#   Slave_Count      - Slave pulse counter (continuous)\n");
    printf("#   Samples          - Total samples processed\n");
    printf("#\n");
    printf("Timestamp,Fix,Sats,GM_Phase_ns,Slave_Phase_ns,GM_Slave_ns,GM_Count,Slave_Count,Samples\n");
}

/**
 * Output measurement as CSV
 */
static void output_csv_measurement(void) {
    // Output timestamp (GPS UTC time or sample count if GPS not locked)
    if (gps_status.time_valid) {
        printf("%s,", gps_status.gps_utc_time);
    } else {
        printf("NO_GPS_%lu,", (unsigned long)phase_measurement.sample_count);
    }

    // GPS fix status
    printf("%s,", gps_status.gps_fix ? "YES" : "NO");

    // Satellite count
    printf("%d,", gps_status.gps_sats);

    // Phase measurements
    printf("%lld,", (long long)phase_measurement.gm_phase_offset_ns);
    printf("%lld,", (long long)phase_measurement.slave_phase_offset_ns);
    printf("%lld,", (long long)phase_measurement.gm_slave_phase_diff_ns);

    // Pulse counters
    printf("%lu,", (unsigned long)phase_measurement.gm_pps_count);
    printf("%lu,", (unsigned long)phase_measurement.slave_pps_count);

    // Sample count
    printf("%lu\n", (unsigned long)phase_measurement.sample_count);
}

int main(void) {
    // Initialize stdio for USB serial
    stdio_init_all();

    // Wait a moment for USB serial to stabilize
    sleep_ms(2000);

    printf("\n\n=== Phase Logger Starting ===\n\n");

    // Initialize GPS UART
    gps_uart_init();
    gps_init();
    printf("GPS UART initialized on GPIO%d/%d @ %d baud\n",
           GPS_UART_TX_PIN, GPS_UART_RX_PIN, GPS_UART_BAUD);

    // Initialize PIO timestamp capture (3 state machines)
    pio_timestamp_init();

    // Launch Core 1 (timestamp processor)
    multicore_launch_core1(core1_entry);

    printf("\nWaiting for GPS fix...\n");

    // Wait for GPS fix before starting logging
    while (!gps_has_fix()) {
        gps_process_uart();
        update_gps_status();
        sleep_ms(100);

        // Print GPS status every 5 seconds while waiting
        static uint32_t last_status_ms = 0;
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_status_ms >= 5000) {
            printf("  Waiting for GPS... Sats: %d\n", gps_status.gps_sats);
            last_status_ms = now_ms;
        }
    }

    printf("\nGPS Fix Acquired!\n");
    printf("  Satellites: %d\n", gps_status.gps_sats);
    printf("  UTC Time: %s\n", gps_status.gps_utc_time);
    printf("  Date: %s\n\n", gps_status.gps_date);

    // Print CSV header
    print_csv_header();

    uint32_t last_sample_count = 0;
    uint32_t last_status_update_ms = 0;
    uint32_t last_diagnostic_ms = 0;

    // Main logging loop
    while (true) {
        // Process GPS NMEA data
        gps_process_uart();

        // Update GPS status every 100ms
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_status_update_ms >= 100) {
            update_gps_status();
            last_status_update_ms = now_ms;
        }

        // Diagnostic output every 5 seconds (before CSV data starts flowing)
        if (phase_measurement.sample_count == 0 && now_ms - last_diagnostic_ms >= 5000) {
            printf("# Waiting for first measurement from Core 1...\n");
            last_diagnostic_ms = now_ms;
        }

        // Output CSV when new measurement available
        // Throttle to 1 Hz (every 100th 100 PPS pulse)
        if (phase_measurement.data_valid &&
            phase_measurement.sample_count > last_sample_count &&
            (phase_measurement.slave_pps_count % 100) == 0) {

            output_csv_measurement();
            last_sample_count = phase_measurement.sample_count;
        }

        // Small delay to prevent busy-wait
        sleep_ms(1);
    }

    return 0;
}

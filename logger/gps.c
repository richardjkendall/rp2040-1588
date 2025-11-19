/**
 * GPS Module Implementation
 */

#include "gps.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "pico/time.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "pps_capture.pio.h"

// Module state
static struct {
    uart_inst_t *uart;
    PIO pio;
    uint sm;
    uint pps_pin;

    // PPS data
    volatile uint64_t last_pps_timestamp_us;
    volatile bool pps_available;

    // GPS data
    gps_data_t gps_data;

    // NMEA parsing buffer
    char nmea_buffer[128];
    uint8_t nmea_index;
} gps_state;

// PIO IRQ handler - called on GPS PPS rising edge
static void gps_pps_irq_handler() {
    // Check if our PIO triggered the interrupt
    if (pio_interrupt_get(gps_state.pio, 0)) {
        // Clear the interrupt
        pio_interrupt_clear(gps_state.pio, 0);

        // Capture timestamp immediately
        gps_state.last_pps_timestamp_us = time_us_64();
        gps_state.pps_available = true;
    }
}

// Parse NMEA checksum
static bool nmea_validate_checksum(const char *sentence) {
    if (sentence[0] != '$') return false;

    const char *star = strchr(sentence, '*');
    if (!star) return false;

    // Calculate checksum (XOR of all characters between $ and *)
    uint8_t checksum = 0;
    for (const char *p = sentence + 1; p < star; p++) {
        checksum ^= *p;
    }

    // Parse provided checksum
    uint8_t provided = 0;
    sscanf(star + 1, "%2hhx", &provided);

    return checksum == provided;
}

// Parse $GPRMC sentence
// Format: $GPRMC,hhmmss.sss,A,,,,,,,,,*hh
// We mainly care about: time and validity status
static void parse_gprmc(const char *sentence) {
    char status;
    int hours, minutes, seconds, milliseconds;

    // Simple parsing - extract time and status
    int parsed = sscanf(sentence, "$GPRMC,%2d%2d%2d.%3d,%c",
                       &hours, &minutes, &seconds, &milliseconds, &status);

    if (parsed >= 5) {
        gps_state.gps_data.hours = hours;
        gps_state.gps_data.minutes = minutes;
        gps_state.gps_data.seconds = seconds;
        gps_state.gps_data.milliseconds = milliseconds;
        gps_state.gps_data.valid = (status == 'A'); // A = valid, V = invalid
    }
}

// Parse $GPGGA sentence
// Format: $GPGGA,hhmmss.sss,,,,,Q,SS,...*hh
// Q = fix quality (0=none, 1=GPS, 2=DGPS)
// SS = number of satellites
static void parse_gpgga(const char *sentence) {
    char *token;
    char sentence_copy[128];
    strncpy(sentence_copy, sentence, sizeof(sentence_copy) - 1);
    sentence_copy[sizeof(sentence_copy) - 1] = '\0';

    int field = 0;
    token = strtok(sentence_copy, ",");

    while (token != NULL && field < 8) {
        switch (field) {
            case 1: // Time (hhmmss.sss)
                if (strlen(token) >= 6) {
                    int hours, minutes, seconds, milliseconds = 0;
                    sscanf(token, "%2d%2d%2d.%3d", &hours, &minutes, &seconds, &milliseconds);
                    gps_state.gps_data.hours = hours;
                    gps_state.gps_data.minutes = minutes;
                    gps_state.gps_data.seconds = seconds;
                    gps_state.gps_data.milliseconds = milliseconds;
                }
                break;
            case 6: // Fix quality
                {
                    int quality = atoi(token);
                    if (quality == 0) {
                        gps_state.gps_data.fix_status = GPS_FIX_NONE;
                    } else if (quality >= 1) {
                        gps_state.gps_data.fix_status = GPS_FIX_3D;
                    }
                }
                break;
            case 7: // Number of satellites
                gps_state.gps_data.satellites = atoi(token);
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }
}

// Process complete NMEA sentence
static void process_nmea_sentence(const char *sentence) {
    // Validate checksum
    if (!nmea_validate_checksum(sentence)) {
        return;
    }

    // Parse based on sentence type
    if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        parse_gprmc(sentence);
    } else if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        parse_gpgga(sentence);
    }
}

void gps_init(uart_inst_t *uart_id, uint tx_pin, uint rx_pin, uint pps_pin, PIO pio, uint sm) {
    gps_state.uart = uart_id;
    gps_state.pio = pio;
    gps_state.sm = sm;
    gps_state.pps_pin = pps_pin;
    gps_state.pps_available = false;
    gps_state.last_pps_timestamp_us = 0;
    gps_state.nmea_index = 0;

    // Initialize GPS data
    memset(&gps_state.gps_data, 0, sizeof(gps_data_t));

    // Initialize UART for NMEA sentences (9600 baud, 8N1)
    uart_init(uart_id, 9600);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    uart_set_format(uart_id, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart_id, true);

    // Initialize PIO for PPS capture
    uint offset = pio_add_program(pio, &pps_capture_program);
    pps_capture_program_init(pio, sm, offset, pps_pin);

    // Set up PIO IRQ handler
    uint pio_irq = (pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0;
    irq_set_exclusive_handler(pio_irq, gps_pps_irq_handler);
    irq_set_enabled(pio_irq, true);
    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);

    printf("GPS ready (TX=%d RX=%d PPS=%d)\n", tx_pin, rx_pin, pps_pin);
}

bool gps_get_pps(gps_pps_t *pps) {
    if (!gps_state.pps_available) {
        return false;
    }

    // Copy PPS data
    pps->timestamp_us = gps_state.last_pps_timestamp_us;
    pps->valid = gps_has_fix();

    // Clear flag
    gps_state.pps_available = false;

    return true;
}

void gps_get_data(gps_data_t *data) {
    memcpy(data, &gps_state.gps_data, sizeof(gps_data_t));
}

void gps_process() {
    // Read available UART data and process NMEA sentences
    while (uart_is_readable(gps_state.uart)) {
        char c = uart_getc(gps_state.uart);

        // Handle newline - process complete sentence
        if (c == '\n' || c == '\r') {
            if (gps_state.nmea_index > 0) {
                gps_state.nmea_buffer[gps_state.nmea_index] = '\0';
                process_nmea_sentence(gps_state.nmea_buffer);
                gps_state.nmea_index = 0;
            }
        }
        // Add character to buffer
        else if (gps_state.nmea_index < sizeof(gps_state.nmea_buffer) - 1) {
            gps_state.nmea_buffer[gps_state.nmea_index++] = c;
        }
        // Buffer overflow - reset
        else {
            gps_state.nmea_index = 0;
        }
    }
}

bool gps_has_fix() {
    return gps_state.gps_data.fix_status == GPS_FIX_3D &&
           gps_state.gps_data.satellites >= 3;
}

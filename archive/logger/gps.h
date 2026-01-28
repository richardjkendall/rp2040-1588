/**
 * GPS Module
 *
 * Handles GPS PPS signal capture and NMEA sentence parsing for the
 * Adafruit Ultimate GPS Breakout v3 (MTK3339 chipset).
 *
 * Features:
 * - PIO-based PPS edge detection with hardware timestamps
 * - NMEA sentence parsing ($GPRMC, $GPGGA)
 * - GPS fix validation
 * - UTC time extraction
 */

#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

// GPS fix status
typedef enum {
    GPS_FIX_NONE = 0,
    GPS_FIX_INVALID = 1,
    GPS_FIX_2D = 2,
    GPS_FIX_3D = 3
} gps_fix_status_t;

// GPS data structure
typedef struct {
    bool valid;                  // Is GPS data valid?
    gps_fix_status_t fix_status; // Current fix status
    uint8_t satellites;          // Number of satellites
    uint8_t hours;               // UTC hours (0-23)
    uint8_t minutes;             // UTC minutes (0-59)
    uint8_t seconds;             // UTC seconds (0-59)
    uint16_t milliseconds;       // UTC milliseconds (0-999)
} gps_data_t;

// PPS timestamp structure
typedef struct {
    uint64_t timestamp_us;       // Hardware timer value in microseconds
    bool valid;                  // Is this timestamp valid?
} gps_pps_t;

/**
 * Initialize GPS module
 *
 * @param uart_id UART instance (uart0 or uart1)
 * @param tx_pin GPIO pin for UART TX (connects to GPS RX)
 * @param rx_pin GPIO pin for UART RX (connects to GPS TX)
 * @param pps_pin GPIO pin for PPS input
 * @param pio PIO instance for PPS capture
 * @param sm State machine number for PPS capture
 */
void gps_init(uart_inst_t *uart_id, uint tx_pin, uint rx_pin, uint pps_pin, PIO pio, uint sm);

/**
 * Get the latest GPS PPS timestamp
 *
 * @param pps Pointer to structure to receive PPS data
 * @return true if new PPS event available, false otherwise
 */
bool gps_get_pps(gps_pps_t *pps);

/**
 * Get current GPS data (fix status, satellite count, UTC time)
 *
 * @param data Pointer to structure to receive GPS data
 */
void gps_get_data(gps_data_t *data);

/**
 * Process incoming GPS data (call regularly from main loop)
 * Reads UART and parses NMEA sentences
 */
void gps_process();

/**
 * Check if GPS has a valid fix
 *
 * @return true if GPS has valid 3D fix, false otherwise
 */
bool gps_has_fix();

#endif // GPS_H

/**
 * Simple GPS NMEA Parser (Logger Version)
 *
 * Simplified GPS parser without PIO dependencies.
 * Only parses NMEA sentences for time, date, fix status, and satellite count.
 */

#ifndef GPS_SIMPLE_H
#define GPS_SIMPLE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * Initialize GPS parser
 */
void gps_init(void);

/**
 * Process one character from GPS NMEA stream
 *
 * @param c Character from GPS UART
 */
void gps_process_char(char c);

/**
 * Check if GPS has a valid fix
 *
 * @return true if GPS has fix, false otherwise
 */
bool gps_has_fix(void);

/**
 * Get number of satellites
 *
 * @return Satellite count
 */
uint8_t gps_get_satellites(void);

/**
 * Get UTC time string
 *
 * @param buffer Buffer to store time string
 * @param size Buffer size
 * @return true if time is valid, false otherwise
 */
bool gps_get_utc_time(char *buffer, size_t size);

/**
 * Get date string
 *
 * @param buffer Buffer to store date string
 * @param size Buffer size
 * @return true if date is valid, false otherwise
 */
bool gps_get_date(char *buffer, size_t size);

#endif // GPS_SIMPLE_H

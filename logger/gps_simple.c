/**
 * Simple GPS NMEA Parser Implementation
 */

#include "gps_simple.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Debug flag - set to 1 to enable NMEA sentence logging
#define DEBUG_NMEA 0

#define MAX_NMEA_LENGTH 128

// GPS state
static struct {
    char nmea_buffer[MAX_NMEA_LENGTH];
    uint8_t nmea_index;
    bool has_fix;
    uint8_t satellites;
    char utc_time[16];
    char date[12];
    bool time_valid;
    bool date_valid;
} gps_state = {0};

/**
 * Parse CSV fields from NMEA sentence, preserving empty fields
 * Returns number of fields parsed
 */
static int parse_csv_fields(const char *sentence, char fields[][32], int max_fields) {
    int field_idx = 0;
    int char_idx = 0;

    for (const char *p = sentence; *p && field_idx < max_fields; p++) {
        if (*p == ',' || *p == '*' || *p == '\r' || *p == '\n') {
            // End of field
            fields[field_idx][char_idx] = '\0';
            field_idx++;
            char_idx = 0;
        } else if (char_idx < 31) {
            // Add character to current field
            fields[field_idx][char_idx++] = *p;
        }
    }

    // Terminate last field if any
    if (char_idx > 0 && field_idx < max_fields) {
        fields[field_idx][char_idx] = '\0';
        field_idx++;
    }

    return field_idx;
}

/**
 * Parse GPGGA sentence for fix and satellites
 * $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
 */
static void parse_gpgga(const char *sentence) {
#if DEBUG_NMEA
    printf("  >> INSIDE parse_gpgga(), sentence='%s'\n", sentence);
#endif
    char fields[15][32];
    int field_count = parse_csv_fields(sentence, fields, 15);

#if DEBUG_NMEA
    printf("  GPGGA field_count=%d\n", field_count);
    for (int i = 0; i < field_count && i < 8; i++) {
        printf("    fields[%d]='%s'\n", i, fields[i]);
    }
#endif

    if (field_count >= 8) {
        // Field 1: UTC time
        if (strlen(fields[1]) >= 6) {
            snprintf(gps_state.utc_time, sizeof(gps_state.utc_time),
                     "%c%c:%c%c:%c%c.%s",
                     fields[1][0], fields[1][1],
                     fields[1][2], fields[1][3],
                     fields[1][4], fields[1][5],
                     fields[1] + 6);
            gps_state.time_valid = true;
#if DEBUG_NMEA
            printf("  Parsed UTC: %s\n", gps_state.utc_time);
#endif
        }

        // Field 6: Fix quality (0=invalid, 1=GPS fix, 2=DGPS fix)
        if (strlen(fields[6]) > 0) {
            int fix_quality = atoi(fields[6]);
            gps_state.has_fix = (fix_quality > 0);
#if DEBUG_NMEA
            printf("  Fix quality: %d (has_fix=%d)\n", fix_quality, gps_state.has_fix);
#endif
        }

        // Field 7: Number of satellites (validate range)
        if (strlen(fields[7]) > 0) {
            int sats = atoi(fields[7]);
            if (sats >= 0 && sats <= 50) {  // Sanity check
                gps_state.satellites = (uint8_t)sats;
#if DEBUG_NMEA
                printf("  Satellites: %d\n", sats);
#endif
            }
        }
    }
}

/**
 * Parse GPRMC sentence for date
 * $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
 */
static void parse_gprmc(const char *sentence) {
    char fields[12][32];
    int field_count = parse_csv_fields(sentence, fields, 12);

    if (field_count >= 10) {
        // Field 9: Date (DDMMYY)
        if (strlen(fields[9]) == 6) {
            snprintf(gps_state.date, sizeof(gps_state.date),
                     "20%c%c-%c%c-%c%c",
                     fields[9][4], fields[9][5],  // Year
                     fields[9][2], fields[9][3],  // Month
                     fields[9][0], fields[9][1]); // Day
            gps_state.date_valid = true;
#if DEBUG_NMEA
            printf("  Parsed Date: %s\n", gps_state.date);
#endif
        }
    }
}

/**
 * Parse complete NMEA sentence
 */
static void parse_nmea_sentence(const char *sentence) {
#if DEBUG_NMEA
    printf("NMEA: %s", sentence);  // Already has \n
#endif

    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
#if DEBUG_NMEA
        printf("  >> Calling parse_gpgga()\n");
#endif
        parse_gpgga(sentence);
    } else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
#if DEBUG_NMEA
        printf("  >> Calling parse_gprmc()\n");
#endif
        parse_gprmc(sentence);
    }
}

void gps_init(void) {
    memset(&gps_state, 0, sizeof(gps_state));
}

void gps_process_char(char c) {
    // Look for start of sentence
    if (c == '$') {
        gps_state.nmea_index = 0;
        gps_state.nmea_buffer[gps_state.nmea_index++] = c;
        return;
    }

    // Accumulate sentence
    if (gps_state.nmea_index > 0 && gps_state.nmea_index < MAX_NMEA_LENGTH - 1) {
        gps_state.nmea_buffer[gps_state.nmea_index++] = c;

        // End of sentence
        if (c == '\n') {
            gps_state.nmea_buffer[gps_state.nmea_index] = '\0';
            parse_nmea_sentence(gps_state.nmea_buffer);
            gps_state.nmea_index = 0;
        }
    }
}

bool gps_has_fix(void) {
    return gps_state.has_fix;
}

uint8_t gps_get_satellites(void) {
    return gps_state.satellites;
}

bool gps_get_utc_time(char *buffer, size_t size) {
    if (gps_state.time_valid && buffer && size > 0) {
        strncpy(buffer, gps_state.utc_time, size - 1);
        buffer[size - 1] = '\0';
        return true;
    }
    return false;
}

bool gps_get_date(char *buffer, size_t size) {
    if (gps_state.date_valid && buffer && size > 0) {
        strncpy(buffer, gps_state.date, size - 1);
        buffer[size - 1] = '\0';
        return true;
    }
    return false;
}

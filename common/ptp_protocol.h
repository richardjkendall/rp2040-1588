/**
 * PTP (IEEE 1588-2008) Protocol Definitions
 *
 * Simplified PTP implementation for GPS-disciplined grandmaster.
 * Implements only the messages needed for one-way time distribution:
 * - Announce (grandmaster declaration)
 * - Sync (time synchronization event)
 * - Follow_Up (precise timestamp)
 */

#ifndef PTP_PROTOCOL_H
#define PTP_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

// PTP message types
#define PTP_MSGTYPE_SYNC 0x00
#define PTP_MSGTYPE_DELAY_REQ 0x01
#define PTP_MSGTYPE_FOLLOW_UP 0x08
#define PTP_MSGTYPE_DELAY_RESP 0x09
#define PTP_MSGTYPE_ANNOUNCE 0x0B

// PTP version
#define PTP_VERSION 0x02

// PTP flag field bits
#define PTP_FLAG_TWO_STEP (1 << 9)      // Two-step clock (using Follow_Up)
#define PTP_FLAG_PTP_TIMESCALE (1 << 3) // PTP timescale (not ARB)

// PTP control field values
#define PTP_CONTROL_SYNC 0x00
#define PTP_CONTROL_DELAY_REQ 0x01
#define PTP_CONTROL_FOLLOW_UP 0x02
#define PTP_CONTROL_DELAY_RESP 0x03
#define PTP_CONTROL_OTHER 0x05

// PTP time source (for Announce)
#define PTP_TIMESOURCE_GPS 0x20

// PTP clock class
#define PTP_CLOCK_CLASS_GPS_LOCKED 6  // Locked to GPS

// PTP clock accuracy
#define PTP_CLOCK_ACCURACY_100NS 0x20  // Within 100 nanoseconds

// Message lengths
#define PTP_HEADER_LENGTH 34
#define PTP_ANNOUNCE_LENGTH 64
#define PTP_SYNC_LENGTH 44
#define PTP_FOLLOW_UP_LENGTH 44
#define PTP_DELAY_REQ_LENGTH 44
#define PTP_DELAY_RESP_LENGTH 54

/**
 * PTP Timestamp (TAI - 10 byte format)
 * 6 bytes seconds + 4 bytes nanoseconds
 */
typedef struct {
    uint16_t seconds_hi;      // Upper 16 bits of seconds
    uint32_t seconds_lo;      // Lower 32 bits of seconds
    uint32_t nanoseconds;     // Nanoseconds (0-999,999,999)
} __attribute__((packed)) ptp_timestamp_t;

/**
 * PTP Port Identity (10 bytes)
 * Identifies a specific port on a PTP clock
 */
typedef struct {
    uint8_t clock_identity[8];  // Clock identifier (usually MAC-based)
    uint16_t port_number;        // Port number (network byte order)
} __attribute__((packed)) ptp_port_identity_t;

/**
 * PTP Common Header (34 bytes)
 * Present in all PTP messages
 */
typedef struct {
    uint8_t message_type;           // Message type (Sync, Announce, etc.)
    uint8_t version_ptp;            // PTP version (0x02)
    uint16_t message_length;        // Total message length
    uint8_t domain_number;          // PTP domain (0 = default)
    uint8_t reserved1;
    uint16_t flag_field;            // Bit flags
    int64_t correction_field;       // Nanosecond correction
    uint32_t reserved2;
    ptp_port_identity_t source_port_identity;
    uint16_t sequence_id;           // Sequence number
    uint8_t control_field;          // Message control
    int8_t log_message_interval;    // Log2 of message period
} __attribute__((packed)) ptp_header_t;

/**
 * PTP Announce Message (64 bytes total)
 */
typedef struct {
    ptp_header_t header;

    ptp_timestamp_t origin_timestamp;  // Not used for Announce
    int16_t current_utc_offset;        // Seconds between TAI and UTC (37 in 2024)
    uint8_t reserved;
    uint8_t grandmaster_priority1;     // Priority 1 (lower is better)
    uint8_t grandmaster_clock_class;   // Clock quality class
    uint8_t grandmaster_clock_accuracy; // Clock accuracy
    uint16_t grandmaster_clock_variance; // Clock variance (log)
    uint8_t grandmaster_priority2;     // Priority 2 (lower is better)
    uint8_t grandmaster_identity[8];   // Grandmaster clock ID
    uint16_t steps_removed;            // Hops from grandmaster (0 for GM)
    uint8_t time_source;               // Time source (GPS, atomic, etc.)
} __attribute__((packed)) ptp_announce_msg_t;

/**
 * PTP Sync Message (44 bytes total)
 */
typedef struct {
    ptp_header_t header;
    ptp_timestamp_t origin_timestamp;  // Approximate timestamp (corrected in Follow_Up)
} __attribute__((packed)) ptp_sync_msg_t;

/**
 * PTP Follow_Up Message (44 bytes total)
 */
typedef struct {
    ptp_header_t header;
    ptp_timestamp_t precise_origin_timestamp;  // Precise timestamp from Sync
} __attribute__((packed)) ptp_follow_up_msg_t;

/**
 * PTP Delay_Req Message (44 bytes total)
 * Sent by slave to request delay measurement
 */
typedef struct {
    ptp_header_t header;
    ptp_timestamp_t origin_timestamp;  // Not used (set to 0)
} __attribute__((packed)) ptp_delay_req_msg_t;

/**
 * PTP Delay_Resp Message (54 bytes total)
 * Sent by master in response to Delay_Req
 */
typedef struct {
    ptp_header_t header;
    ptp_timestamp_t receive_timestamp;     // When Delay_Req was received (t4)
    ptp_port_identity_t requesting_port_identity;  // Which slave this is for
} __attribute__((packed)) ptp_delay_resp_msg_t;

/**
 * Clock Identity (8 bytes)
 * Typically derived from MAC address
 */
typedef struct {
    uint8_t id[8];
} ptp_clock_identity_t;

// Helper functions

/**
 * Initialize PTP clock identity from MAC address
 * Uses EUI-64 format: MAC[0:2] + 0xFF + 0xFE + MAC[3:5]
 */
void ptp_init_clock_identity(ptp_clock_identity_t *id, const uint8_t mac[6]);

/**
 * Build PTP Announce message
 *
 * @param msg Output message buffer
 * @param clock_id Clock identity
 * @param domain PTP domain number (0 = default)
 * @param sequence_id Sequence number for this message
 */
void ptp_build_announce(ptp_announce_msg_t *msg,
                        const ptp_clock_identity_t *clock_id,
                        uint8_t domain,
                        uint16_t sequence_id);

/**
 * Build PTP Sync message
 *
 * @param msg Output message buffer
 * @param clock_id Clock identity
 * @param domain PTP domain number
 * @param sequence_id Sequence number for this message
 * @param timestamp_ns Approximate timestamp in nanoseconds
 */
void ptp_build_sync(ptp_sync_msg_t *msg,
                    const ptp_clock_identity_t *clock_id,
                    uint8_t domain,
                    uint16_t sequence_id,
                    uint64_t timestamp_ns);

/**
 * Build PTP Follow_Up message
 *
 * @param msg Output message buffer
 * @param clock_id Clock identity
 * @param domain PTP domain number
 * @param sequence_id Sequence number (must match corresponding Sync)
 * @param precise_timestamp_ns Precise timestamp from Sync send event
 */
void ptp_build_follow_up(ptp_follow_up_msg_t *msg,
                         const ptp_clock_identity_t *clock_id,
                         uint8_t domain,
                         uint16_t sequence_id,
                         uint64_t precise_timestamp_ns);

/**
 * Build PTP Delay_Req message
 *
 * @param msg Output message buffer
 * @param clock_id Clock identity
 * @param domain PTP domain number
 * @param sequence_id Sequence number for this message
 */
void ptp_build_delay_req(ptp_delay_req_msg_t *msg,
                         const ptp_clock_identity_t *clock_id,
                         uint8_t domain,
                         uint16_t sequence_id);

/**
 * Build PTP Delay_Resp message
 *
 * @param msg Output message buffer
 * @param clock_id Clock identity (grandmaster's)
 * @param domain PTP domain number
 * @param sequence_id Sequence number (from Delay_Req)
 * @param receive_timestamp_ns When Delay_Req was received (t4)
 * @param requesting_port_identity Port identity of requesting slave
 */
void ptp_build_delay_resp(ptp_delay_resp_msg_t *msg,
                          const ptp_clock_identity_t *clock_id,
                          uint8_t domain,
                          uint16_t sequence_id,
                          uint64_t receive_timestamp_ns,
                          const ptp_port_identity_t *requesting_port_identity);

/**
 * Convert nanosecond timestamp to PTP timestamp format
 *
 * @param timestamp Output PTP timestamp
 * @param ns Input timestamp in nanoseconds since epoch
 */
void ptp_ns_to_timestamp(ptp_timestamp_t *timestamp, uint64_t ns);

/**
 * Convert PTP timestamp to nanoseconds
 *
 * @param timestamp Input PTP timestamp
 * @return Nanoseconds since epoch
 */
uint64_t ptp_timestamp_to_ns(const ptp_timestamp_t *timestamp);

#endif // PTP_PROTOCOL_H

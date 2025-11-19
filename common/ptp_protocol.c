/**
 * PTP Protocol Implementation
 */

#include "ptp_protocol.h"
#include <string.h>
#include "pico/stdlib.h"

// Network byte order conversion
#define htons(x) __builtin_bswap16(x)
#define htonl(x) __builtin_bswap32(x)
#define htonll(x) __builtin_bswap64(x)
#define ntohs(x) __builtin_bswap16(x)
#define ntohl(x) __builtin_bswap32(x)
#define ntohll(x) __builtin_bswap64(x)

void ptp_init_clock_identity(ptp_clock_identity_t *id, const uint8_t mac[6]) {
    // EUI-64 format: MAC[0:2] + 0xFF + 0xFE + MAC[3:5]
    id->id[0] = mac[0];
    id->id[1] = mac[1];
    id->id[2] = mac[2];
    id->id[3] = 0xFF;
    id->id[4] = 0xFE;
    id->id[5] = mac[3];
    id->id[6] = mac[4];
    id->id[7] = mac[5];
}

void ptp_ns_to_timestamp(ptp_timestamp_t *timestamp, uint64_t ns) {
    // Convert nanoseconds to seconds and nanoseconds
    uint64_t seconds = ns / 1000000000ULL;
    uint32_t nanoseconds = ns % 1000000000ULL;

    // Split 48-bit seconds into upper 16 and lower 32 bits
    timestamp->seconds_hi = htons((uint16_t)(seconds >> 32));
    timestamp->seconds_lo = htonl((uint32_t)(seconds & 0xFFFFFFFF));
    timestamp->nanoseconds = htonl(nanoseconds);
}

uint64_t ptp_timestamp_to_ns(const ptp_timestamp_t *timestamp) {
    // Convert from network byte order (big-endian) to host (little-endian)
    uint64_t seconds = ((uint64_t)ntohs(timestamp->seconds_hi) << 32) |
                       (uint64_t)ntohl(timestamp->seconds_lo);
    uint32_t nanoseconds = ntohl(timestamp->nanoseconds);

    return seconds * 1000000000ULL + nanoseconds;
}

static void ptp_init_header(ptp_header_t *header,
                            uint8_t message_type,
                            uint16_t message_length,
                            const ptp_clock_identity_t *clock_id,
                            uint8_t domain,
                            uint16_t sequence_id,
                            uint8_t control_field,
                            int8_t log_interval) {
    memset(header, 0, sizeof(ptp_header_t));

    header->message_type = message_type;
    header->version_ptp = PTP_VERSION;
    header->message_length = htons(message_length);
    header->domain_number = domain;
    header->flag_field = htons(PTP_FLAG_TWO_STEP | PTP_FLAG_PTP_TIMESCALE);
    header->correction_field = 0;  // Grandmaster has no correction

    // Set source port identity
    memcpy(header->source_port_identity.clock_identity, clock_id->id, 8);
    header->source_port_identity.port_number = htons(1);  // Port 1

    header->sequence_id = htons(sequence_id);
    header->control_field = control_field;
    header->log_message_interval = log_interval;
}

void ptp_build_announce(ptp_announce_msg_t *msg,
                        const ptp_clock_identity_t *clock_id,
                        uint8_t domain,
                        uint16_t sequence_id) {
    memset(msg, 0, sizeof(ptp_announce_msg_t));

    // Initialize header
    ptp_init_header(&msg->header,
                    PTP_MSGTYPE_ANNOUNCE,
                    PTP_ANNOUNCE_LENGTH,
                    clock_id,
                    domain,
                    sequence_id,
                    PTP_CONTROL_OTHER,
                    0);  // 1 message per second = 2^0

    // Announce-specific fields
    msg->current_utc_offset = htons(37);  // TAI - UTC = 37 seconds (as of 2024)
    msg->grandmaster_priority1 = 128;     // Default priority
    msg->grandmaster_clock_class = PTP_CLOCK_CLASS_GPS_LOCKED;
    msg->grandmaster_clock_accuracy = PTP_CLOCK_ACCURACY_100NS;
    msg->grandmaster_clock_variance = htons(0xFFFF);  // Unknown variance
    msg->grandmaster_priority2 = 128;     // Default priority
    memcpy(msg->grandmaster_identity, clock_id->id, 8);
    msg->steps_removed = htons(0);        // We are the grandmaster
    msg->time_source = PTP_TIMESOURCE_GPS;
}

void ptp_build_sync(ptp_sync_msg_t *msg,
                    const ptp_clock_identity_t *clock_id,
                    uint8_t domain,
                    uint16_t sequence_id,
                    uint64_t timestamp_ns) {
    memset(msg, 0, sizeof(ptp_sync_msg_t));

    // Initialize header
    ptp_init_header(&msg->header,
                    PTP_MSGTYPE_SYNC,
                    PTP_SYNC_LENGTH,
                    clock_id,
                    domain,
                    sequence_id,
                    PTP_CONTROL_SYNC,
                    0);  // 1 message per second = 2^0

    // Origin timestamp (approximate - will be corrected in Follow_Up)
    ptp_ns_to_timestamp(&msg->origin_timestamp, timestamp_ns);
}

void ptp_build_follow_up(ptp_follow_up_msg_t *msg,
                         const ptp_clock_identity_t *clock_id,
                         uint8_t domain,
                         uint16_t sequence_id,
                         uint64_t precise_timestamp_ns) {
    memset(msg, 0, sizeof(ptp_follow_up_msg_t));

    // Initialize header
    ptp_init_header(&msg->header,
                    PTP_MSGTYPE_FOLLOW_UP,
                    PTP_FOLLOW_UP_LENGTH,
                    clock_id,
                    domain,
                    sequence_id,
                    PTP_CONTROL_FOLLOW_UP,
                    0);  // 1 message per second = 2^0

    // Precise origin timestamp
    ptp_ns_to_timestamp(&msg->precise_origin_timestamp, precise_timestamp_ns);
}

void ptp_build_delay_req(ptp_delay_req_msg_t *msg,
                         const ptp_clock_identity_t *clock_id,
                         uint8_t domain,
                         uint16_t sequence_id) {
    memset(msg, 0, sizeof(ptp_delay_req_msg_t));

    // Initialize header
    ptp_init_header(&msg->header,
                    PTP_MSGTYPE_DELAY_REQ,
                    PTP_DELAY_REQ_LENGTH,
                    clock_id,
                    domain,
                    sequence_id,
                    PTP_CONTROL_DELAY_REQ,
                    0);  // Log message interval (not critical for delay req)

    // Origin timestamp is not used for Delay_Req (set to 0)
    memset(&msg->origin_timestamp, 0, sizeof(ptp_timestamp_t));
}

void ptp_build_delay_resp(ptp_delay_resp_msg_t *msg,
                          const ptp_clock_identity_t *clock_id,
                          uint8_t domain,
                          uint16_t sequence_id,
                          uint64_t receive_timestamp_ns,
                          const ptp_port_identity_t *requesting_port_identity) {
    memset(msg, 0, sizeof(ptp_delay_resp_msg_t));

    // Initialize header
    ptp_init_header(&msg->header,
                    PTP_MSGTYPE_DELAY_RESP,
                    PTP_DELAY_RESP_LENGTH,
                    clock_id,
                    domain,
                    sequence_id,
                    PTP_CONTROL_DELAY_RESP,
                    0);  // Log message interval

    // Receive timestamp (t4 - when Delay_Req was received)
    ptp_ns_to_timestamp(&msg->receive_timestamp, receive_timestamp_ns);

    // Copy requesting port identity
    memcpy(&msg->requesting_port_identity, requesting_port_identity, sizeof(ptp_port_identity_t));
}

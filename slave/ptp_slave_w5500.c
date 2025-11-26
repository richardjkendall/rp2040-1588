/**
 * PTP Slave Protocol Handler - W5500 Ethernet
 *
 * Implements IEEE 1588 slave message handling with timestamp capture.
 */

#include "ptp_slave_w5500.h"
#include "shared_state_slave.h"
#include "ptp_protocol.h"
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"  // For pio_encode_* functions
#include "pico/stdlib.h"
#include <string.h>
#include <stdio.h>

// Helper: Convert host byte order to network byte order (16-bit)
static inline uint16_t htons(uint16_t x) {
    return ((x & 0xFF) << 8) | ((x >> 8) & 0xFF);
}

// Helper: Convert network byte order to host byte order (16-bit)
static inline uint16_t ntohs(uint16_t x) {
    return htons(x);  // Same operation
}

// External PIO counter access
#define DISCIPLINE_PIO pio0
#define COUNTER_SM 0

// PTP state
static struct {
    ptp_clock_identity_t clock_id;
    uint8_t domain;
    uint16_t delay_req_sequence;

    // Grandmaster info (learned from first Sync)
    uint8_t gm_mac[6];
    uint32_t gm_ip;
    bool gm_known;

    // Statistics
    uint32_t sync_count;
    uint32_t followup_count;
    uint32_t delay_req_count;
    uint32_t delay_resp_count;

    // Pending Sync (waiting for Follow_Up)
    uint16_t pending_sync_sequence;
    uint64_t pending_t2_ptp_ns;       // t2 from PTP clock
    uint64_t pending_t2_slave_us;     // t2 from system timer (reference)
    uint32_t pending_counter_value;
    int64_t pending_correction_sync_ns;
    bool sync_pending;
} ptp_state = {0};

/**
 * Initialize PTP slave protocol
 */
void ptp_slave_w5500_init(ptp_clock_identity_t *clock_id, uint8_t domain) {
    memcpy(&ptp_state.clock_id, clock_id, sizeof(ptp_clock_identity_t));
    ptp_state.domain = domain;
    ptp_state.delay_req_sequence = 0;
    ptp_state.gm_known = false;
    ptp_state.sync_pending = false;

    printf("PTP slave protocol initialized (domain %d)\n", domain);
}

/**
 * Handle received Sync message
 */
void ptp_slave_w5500_handle_sync(const ptp_sync_msg_t *sync, const udp_packet_t *udp,
                                 const uint8_t *src_mac, uint32_t src_ip) {
    // CRITICAL: Capture timestamps IMMEDIATELY
    // t2 = our PTP clock time when Sync arrived (NOT system timer!)
    extern uint64_t get_ptp_time_ns(void);
    uint64_t t2_ptp_ns = get_ptp_time_ns();
    uint64_t t2_slave_us = time_us_64();  // Also capture system time for reference

    // Atomic counter snapshot: Pause SM, read X, resume SM (for crystal characterization - disabled)
    pio_sm_set_enabled(DISCIPLINE_PIO, COUNTER_SM, false);
    pio_sm_exec(DISCIPLINE_PIO, COUNTER_SM, pio_encode_mov(pio_isr, pio_x));
    pio_sm_exec(DISCIPLINE_PIO, COUNTER_SM, pio_encode_push(false, false));
    uint32_t counter_value = pio_sm_get(DISCIPLINE_PIO, COUNTER_SM);
    pio_sm_set_enabled(DISCIPLINE_PIO, COUNTER_SM, true);

    // Learn grandmaster from first Sync
    if (!ptp_state.gm_known) {
        memcpy(ptp_state.gm_mac, src_mac, 6);
        ptp_state.gm_ip = src_ip;
        ptp_state.gm_known = true;
        printf("PTP grandmaster discovered: %u.%u.%u.%u\n",
               (src_ip >> 24) & 0xFF, (src_ip >> 16) & 0xFF,
               (src_ip >> 8) & 0xFF, src_ip & 0xFF);
    }

    // Convert sequence ID from network byte order
    uint16_t sync_seq = (sync->header.sequence_id >> 8) |
                       ((sync->header.sequence_id & 0xFF) << 8);

    // Extract correction field (nanoseconds in upper 48 bits)
    int64_t correction_ns = sync->header.correction_field >> 16;

    // Store pending Sync data (waiting for Follow_Up)
    ptp_state.pending_sync_sequence = sync_seq;
    ptp_state.pending_t2_ptp_ns = t2_ptp_ns;
    ptp_state.pending_t2_slave_us = t2_slave_us;
    ptp_state.pending_counter_value = counter_value;
    ptp_state.pending_correction_sync_ns = correction_ns;
    ptp_state.sync_pending = true;

    ptp_state.sync_count++;
}

/**
 * Handle received Follow_Up message
 */
void ptp_slave_w5500_handle_followup(const ptp_follow_up_msg_t *followup) {
    // Convert sequence ID from network byte order
    uint16_t followup_seq = (followup->header.sequence_id >> 8) |
                           ((followup->header.sequence_id & 0xFF) << 8);

    // Must match pending Sync
    if (!ptp_state.sync_pending || followup_seq != ptp_state.pending_sync_sequence) {
        printf("Follow_Up mismatch: expected seq %u, got %u\n",
               ptp_state.pending_sync_sequence, followup_seq);
        return;
    }

    // Extract t1 (master TX timestamp)
    uint64_t t1_master_ns = ptp_timestamp_to_ns(&followup->precise_origin_timestamp);

    // Add Follow_Up correction field to Sync correction
    int64_t correction_followup_ns = followup->header.correction_field >> 16;
    int64_t total_correction_ns = ptp_state.pending_correction_sync_ns + correction_followup_ns;

    // Pass to discipline module (Core 0)
    ptp_sync_data.t1_master_ns = t1_master_ns;
    ptp_sync_data.t2_ptp_ns = ptp_state.pending_t2_ptp_ns;
    ptp_sync_data.t2_slave_us = ptp_state.pending_t2_slave_us;
    ptp_sync_data.correction_sync_ns = total_correction_ns;
    ptp_sync_data.counter_at_sync = ptp_state.pending_counter_value;
    ptp_sync_data.sync_sequence = followup_seq;
    ptp_sync_data.sync_followup_ready = true;

    ptp_state.sync_pending = false;
    ptp_state.followup_count++;
}

/**
 * Handle received Delay_Resp message
 */
void ptp_slave_w5500_handle_delay_resp(const ptp_delay_resp_msg_t *delay_resp) {
    // Convert sequence ID from network byte order
    uint16_t delay_resp_seq = (delay_resp->header.sequence_id >> 8) |
                             ((delay_resp->header.sequence_id & 0xFF) << 8);

    // Extract t4 (master RX timestamp)
    uint64_t t4_master_ns = ptp_timestamp_to_ns(&delay_resp->receive_timestamp);

    // Extract correction field
    int64_t correction_ns = delay_resp->header.correction_field >> 16;

    // Pass to discipline module (Core 0)
    ptp_sync_data.t4_master_ns = t4_master_ns;
    ptp_sync_data.correction_delay_resp_ns = correction_ns;
    ptp_sync_data.delay_req_sequence = delay_resp_seq;
    ptp_sync_data.delay_resp_ready = true;

    ptp_state.delay_resp_count++;
}

/**
 * Send Delay_Req message to grandmaster
 */
uint16_t ptp_slave_w5500_send_delay_req(const uint8_t *dest_mac, uint32_t dest_ip,
                                        uint8_t *frame_buffer) {
    // Build Delay_Req message
    ptp_delay_req_msg_t delay_req;
    memset(&delay_req, 0, sizeof(delay_req));

    // Header
    delay_req.header.message_type = PTP_MSGTYPE_DELAY_REQ;
    delay_req.header.version_ptp = PTP_VERSION;
    delay_req.header.message_length = htons(sizeof(ptp_delay_req_msg_t));
    delay_req.header.domain_number = ptp_state.domain;
    delay_req.header.flag_field = 0;
    delay_req.header.correction_field = 0;
    memcpy(&delay_req.header.source_port_identity, &ptp_state.clock_id, 8);
    delay_req.header.source_port_identity.port_number = htons(1);
    delay_req.header.sequence_id = htons(ptp_state.delay_req_sequence);
    delay_req.header.control_field = PTP_CONTROL_DELAY_REQ;
    delay_req.header.log_message_interval = 0; // 1 second (2^0)

    // Origin timestamp not used for Delay_Req (set to 0)
    memset(&delay_req.origin_timestamp, 0, sizeof(ptp_timestamp_t));

    // Wrap in UDP/IP/Ethernet
    uint16_t frame_len = eth_build_udp(frame_buffer, dest_mac, dest_ip,
                                       319, 319, // PTP event port
                                       (uint8_t*)&delay_req, sizeof(delay_req));

    // Note: t3 timestamp will be captured by caller AFTER w5500_send_frame()
    // to account for SPI transfer and W5500 processing latency

    ptp_state.delay_req_sequence++;
    ptp_state.delay_req_count++;

    return frame_len;
}

/**
 * Get statistics
 */
void ptp_slave_w5500_get_stats(uint32_t *sync_count, uint32_t *followup_count,
                               uint32_t *delay_req_count, uint32_t *delay_resp_count,
                               bool *gm_known) {
    *sync_count = ptp_state.sync_count;
    *followup_count = ptp_state.followup_count;
    *delay_req_count = ptp_state.delay_req_count;
    *delay_resp_count = ptp_state.delay_resp_count;
    *gm_known = ptp_state.gm_known;
}

/**
 * Get grandmaster MAC address
 */
const uint8_t* ptp_slave_w5500_get_gm_mac(void) {
    return ptp_state.gm_mac;
}

/**
 * Get grandmaster IP address
 */
uint32_t ptp_slave_w5500_get_gm_ip(void) {
    return ptp_state.gm_ip;
}

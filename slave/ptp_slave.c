/**
 * PTP Slave Implementation
 * Receives PTP packets from grandmaster and extracts timestamps
 */

#include "ptp_slave.h"
#include "ptp_protocol.h"
#include "shared_state.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"

// PTP ports
#define PTP_EVENT_PORT 319
#define PTP_GENERAL_PORT 320

// PTP domain
#define PTP_DOMAIN 0

// Delay request interval (randomized between MIN and MAX to avoid collisions)
#define DELAY_REQ_MIN_INTERVAL_MS 1000  // 1 second minimum
#define DELAY_REQ_MAX_INTERVAL_MS 3000  // 3 seconds maximum

// PDV-based filtering threshold
// Reject measurements where path delay changes by more than 50ms (network jitter spike)
#define PDV_REJECT_THRESHOLD_NS 50000000  // 50ms

/**
 * Two-way PTP timing state
 * Tracks all four timestamps for offset and delay calculation
 */
typedef struct {
    // Timestamps
    uint64_t t1_ns;  // Master Sync transmission time (from Follow_Up)
    uint64_t t2_ns;  // Slave Sync reception time (local clock)
    uint64_t t3_ns;  // Slave Delay_Req transmission time (local clock)
    uint64_t t4_ns;  // Master Delay_Req reception time (from Delay_Resp)

    // Calculated values
    int64_t offset_ns;      // Clock offset (negative = slave ahead)
    int64_t path_delay_ns;  // One-way path delay
    int64_t pdv_ns;         // Path delay variation (jitter indicator)

    // Previous path delay for PDV calculation
    int64_t prev_path_delay_ns;

    // Delay_Req management
    uint16_t delay_req_sequence;
    bool waiting_for_delay_resp;
    uint32_t last_delay_req_ms;
    uint32_t next_delay_req_interval_ms;

    // Valid data flags
    bool have_sync_data;     // Have t1 and t2
    bool have_delay_data;    // Have t3 and t4
    bool offset_valid;       // Have valid offset calculation

} ptp_timing_t;

// PTP state
static struct {
    bool initialized;

    // UDP sockets
    struct udp_pcb *event_pcb;      // For Sync and Delay_Resp (port 319)
    struct udp_pcb *general_pcb;    // For Announce and Follow_Up (port 320)

    // Clock identity
    ptp_clock_identity_t clock_id;

    // Pending Sync (waiting for Follow_Up)
    bool sync_pending;
    uint16_t sync_sequence;
    uint64_t sync_receive_time_us;  // For backwards compat
    uint64_t sync_receive_time_ns;  // Actual t2 timestamp (continuous disciplined clock)

    // Timing state (two-way PTP)
    ptp_timing_t timing;

    // Statistics
    uint32_t sync_count;
    uint32_t announce_count;
    uint32_t followup_count;
    uint32_t delay_req_count;
    uint32_t delay_resp_count;
    uint32_t pdv_rejected;  // Measurements rejected due to excessive PDV
    uint32_t errors;

    // Debug throttling
    uint32_t last_offset_warning_ms;

} ptp_state = {0};

/**
 * Calculate offset and path delay from four timestamps
 */
static void calculate_offset_and_delay(void) {
    if (!ptp_state.timing.have_sync_data || !ptp_state.timing.have_delay_data) {
        return;  // Need all four timestamps
    }

    // Skip if t2 or t3 is 0 (first sync not yet processed)
    if (ptp_state.timing.t2_ns == 0 || ptp_state.timing.t3_ns == 0) {
        return;
    }

    // Master-to-slave delay (forward path)
    int64_t ms_delay = (int64_t)ptp_state.timing.t2_ns - (int64_t)ptp_state.timing.t1_ns;

    // Slave-to-master delay (reverse path)
    int64_t sm_delay = (int64_t)ptp_state.timing.t4_ns - (int64_t)ptp_state.timing.t3_ns;

    // One-way path delay (mean of forward and reverse)
    int64_t new_path_delay = (ms_delay + sm_delay) / 2;

    // Path delay variation (PDV) - change from previous measurement
    int64_t pdv_ns = 0;
    if (ptp_state.timing.offset_valid) {
        pdv_ns = new_path_delay - ptp_state.timing.prev_path_delay_ns;

        // Reject measurements with excessive PDV (network jitter spike)
        // This prevents corrupted measurements from affecting discipline
        int64_t abs_pdv = (pdv_ns < 0) ? -pdv_ns : pdv_ns;
        if (abs_pdv > PDV_REJECT_THRESHOLD_NS) {
            // This measurement has a jitter spike (>50ms path delay change)
            // Likely caused by WiFi retransmission or congestion - discard it
            ptp_state.pdv_rejected++;
            return;
        }
    }

    // Measurement is good - calculate offset and update state
    // Clock offset: how far ahead/behind we are from master
    // Positive offset = we're behind master (master time > slave time)
    // Negative offset = we're ahead of master (master time < slave time)
    int64_t offset_ns = (ms_delay - sm_delay) / 2;

    // Always store path delay and PDV (useful for network monitoring)
    ptp_state.timing.path_delay_ns = new_path_delay;
    ptp_state.timing.pdv_ns = pdv_ns;
    ptp_state.timing.prev_path_delay_ns = new_path_delay;

    // Only mark offset as valid if it's reasonable (within ±10 seconds)
    // Large offsets indicate time base mismatch (slave using boot timer vs master using GPS time)
    // Path delay is still useful for monitoring, but offset can't be used for discipline
    int64_t abs_offset = (offset_ns < 0) ? -offset_ns : offset_ns;
    if (abs_offset < 10000000000LL) {  // 10 seconds
        ptp_state.timing.offset_ns = offset_ns;
        ptp_state.timing.offset_valid = true;
    } else {
        // Offset too large - time bases not synchronized, can't use for discipline
        ptp_state.timing.offset_valid = false;
    }
}

/**
 * Send Delay_Req message to grandmaster
 */
static void send_delay_req(void) {
    ptp_delay_req_msg_t msg;

    // Build Delay_Req message
    ptp_build_delay_req(&msg, &ptp_state.clock_id, PTP_DOMAIN, ptp_state.timing.delay_req_sequence);

    // Allocate pbuf for Delay_Req
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(msg), PBUF_RAM);
    if (p == NULL) {
        printf("ERROR: Failed to allocate pbuf for Delay_Req\n");
        return;
    }

    // Copy message to pbuf
    memcpy(p->payload, &msg, sizeof(msg));

    // Get grandmaster address (from config or use multicast)
    ip_addr_t gm_addr;
    // TODO: Use configured grandmaster address or multicast
    // For now, assume broadcast/multicast will work
    IP4_ADDR(&gm_addr, 255, 255, 255, 255);  // Broadcast for now

    // Send Delay_Req via event socket
    err_t err = udp_sendto(ptp_state.event_pcb, p, &gm_addr, PTP_EVENT_PORT);
    pbuf_free(p);

    if (err != ERR_OK) {
        printf("ERROR: Failed to send Delay_Req (err=%d)\n", err);
        return;
    }

    // Record t3 timestamp (transmission time from our PTP estimate clock)
    // Use PTP estimate (same time base as t2 and grandmaster's t1/t4)
    if (core1_stats.first_sync_received) {
        ptp_state.timing.t3_ns = core1_stats.ptp_estimate_ns;
    } else {
        ptp_state.timing.t3_ns = 0;  // Not initialized yet
    }
    ptp_state.timing.have_delay_data = false;  // Waiting for t4 from Delay_Resp

    ptp_state.timing.delay_req_sequence++;
    ptp_state.timing.waiting_for_delay_resp = true;
    ptp_state.delay_req_count++;
}

/**
 * Callback for event messages (Sync, Delay_Resp) on port 319
 */
static void event_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                                const ip_addr_t *addr, u16_t port) {
    if (p->len < sizeof(ptp_header_t)) {
        pbuf_free(p);
        ptp_state.errors++;
        return;
    }

    ptp_header_t *header = (ptp_header_t *)p->payload;
    uint8_t msg_type = header->message_type & 0x0F;

    // Check if this is a Sync message
    if (msg_type == PTP_MSGTYPE_SYNC) {
        // Record when we received this Sync (t2)
        // Use PTP estimate clock - smooth, monotonic PTP time estimate (no phase jumps)
        uint64_t now_us = time_us_64();
        ptp_state.sync_receive_time_us = now_us;  // For backwards compat

        // Capture t2 using PTP estimate (same time base as grandmaster's t1/t4)
        if (core1_stats.first_sync_received) {
            ptp_state.sync_receive_time_ns = core1_stats.ptp_estimate_ns;
        } else {
            ptp_state.sync_receive_time_ns = 0;  // Not initialized yet
        }

        ptp_state.sync_sequence = ntohs(header->sequence_id);
        ptp_state.sync_pending = true;
        ptp_state.sync_count++;
    }

    pbuf_free(p);
}

/**
 * Callback for general messages (Announce, Follow_Up, Delay_Resp) on port 320
 */
static void general_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                                  const ip_addr_t *addr, u16_t port) {
    if (p->len < sizeof(ptp_header_t)) {
        pbuf_free(p);
        ptp_state.errors++;
        return;
    }

    ptp_header_t *header = (ptp_header_t *)p->payload;
    uint8_t msg_type = header->message_type & 0x0F;

    if (msg_type == PTP_MSGTYPE_ANNOUNCE) {
        ptp_state.announce_count++;
    }
    else if (msg_type == PTP_MSGTYPE_FOLLOW_UP) {
        // Follow_Up contains the precise timestamp for the Sync
        if (p->len < sizeof(ptp_follow_up_msg_t)) {
            pbuf_free(p);
            ptp_state.errors++;
            return;
        }

        ptp_follow_up_msg_t *followup = (ptp_follow_up_msg_t *)p->payload;
        uint16_t followup_sequence = ntohs(followup->header.sequence_id);

        // Check if this matches our pending Sync
        if (ptp_state.sync_pending && followup_sequence == ptp_state.sync_sequence) {
            // Extract precise timestamp from Follow_Up (t1)
            uint64_t ptp_time_ns = ptp_timestamp_to_ns(&followup->precise_origin_timestamp);

            // Store t1 and t2 for offset calculation
            ptp_state.timing.t1_ns = ptp_time_ns;
            // Use t2 from disciplined continuous clock (same time base as t3)
            ptp_state.timing.t2_ns = ptp_state.sync_receive_time_ns;
            ptp_state.timing.have_sync_data = true;

            // Pass timestamp to Core 1 for discipline
            // Write all data fields first
            ptp_sync_data.ptp_time_ns = ptp_time_ns;
            ptp_sync_data.local_time_us = ptp_state.sync_receive_time_us;
            ptp_sync_data.sequence = followup_sequence;

            // Also pass offset if available (for Core 1 to use instead of raw phase error)
            ptp_sync_data.offset_ns = ptp_state.timing.offset_ns;
            ptp_sync_data.offset_valid = ptp_state.timing.offset_valid;

            // Memory barrier to ensure all data writes complete before setting valid flag
            __compiler_memory_barrier();

            // Set valid flag LAST to signal Core 1 that data is ready
            ptp_sync_data.valid = true;

            ptp_state.sync_pending = false;
            ptp_state.followup_count++;

            // Try to calculate offset if we also have delay data
            calculate_offset_and_delay();
        }
    }
    else if (msg_type == PTP_MSGTYPE_DELAY_RESP && p->len >= sizeof(ptp_delay_resp_msg_t)) {
        if (ptp_state.timing.waiting_for_delay_resp) {
            ptp_delay_resp_msg_t *delay_resp = (ptp_delay_resp_msg_t *)p->payload;

            // Verify this Delay_Resp is for us (check requesting port identity)
            // Compare only the clock_identity portion (8 bytes), ignore port_number
            bool port_match = (memcmp(delay_resp->requesting_port_identity.clock_identity,
                                     ptp_state.clock_id.id,
                                     8) == 0);

            if (port_match) {
                // Extract t4 timestamp from Delay_Resp
                ptp_state.timing.t4_ns = ptp_timestamp_to_ns(&delay_resp->receive_timestamp);
                ptp_state.timing.have_delay_data = true;
                ptp_state.timing.waiting_for_delay_resp = false;
                ptp_state.delay_resp_count++;

                // Calculate offset and path delay now that we have all four timestamps
                calculate_offset_and_delay();
            }
        }
    }

    pbuf_free(p);
}

bool ptp_slave_init(void) {
    printf("Initializing PTP slave...\n");

    // Get MAC address for clock identity
    uint8_t mac[6];
    cyw43_hal_get_mac(CYW43_ITF_STA, mac);

    // Initialize clock identity from MAC address
    ptp_init_clock_identity(&ptp_state.clock_id, mac);

    // Initialize timing state with random Delay_Req interval
    ptp_state.timing.next_delay_req_interval_ms =
        DELAY_REQ_MIN_INTERVAL_MS +
        (rand() % (DELAY_REQ_MAX_INTERVAL_MS - DELAY_REQ_MIN_INTERVAL_MS));

    // Create UDP socket for event messages (Sync only)
    ptp_state.event_pcb = udp_new();
    if (ptp_state.event_pcb == NULL) {
        printf("ERROR: Failed to create event UDP socket\n");
        return false;
    }

    // Bind to PTP event port
    if (udp_bind(ptp_state.event_pcb, IP_ADDR_ANY, PTP_EVENT_PORT) != ERR_OK) {
        printf("ERROR: Failed to bind event socket to port %d\n", PTP_EVENT_PORT);
        udp_remove(ptp_state.event_pcb);
        return false;
    }

    // Set receive callback for event messages
    udp_recv(ptp_state.event_pcb, event_recv_callback, NULL);

    // Create UDP socket for general messages (Announce, Follow_Up, Delay_Resp)
    ptp_state.general_pcb = udp_new();
    if (ptp_state.general_pcb == NULL) {
        printf("ERROR: Failed to create general UDP socket\n");
        udp_remove(ptp_state.event_pcb);
        return false;
    }

    // Bind to PTP general port
    if (udp_bind(ptp_state.general_pcb, IP_ADDR_ANY, PTP_GENERAL_PORT) != ERR_OK) {
        printf("ERROR: Failed to bind general socket to port %d\n", PTP_GENERAL_PORT);
        udp_remove(ptp_state.event_pcb);
        udp_remove(ptp_state.general_pcb);
        return false;
    }

    // Set receive callback for general messages
    udp_recv(ptp_state.general_pcb, general_recv_callback, NULL);

    printf("PTP slave ready (listening on ports %d/%d)\n", PTP_EVENT_PORT, PTP_GENERAL_PORT);

    ptp_state.initialized = true;
    return true;
}

void ptp_slave_process(void) {
    if (!ptp_state.initialized) {
        return;
    }

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    // Send Delay_Req periodically with randomized interval (to avoid multi-slave collisions)
    if (now_ms - ptp_state.timing.last_delay_req_ms >= ptp_state.timing.next_delay_req_interval_ms) {
        send_delay_req();
        ptp_state.timing.last_delay_req_ms = now_ms;

        // Randomize next interval (1-3 seconds)
        ptp_state.timing.next_delay_req_interval_ms =
            DELAY_REQ_MIN_INTERVAL_MS +
            (rand() % (DELAY_REQ_MAX_INTERVAL_MS - DELAY_REQ_MIN_INTERVAL_MS));
    }
}

void ptp_slave_get_stats(uint32_t *sync_count, uint32_t *announce_count) {
    if (sync_count) *sync_count = ptp_state.sync_count;
    if (announce_count) *announce_count = ptp_state.announce_count;
}

uint32_t ptp_slave_get_pdv_rejected(void) {
    return ptp_state.pdv_rejected;
}

void ptp_slave_get_timing(int64_t *offset_ns, int64_t *path_delay_ns, int64_t *pdv_ns, bool *offset_valid) {
    if (offset_ns) *offset_ns = ptp_state.timing.offset_ns;
    if (path_delay_ns) *path_delay_ns = ptp_state.timing.path_delay_ns;
    if (pdv_ns) *pdv_ns = ptp_state.timing.pdv_ns;
    if (offset_valid) *offset_valid = ptp_state.timing.offset_valid;
}

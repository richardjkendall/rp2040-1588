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

// PTP state
static struct {
    bool initialized;

    // UDP sockets
    struct udp_pcb *event_pcb;      // For Sync messages (port 319)
    struct udp_pcb *general_pcb;    // For Announce and Follow_Up (port 320)

    // Pending Sync (waiting for Follow_Up)
    bool sync_pending;
    uint16_t sync_sequence;
    uint64_t sync_receive_time_us;

    // Statistics
    uint32_t sync_count;
    uint32_t announce_count;
    uint32_t followup_count;
    uint32_t errors;

} ptp_state = {0};

/**
 * Callback for event messages (Sync) on port 319
 */
static void event_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                                const ip_addr_t *addr, u16_t port) {
    if (p->len < sizeof(ptp_header_t)) {
        pbuf_free(p);
        ptp_state.errors++;
        return;
    }

    ptp_header_t *header = (ptp_header_t *)p->payload;

    // Check if this is a Sync message
    if ((header->message_type & 0x0F) == PTP_MSGTYPE_SYNC) {
        // Record when we received this Sync
        ptp_state.sync_receive_time_us = time_us_64();
        ptp_state.sync_sequence = ntohs(header->sequence_id);
        ptp_state.sync_pending = true;
        ptp_state.sync_count++;
    }

    pbuf_free(p);
}

/**
 * Callback for general messages (Announce, Follow_Up) on port 320
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
            // Extract precise timestamp from Follow_Up
            uint64_t ptp_time_ns = ptp_timestamp_to_ns(&followup->precise_origin_timestamp);

            // DEBUG: Print first few timestamps to verify byte order
            static int debug_count = 0;
            if (debug_count < 3) {
                printf("DEBUG: PTP timestamp: %llu ns (%llu.%09lu sec)\n",
                       ptp_time_ns,
                       ptp_time_ns / 1000000000ULL,
                       (unsigned long)(ptp_time_ns % 1000000000ULL));
                debug_count++;
            }

            // Pass timestamp to Core 1 for discipline
            // Write all data fields first
            ptp_sync_data.ptp_time_ns = ptp_time_ns;
            ptp_sync_data.local_time_us = ptp_state.sync_receive_time_us;
            ptp_sync_data.sequence = followup_sequence;

            // Memory barrier to ensure all data writes complete before setting valid flag
            __compiler_memory_barrier();

            // Set valid flag LAST to signal Core 1 that data is ready
            ptp_sync_data.valid = true;

            ptp_state.sync_pending = false;
            ptp_state.followup_count++;
        }
    }

    pbuf_free(p);
}

bool ptp_slave_init(void) {
    printf("Initializing PTP slave...\n");

    // Create UDP socket for event messages (Sync)
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

    // Create UDP socket for general messages (Announce, Follow_Up)
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
    // Packet reception is handled by lwIP callbacks
    // This function is here for future expansion (e.g., timeout detection)
}

void ptp_slave_get_stats(uint32_t *sync_count, uint32_t *announce_count) {
    if (sync_count) *sync_count = ptp_state.sync_count;
    if (announce_count) *announce_count = ptp_state.announce_count;
}

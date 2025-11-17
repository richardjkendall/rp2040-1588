/**
 * PTP Grandmaster Protocol Implementation
 */

#include "ptp_grandmaster.h"
#include "ptp_protocol.h"
#include "shared_state.h"
#include "wifi_config.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/igmp.h"

// PTP multicast addresses (fallback if not using unicast)
#define PTP_EVENT_MULTICAST_ADDR "224.0.1.129"
#define PTP_EVENT_PORT 319
#define PTP_GENERAL_PORT 320

// PTP domain
#define PTP_DOMAIN 0

// Message transmission interval (1 second)
#define PTP_MESSAGE_INTERVAL_MS 1000

// PTP state
static struct {
    bool initialized;

    // UDP sockets
    struct udp_pcb *event_pcb;      // For Sync messages (port 319)
    struct udp_pcb *general_pcb;    // For Announce and Follow_Up (port 320)

    // Destination address (multicast or unicast)
    ip_addr_t dest_addr;
    bool use_unicast;

    // Clock identity
    ptp_clock_identity_t clock_id;

    // Sequence counters
    uint16_t announce_sequence;
    uint16_t sync_sequence;

    // Statistics
    uint32_t announce_count;
    uint32_t sync_count;
    uint32_t followup_count;

    // Timing
    uint32_t last_message_time_ms;

} ptp_state = {0};

bool ptp_grandmaster_init(void) {
    printf("Initializing PTP...\n");

    // Get MAC address for clock identity
    uint8_t mac[6];
    cyw43_hal_get_mac(CYW43_ITF_STA, mac);

    // Initialize clock identity from MAC address
    ptp_init_clock_identity(&ptp_state.clock_id, mac);

    // Determine destination: unicast or multicast
    const char *dest_ip_str = PTP_DEST_IP;

    // Check if configured for unicast (not "0.0.0.0")
    if (strcmp(dest_ip_str, "0.0.0.0") == 0) {
        // Use multicast
        ptp_state.use_unicast = false;
        if (!ip4addr_aton(PTP_EVENT_MULTICAST_ADDR, &ptp_state.dest_addr)) {
            printf("ERROR: Invalid PTP multicast addr\n");
            return false;
        }
    } else {
        // Use unicast to specific IP
        ptp_state.use_unicast = true;
        if (!ip4addr_aton(dest_ip_str, &ptp_state.dest_addr)) {
            printf("ERROR: Invalid PTP dest IP\n");
            return false;
        }
    }
    printf("PTP dest: %s (%s)\n",
           ip4addr_ntoa(&ptp_state.dest_addr),
           ptp_state.use_unicast ? "unicast" : "multicast");

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

    // Join IGMP multicast group (only if using multicast)
    if (!ptp_state.use_unicast) {
        ip4_addr_t multicast_ip4;
        ip4_addr_copy(multicast_ip4, ptp_state.dest_addr);

        if (igmp_joingroup_netif(netif_default, &multicast_ip4) != ERR_OK) {
            printf("WARNING: IGMP join failed\n");
        }
    }

    printf("PTP ready (ports %d/%d, domain %d)\n", PTP_EVENT_PORT, PTP_GENERAL_PORT, PTP_DOMAIN);

    ptp_state.initialized = true;
    ptp_state.last_message_time_ms = to_ms_since_boot(get_absolute_time());

    return true;
}

static void send_announce_message(void) {
    ptp_announce_msg_t msg;

    // Build Announce message
    ptp_build_announce(&msg, &ptp_state.clock_id, PTP_DOMAIN, ptp_state.announce_sequence);

    // Allocate pbuf for message
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(msg), PBUF_RAM);
    if (p == NULL) {
        printf("ERROR: Failed to allocate pbuf for Announce\n");
        return;
    }

    // Copy message to pbuf
    memcpy(p->payload, &msg, sizeof(msg));

    // Send via general messages socket
    err_t err = udp_sendto(ptp_state.general_pcb, p, &ptp_state.dest_addr, PTP_GENERAL_PORT);

    // Free pbuf
    pbuf_free(p);

    if (err != ERR_OK) {
        printf("ERROR: Failed to send Announce message (err=%d)\n", err);
        return;
    }

    ptp_state.announce_sequence++;
    ptp_state.announce_count++;
}

static void send_sync_and_followup(void) {
    // Read disciplined clock time from Core 1
    uint64_t timestamp_ns = core1_stats.disciplined_time_ns;

    // Build Sync message with approximate timestamp
    ptp_sync_msg_t sync_msg;
    ptp_build_sync(&sync_msg, &ptp_state.clock_id, PTP_DOMAIN, ptp_state.sync_sequence, timestamp_ns);

    // Allocate pbuf for Sync
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(sync_msg), PBUF_RAM);
    if (p == NULL) {
        printf("ERROR: Failed to allocate pbuf for Sync\n");
        return;
    }

    // Copy Sync message to pbuf
    memcpy(p->payload, &sync_msg, sizeof(sync_msg));

    // Send Sync message
    err_t err = udp_sendto(ptp_state.event_pcb, p, &ptp_state.dest_addr, PTP_EVENT_PORT);
    pbuf_free(p);

    if (err != ERR_OK) {
        printf("ERROR: Failed to send Sync message (err=%d)\n", err);
        return;
    }

    ptp_state.sync_count++;

    // Immediately read precise timestamp for Follow_Up
    uint64_t precise_timestamp_ns = core1_stats.disciplined_time_ns;

    // Build Follow_Up message with precise timestamp
    ptp_follow_up_msg_t followup_msg;
    ptp_build_follow_up(&followup_msg, &ptp_state.clock_id, PTP_DOMAIN, ptp_state.sync_sequence, precise_timestamp_ns);

    // Allocate pbuf for Follow_Up
    p = pbuf_alloc(PBUF_TRANSPORT, sizeof(followup_msg), PBUF_RAM);
    if (p == NULL) {
        printf("ERROR: Failed to allocate pbuf for Follow_Up\n");
        return;
    }

    // Copy Follow_Up message to pbuf
    memcpy(p->payload, &followup_msg, sizeof(followup_msg));

    // Send Follow_Up message
    err = udp_sendto(ptp_state.general_pcb, p, &ptp_state.dest_addr, PTP_GENERAL_PORT);
    pbuf_free(p);

    if (err != ERR_OK) {
        printf("ERROR: Failed to send Follow_Up message (err=%d)\n", err);
        return;
    }

    ptp_state.followup_count++;
    ptp_state.sync_sequence++;
}

void ptp_grandmaster_process(void) {
    if (!ptp_state.initialized) {
        return;
    }

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    // Send messages every 1 second
    if (now_ms - ptp_state.last_message_time_ms >= PTP_MESSAGE_INTERVAL_MS) {
        ptp_state.last_message_time_ms = now_ms;

        // Send Announce message
        send_announce_message();

        // Send Sync + Follow_Up messages
        send_sync_and_followup();
    }
}

void ptp_grandmaster_get_stats(uint32_t *announce_count, uint32_t *sync_count, uint32_t *followup_count) {
    if (announce_count) *announce_count = ptp_state.announce_count;
    if (sync_count) *sync_count = ptp_state.sync_count;
    if (followup_count) *followup_count = ptp_state.followup_count;
}

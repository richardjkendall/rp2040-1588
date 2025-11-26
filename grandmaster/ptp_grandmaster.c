/**
 * PTP Grandmaster Protocol Implementation
 */

#include "ptp_grandmaster.h"
#include "ptp_protocol.h"
#include "shared_state.h"
#include "network_config.h"
#include "network_interface.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
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

// Announce message control (set to 0 to disable for fixed unicast topology)
// Announce is only needed for BMCA (Best Master Clock Algorithm) and domain discovery
// For fixed IP unicast setups, it provides no value and wastes bandwidth
#define ENABLE_ANNOUNCE_MESSAGES 0

// Slave session management
#define MAX_SLAVES 8
#define SLAVE_TIMEOUT_MS 30000  // 30 seconds

/**
 * Slave session tracking
 * Each slave that sends Delay_Req gets a session
 */
typedef struct {
    bool active;                            // Is this session valid?
    ptp_port_identity_t port_identity;      // Slave's unique port identity
    ip_addr_t ip_address;                   // Slave's IP for unicast Delay_Resp
    uint16_t port;                          // Source port
    uint32_t last_contact_ms;               // Last Delay_Req received timestamp
    uint16_t last_delay_req_sequence;       // Last Delay_Req sequence number
} slave_session_t;

// PTP state
static struct {
    bool initialized;

    // UDP sockets
    struct udp_pcb *event_pcb;      // For Sync and Delay_Req (port 319)
    struct udp_pcb *general_pcb;    // For Announce, Follow_Up, and Delay_Resp (port 320)

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
    uint32_t delay_resp_count;

    // Timing
    uint32_t last_message_time_ms;
    uint32_t last_cleanup_ms;

    // Slave sessions
    slave_session_t slaves[MAX_SLAVES];

} ptp_state = {0};

/**
 * Find or create a slave session by port identity
 */
static slave_session_t* find_or_create_slave(const ptp_port_identity_t *port_id,
                                             const ip_addr_t *ip_addr,
                                             uint16_t port) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    slave_session_t *free_slot = NULL;

    // First, try to find existing session
    for (int i = 0; i < MAX_SLAVES; i++) {
        if (ptp_state.slaves[i].active) {
            if (memcmp(&ptp_state.slaves[i].port_identity, port_id, sizeof(ptp_port_identity_t)) == 0) {
                // Found existing session
                return &ptp_state.slaves[i];
            }
        } else if (free_slot == NULL) {
            // Remember first free slot
            free_slot = &ptp_state.slaves[i];
        }
    }

    // Not found - create new session if we have space
    if (free_slot != NULL) {
        free_slot->active = true;
        memcpy(&free_slot->port_identity, port_id, sizeof(ptp_port_identity_t));
        free_slot->ip_address = *ip_addr;
        free_slot->port = port;
        free_slot->last_contact_ms = now_ms;
        free_slot->last_delay_req_sequence = 0;
        return free_slot;
    }

    // No space available
    return NULL;
}

/**
 * Clean up stale slave sessions (haven't heard from in >30 seconds)
 */
static void cleanup_stale_slaves(void) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    for (int i = 0; i < MAX_SLAVES; i++) {
        if (ptp_state.slaves[i].active) {
            if (now_ms - ptp_state.slaves[i].last_contact_ms > SLAVE_TIMEOUT_MS) {
                ptp_state.slaves[i].active = false;
                // Optional: Could log slave disconnect here
            }
        }
    }
}

/**
 * Get number of active slaves
 */
static uint32_t get_active_slave_count(void) {
    uint32_t count = 0;
    for (int i = 0; i < MAX_SLAVES; i++) {
        if (ptp_state.slaves[i].active) {
            count++;
        }
    }
    return count;
}

/**
 * Send Delay_Resp message to a specific slave
 */
static void send_delay_resp(slave_session_t *slave, uint64_t t4_timestamp_ns) {
    ptp_delay_resp_msg_t msg;

    // Build Delay_Resp message
    ptp_build_delay_resp(&msg, &ptp_state.clock_id, PTP_DOMAIN,
                         slave->last_delay_req_sequence, t4_timestamp_ns,
                         &slave->port_identity);

    // Allocate pbuf for Delay_Resp
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(msg), PBUF_RAM);
    if (p == NULL) {
        printf("ERROR: Failed to allocate pbuf for Delay_Resp\n");
        return;
    }

    // Copy message to pbuf
    memcpy(p->payload, &msg, sizeof(msg));

    // Send Delay_Resp via general messages socket (unicast to this slave's IP, port 320)
    // Note: Must use PTP_GENERAL_PORT (320), not the source port from Delay_Req
    err_t err = udp_sendto(ptp_state.general_pcb, p, &slave->ip_address, PTP_GENERAL_PORT);
    pbuf_free(p);

    if (err != ERR_OK) {
        printf("ERROR: Failed to send Delay_Resp (err=%d)\n", err);
        return;
    }

    ptp_state.delay_resp_count++;
}

// Forward declaration for GPS time function
extern uint64_t get_gps_time_ns(void);

/**
 * Callback for event messages (Sync, Delay_Req) on port 319
 */
static void event_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                                const ip_addr_t *addr, u16_t port) {
    if (p->len < sizeof(ptp_header_t)) {
        pbuf_free(p);
        return;
    }

    ptp_header_t *header = (ptp_header_t *)p->payload;
    uint8_t msg_type = header->message_type & 0x0F;

    // Check if this is a Delay_Req message
    if (msg_type == PTP_MSGTYPE_DELAY_REQ && p->len >= sizeof(ptp_delay_req_msg_t)) {
        ptp_delay_req_msg_t *delay_req = (ptp_delay_req_msg_t *)p->payload;

        // Record reception timestamp (t4) - call GPS time directly
        uint64_t t4_ns = get_gps_time_ns();

        // Find or create slave session
        slave_session_t *slave = find_or_create_slave(&delay_req->header.source_port_identity, addr, port);

        if (slave != NULL) {
            // Update session
            slave->last_contact_ms = to_ms_since_boot(get_absolute_time());
            slave->last_delay_req_sequence = ntohs(delay_req->header.sequence_id);

            // Send Delay_Resp with t4 timestamp
            send_delay_resp(slave, t4_ns);
        }
        // else: no space for new slave (silently drop)
    }

    pbuf_free(p);
}

bool ptp_grandmaster_init(void) {
    printf("Initializing PTP...\n");

    // Get MAC address for clock identity
    uint8_t mac[6];
    network_get_mac(mac);

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

    // Set receive callback for event messages (Delay_Req)
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

#if ENABLE_ANNOUNCE_MESSAGES
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
#endif // ENABLE_ANNOUNCE_MESSAGES

static void send_sync_and_followup(void) {
    // Read GPS timestamp for Sync message
    uint64_t timestamp_ns = get_gps_time_ns();

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

    // Immediately read precise GPS timestamp for Follow_Up
    uint64_t precise_timestamp_ns = get_gps_time_ns();

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

#if ENABLE_ANNOUNCE_MESSAGES
        // Send Announce message (disabled for fixed unicast topology)
        send_announce_message();
#endif

        // Send Sync + Follow_Up messages
        send_sync_and_followup();
    }

    // Clean up stale slave sessions every 10 seconds
    if (now_ms - ptp_state.last_cleanup_ms >= 10000) {
        ptp_state.last_cleanup_ms = now_ms;
        cleanup_stale_slaves();
    }
}

void ptp_grandmaster_get_stats(uint32_t *announce_count, uint32_t *sync_count, uint32_t *followup_count) {
    if (announce_count) *announce_count = ptp_state.announce_count;
    if (sync_count) *sync_count = ptp_state.sync_count;
    if (followup_count) *followup_count = ptp_state.followup_count;
}

void ptp_grandmaster_get_stats_extended(uint32_t *announce_count, uint32_t *sync_count,
                                        uint32_t *followup_count, uint32_t *delay_resp_count,
                                        uint32_t *active_slaves) {
    if (announce_count) *announce_count = ptp_state.announce_count;
    if (sync_count) *sync_count = ptp_state.sync_count;
    if (followup_count) *followup_count = ptp_state.followup_count;
    if (delay_resp_count) *delay_resp_count = ptp_state.delay_resp_count;
    if (active_slaves) *active_slaves = get_active_slave_count();
}

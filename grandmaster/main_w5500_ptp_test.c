/**
 * W5500 + PTP Test - GPS-Disciplined Grandmaster
 *
 * Simple W5500 driver with GPS-disciplined PTP grandmaster.
 * Uses GPS PPS for sub-microsecond accuracy.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "w5500_simple.h"
#include "eth_simple.h"
#include "ptp_protocol.h"
#include "gps.h"
#include "discipline_v3.h"
#include "shared_state.h"

// Network configuration (MUST match network_config.h values)
#define MY_MAC          {0x00, 0x08, 0xDC, 0x12, 0x34, 0x01}
#define MY_IP_ADDR      "192.168.1.100"
#define MY_NETMASK      "255.255.255.0"
#define MY_GATEWAY      "192.168.1.1"
#define PTP_DEST_IP     "192.168.1.10"  // Slave IP (unicast)

// PTP configuration
#define PTP_EVENT_PORT  319
#define PTP_GENERAL_PORT 320
#define PTP_DOMAIN      0

// GPS configuration
#define GPS_UART_ID     uart0
#define GPS_TX_PIN      0
#define GPS_RX_PIN      1
#define GPS_PPS_PIN     2
#define GPS_PIO         pio1
#define GPS_SM          0
#define DEBUG_PPS_PIN   4   // Toggle on GPS PPS IRQ
#define DEBUG_LOCK_PIN  5   // High when GPS locked

// Export debug pins for discipline module
uint debug_lock_pin = DEBUG_LOCK_PIN;
uint debug_pps_pin = DEBUG_PPS_PIN;

// Convert IP string to uint32 (host byte order)
static uint32_t ip_str_to_u32(const char *ip_str) {
    uint32_t a, b, c, d;
    sscanf(ip_str, "%lu.%lu.%lu.%lu", &a, &b, &c, &d);
    return (a << 24) | (b << 16) | (c << 8) | d;
}

// PTP state
static struct {
    ptp_clock_identity_t clock_id;
    uint16_t sync_sequence;
    uint32_t sync_count;
    uint32_t followup_count;
    uint32_t delay_resp_count;
    uint32_t delay_req_count;
    uint8_t slave_mac[6];      // Learned from first Delay_Req
    uint32_t slave_ip;         // Learned from first Delay_Req
    bool slave_known;
} ptp_state = {0};

/**
 * Send PTP Sync + Follow_Up messages
 */
static void send_sync_and_followup(void) {
    if (!ptp_state.slave_known) {
        return;  // Don't send until we know slave address
    }

    uint8_t buffer[256];
    uint16_t frame_len;

    // Capture timestamp for Sync (GPS-disciplined)
    uint64_t sync_timestamp_ns = get_gps_time_ns();

    // Build Sync message
    ptp_sync_msg_t sync_msg;
    ptp_build_sync(&sync_msg, &ptp_state.clock_id, PTP_DOMAIN,
                   ptp_state.sync_sequence, sync_timestamp_ns);

    // Wrap in UDP/IP/Ethernet and send
    frame_len = eth_build_udp(buffer, ptp_state.slave_mac, ptp_state.slave_ip,
                              PTP_EVENT_PORT, PTP_EVENT_PORT,
                              (uint8_t*)&sync_msg, sizeof(sync_msg));
    w5500_send_frame(buffer, frame_len);
    ptp_state.sync_count++;

    // Capture precise timestamp for Follow_Up (GPS-disciplined)
    uint64_t followup_timestamp_ns = get_gps_time_ns();

    // Build Follow_Up message
    ptp_follow_up_msg_t followup_msg;
    ptp_build_follow_up(&followup_msg, &ptp_state.clock_id, PTP_DOMAIN,
                        ptp_state.sync_sequence, followup_timestamp_ns);

    // Wrap in UDP/IP/Ethernet and send
    frame_len = eth_build_udp(buffer, ptp_state.slave_mac, ptp_state.slave_ip,
                              PTP_GENERAL_PORT, PTP_GENERAL_PORT,
                              (uint8_t*)&followup_msg, sizeof(followup_msg));
    w5500_send_frame(buffer, frame_len);
    ptp_state.followup_count++;

    ptp_state.sync_sequence++;
}

/**
 * Handle received PTP Delay_Req message
 */
static void handle_delay_req(const udp_packet_t *udp,
                             const uint8_t *src_mac, uint32_t src_ip) {
    if (udp->payload_len < sizeof(ptp_delay_req_msg_t)) {
        return;
    }

    ptp_delay_req_msg_t *delay_req = (ptp_delay_req_msg_t*)udp->payload;

    // Capture RX timestamp (t4) - GPS-disciplined
    uint64_t rx_timestamp_ns = get_gps_time_ns();

    // sequence_id is uint16_t in network byte order, convert to host byte order
    uint16_t req_seq = (delay_req->header.sequence_id >> 8) |
                       ((delay_req->header.sequence_id & 0xFF) << 8);

    // Learn slave address from first Delay_Req
    if (!ptp_state.slave_known) {
        memcpy(ptp_state.slave_mac, src_mac, 6);
        ptp_state.slave_ip = src_ip;
        ptp_state.slave_known = true;
        printf("PTP slave discovered: %u.%u.%u.%u\n",
               (src_ip >> 24) & 0xFF, (src_ip >> 16) & 0xFF,
               (src_ip >> 8) & 0xFF, src_ip & 0xFF);
    }

    ptp_state.delay_req_count++;

    // Build Delay_Resp message
    ptp_delay_resp_msg_t delay_resp;
    ptp_build_delay_resp(&delay_resp, &ptp_state.clock_id, PTP_DOMAIN,
                         req_seq, rx_timestamp_ns,
                         &delay_req->header.source_port_identity);

    // Wrap in UDP/IP/Ethernet and send
    uint8_t buffer[256];
    uint16_t frame_len = eth_build_udp(buffer, src_mac, src_ip,
                                       PTP_GENERAL_PORT, PTP_GENERAL_PORT,
                                       (uint8_t*)&delay_resp, sizeof(delay_resp));
    w5500_send_frame(buffer, frame_len);
    ptp_state.delay_resp_count++;
}

int main() {
    // Initialize system
    set_sys_clock_khz(250000, true);
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== W5500 + GPS-Disciplined PTP Grandmaster ===\n");
    printf("System clock: %lu Hz (%lu MHz)\n\n", clock_get_hz(clk_sys), clock_get_hz(clk_sys) / 1000000);

    // Initialize debug GPIOs
    gpio_init(DEBUG_PPS_PIN);
    gpio_set_dir(DEBUG_PPS_PIN, GPIO_OUT);
    gpio_put(DEBUG_PPS_PIN, 0);
    gpio_init(DEBUG_LOCK_PIN);
    gpio_set_dir(DEBUG_LOCK_PIN, GPIO_OUT);
    gpio_put(DEBUG_LOCK_PIN, 0);

    printf("Debug GPIOs:\n");
    printf("  GPIO %d: GPS PPS toggle\n", DEBUG_PPS_PIN);
    printf("  GPIO %d: GPS lock status\n\n", DEBUG_LOCK_PIN);

    // Initialize GPS module (NMEA parsing + PPS on PIO1)
    printf("Initializing GPS...\n");
    gps_init(GPS_UART_ID, GPS_TX_PIN, GPS_RX_PIN, GPS_PPS_PIN, GPS_PIO, GPS_SM);

    // Initialize GPS discipline system
    printf("Initializing GPS discipline...\n");
    if (!discipline_init_v3()) {
        printf("FATAL: GPS discipline init failed\n");
        while (1) { sleep_ms(1000); }
    }
    printf("GPS discipline initialized\n\n");

    // Network configuration
    uint8_t my_mac[] = MY_MAC;
    net_config_t net_cfg = {
        .ip = ip_str_to_u32(MY_IP_ADDR),
        .netmask = ip_str_to_u32(MY_NETMASK),
        .gateway = ip_str_to_u32(MY_GATEWAY)
    };
    memcpy(net_cfg.mac, my_mac, 6);

    // Initialize W5500
    printf("Initializing W5500 Ethernet...\n");
    if (!w5500_simple_init(my_mac)) {
        printf("FATAL: W5500 init failed\n");
        while (1) { sleep_ms(1000); }
    }

    // Initialize Ethernet handler
    eth_init(&net_cfg);

    // Initialize PTP clock identity from MAC
    ptp_init_clock_identity(&ptp_state.clock_id, my_mac);

    printf("\nSystem ready:\n");
    printf("  IP: %s\n", MY_IP_ADDR);
    printf("  Waiting for ARP/PTP packets...\n");
    printf("  Test ping: ping %s\n", MY_IP_ADDR);
    printf("  Test PTP: Start slave pointing to %s\n\n", MY_IP_ADDR);

    uint64_t last_sync_time_us = 0;
    uint64_t last_stats_time_us = 0;
    uint64_t last_garp_time_us = 0;
    uint64_t last_w5500_dump_us = 0;
    uint8_t rx_buffer[W5500_MAX_FRAME_SIZE];
    bool link_reported = false;
    bool garp_sent = false;

    // Main loop
    while (true) {
        uint64_t now_us = time_us_64();

        // Check link status
        bool link_up = w5500_link_up();
        if (link_up && !link_reported) {
            printf("Ethernet link UP\n");
            w5500_dump_status();  // Show W5500 state when link comes up
            link_reported = true;
            garp_sent = false;  // Send GARP after link comes up
        } else if (!link_up && link_reported) {
            printf("Ethernet link DOWN\n");
            link_reported = false;
        }

        // Only process frames if link is up
        if (!link_up) {
            sleep_ms(100);
            continue;
        }

        // Send Gratuitous ARP when link comes up to announce our presence
        // TEMPORARILY DISABLED to force remote to send ARP requests
        // Using ARP Request format (opcode 1) which is more standard and better tolerated by switches
        if (false && (!garp_sent || (now_us - last_garp_time_us >= 5000000))) {
            uint8_t garp_buffer[60];
            uint32_t my_ip = ip_str_to_u32(MY_IP_ADDR);

            // Build Gratuitous ARP Request (asking "who has my IP?" - announces we have it)
            uint16_t garp_len = eth_build_arp_request(garp_buffer, my_ip);

            if (w5500_send_frame(garp_buffer, garp_len)) {
                printf("GARP sent (announcing %s)\n", MY_IP_ADDR);
            } else {
                printf("ERROR: Failed to send GARP\n");
            }
            garp_sent = true;
            last_garp_time_us = now_us;
        }

        // Process GPS NMEA sentences (needed for time sync)
        gps_process();

        // Check for received frames
        uint16_t rx_len;
        if (w5500_recv_frame(rx_buffer, sizeof(rx_buffer), &rx_len)) {
            // Handle Ethernet frame (ARP, ICMP, and UDP)
            udp_packet_t udp;
            uint8_t src_mac[6];
            uint32_t src_ip;

            if (eth_handle_frame(rx_buffer, rx_len, &udp, src_mac, &src_ip)) {
                // Check if PTP event port (Delay_Req on port 319)
                if (udp.dest_port == PTP_EVENT_PORT) {
                    if (udp.payload_len >= 1) {
                        uint8_t msg_type = udp.payload[0] & 0x0F;
                        if (msg_type == PTP_MSGTYPE_DELAY_REQ) {
                            handle_delay_req(&udp, src_mac, src_ip);
                        }
                    }
                }
                // Silently ignore other UDP traffic
            }
        }

        // Send Sync + Follow_Up every 1 second
        if (now_us - last_sync_time_us >= 1000000) {
            send_sync_and_followup();
            last_sync_time_us = now_us;
        }

        // Print stats every 30 seconds (reduce printf overhead)
        if (now_us - last_stats_time_us >= 30000000) {
            // Get GPS discipline stats
            extern volatile uint64_t gps_ns_counter;
            extern volatile int32_t crystal_error_ns;
            extern volatile int64_t interpolation_error_ns;
            uint64_t gps_seconds = gps_ns_counter / 1000000000ULL;
            double crystal_ppb = (double)crystal_error_ns / 1000.0;
            double crystal_ppm = crystal_ppb / 1000.0;

            // Get GPS data
            gps_data_t gps_data;
            gps_get_data(&gps_data);

            printf("Stats: Sync=%lu FollowUp=%lu DelayReq=%lu DelayResp=%lu Slave=%s GPS_Lock=%s PPS=%lu GPS_fix=%s GPS_ns=%llus phase_err=%+ldns (%+.3fppm)\n",
                   ptp_state.sync_count,
                   ptp_state.followup_count,
                   ptp_state.delay_req_count,
                   ptp_state.delay_resp_count,
                   ptp_state.slave_known ? "YES" : "NO",
                   core1_stats.locked ? "YES" : "NO",
                   core1_stats.pps_count,
                   gps_data.valid ? "YES" : "NO",
                   (unsigned long long)gps_seconds,
                   (long)crystal_error_ns,
                   crystal_ppm);
            last_stats_time_us = now_us;
        }

        // Dump W5500 status every 60 seconds for diagnostics
        if (now_us - last_w5500_dump_us >= 60000000) {
            w5500_dump_status();
            last_w5500_dump_us = now_us;
        }

        // Small delay - use tight loop to reduce latency
        sleep_us(100);  // 100us instead of 1ms for faster response
    }

    return 0;
}

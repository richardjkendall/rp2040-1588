/**
 * W5500 Ethernet PTP Slave - DUAL CORE
 *
 * Core 0: PTP discipline ONLY (minimal interference, sub-microsecond accuracy)
 * Core 1: W5500 Ethernet + PTP slave protocol (heavy workload isolated)
 *
 * Synchronizes to PTP grandmaster using IEEE 1588-2008 delay request-response mechanism.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "../grandmaster/w5500_simple.h"
#include "../grandmaster/eth_simple.h"
#include "ptp_protocol.h"
#include "ptp_slave_w5500.h"
#include "ptp_discipline.h"
#include "shared_state_slave.h"

// External shared state for timestamp capture
extern ptp_sync_data_t ptp_sync_data;

// Network configuration
#define MY_MAC          {0x00, 0x08, 0xDC, 0x12, 0x34, 0x02}  // Different from GM
#define MY_IP_ADDR      "192.168.1.102"
#define MY_NETMASK      "255.255.255.0"
#define MY_GATEWAY      "192.168.1.1"

// PTP configuration
#define PTP_EVENT_PORT  319
#define PTP_GENERAL_PORT 320
#define PTP_DOMAIN      0

// Grandmaster configuration (for initial discovery)
#define GM_IP_ADDR      "192.168.1.100"  // Expected GM IP
#define GM_MAC          {0x00, 0x08, 0xDC, 0x12, 0x34, 0x01}  // GM MAC address

// Core synchronization
volatile bool core0_ready = false;

// Convert IP string to uint32 (host byte order)
static uint32_t ip_str_to_u32(const char *ip_str) {
    uint32_t a, b, c, d;
    sscanf(ip_str, "%lu.%lu.%lu.%lu", &a, &b, &c, &d);
    return (a << 24) | (b << 16) | (c << 8) | d;
}

/**
 * Core 1 Entry Point: Network and PTP Slave Protocol
 */
void core1_network_entry(void) {
    // Wait for Core 0 PTP discipline to be ready
    while (!core0_ready) {
        tight_loop_contents();
    }

    printf("\n[Core 1] Network core starting...\n");

    // Network configuration
    uint8_t my_mac[] = MY_MAC;
    net_config_t net_cfg = {
        .ip = ip_str_to_u32(MY_IP_ADDR),
        .netmask = ip_str_to_u32(MY_NETMASK),
        .gateway = ip_str_to_u32(MY_GATEWAY)
    };
    memcpy(net_cfg.mac, my_mac, 6);

    // Initialize W5500
    printf("[Core 1] Initializing W5500 Ethernet...\n");
    if (!w5500_simple_init(my_mac)) {
        printf("[Core 1] FATAL: W5500 init failed\n");
        while (1) { sleep_ms(1000); }
    }

    // Initialize Ethernet handler
    eth_init(&net_cfg);

    // Initialize PTP clock identity from MAC
    ptp_clock_identity_t clock_id;
    ptp_init_clock_identity(&clock_id, my_mac);

    // Initialize PTP slave protocol
    ptp_slave_w5500_init(&clock_id, PTP_DOMAIN);

    printf("[Core 1] Network ready: IP=%s\n", MY_IP_ADDR);
    printf("[Core 1] Waiting for PTP grandmaster...\n\n");

    uint64_t last_delay_req_time_us = 0;
    uint64_t last_stats_time_us = 0;
    uint8_t rx_buffer[W5500_MAX_FRAME_SIZE];
    bool link_reported = false;

    // Main network loop (Core 1)
    while (true) {
        uint64_t now_us = time_us_64();

        // Check link status
        bool link_up = w5500_link_up();
        if (link_up && !link_reported) {
            printf("[Core 1] Ethernet link UP\n");
            link_reported = true;
        } else if (!link_up && link_reported) {
            printf("[Core 1] Ethernet link DOWN\n");
            link_reported = false;
        }

        if (!link_up) {
            sleep_ms(100);
            continue;
        }

        // Check for received frames
        uint16_t rx_len;
        if (w5500_recv_frame(rx_buffer, sizeof(rx_buffer), &rx_len)) {
            udp_packet_t udp;
            uint8_t src_mac[6];
            uint32_t src_ip;

            if (eth_handle_frame(rx_buffer, rx_len, &udp, src_mac, &src_ip)) {
                // Check if PTP port
                if (udp.dest_port == PTP_EVENT_PORT || udp.dest_port == PTP_GENERAL_PORT) {
                    if (udp.payload_len >= sizeof(ptp_header_t)) {
                        uint8_t msg_type = udp.payload[0] & 0x0F;

                        if (msg_type == PTP_MSGTYPE_SYNC) {
                            ptp_slave_w5500_handle_sync((ptp_sync_msg_t*)udp.payload,
                                                        &udp, src_mac, src_ip);
                        } else if (msg_type == PTP_MSGTYPE_FOLLOW_UP) {
                            ptp_slave_w5500_handle_followup((ptp_follow_up_msg_t*)udp.payload);
                        } else if (msg_type == PTP_MSGTYPE_DELAY_RESP) {
                            ptp_slave_w5500_handle_delay_resp((ptp_delay_resp_msg_t*)udp.payload);
                        }
                    }
                }
            }
        }

        // Send Delay_Req every 1 second
        if (now_us - last_delay_req_time_us >= 1000000) {
            uint32_t sync_count, followup_count, delay_req_count, delay_resp_count;
            bool gm_known;
            ptp_slave_w5500_get_stats(&sync_count, &followup_count,
                                     &delay_req_count, &delay_resp_count, &gm_known);

            uint8_t tx_buffer[256];
            const uint8_t *gm_mac;
            uint32_t gm_ip;

            if (gm_known) {
                // Use learned GM address
                gm_mac = ptp_slave_w5500_get_gm_mac();
                gm_ip = ptp_slave_w5500_get_gm_ip();
            } else {
                // Use configured GM MAC/IP for discovery
                static const uint8_t configured_gm_mac[6] = GM_MAC;
                gm_mac = configured_gm_mac;
                gm_ip = ip_str_to_u32(GM_IP_ADDR);
            }

            uint16_t frame_len = ptp_slave_w5500_send_delay_req(
                gm_mac, gm_ip, tx_buffer);

            if (frame_len > 0) {
                w5500_send_frame(tx_buffer, frame_len);

                // CRITICAL: Capture t3 AFTER sending frame
                // This accounts for W5500 SPI transfer + processing latency (~900µs)
                extern uint64_t get_ptp_time_ns(void);
                ptp_sync_data.t3_ptp_ns = get_ptp_time_ns();
                ptp_sync_data.t3_slave_us = time_us_64();
            }

            last_delay_req_time_us = now_us;
        }

        // Print stats every 10 seconds
        if (now_us - last_stats_time_us >= 10000000) {
            uint32_t sync_count, followup_count, delay_req_count, delay_resp_count;
            bool gm_known;
            ptp_slave_w5500_get_stats(&sync_count, &followup_count,
                                     &delay_req_count, &delay_resp_count, &gm_known);

            printf("[Core 1] PTP: Sync=%lu FollowUp=%lu DelayReq=%lu DelayResp=%lu GM=%s\n",
                   sync_count, followup_count, delay_req_count, delay_resp_count,
                   gm_known ? "YES" : "NO");
            printf("[Core 0] Discipline: Lock=%s Offset=%+lldns PathDelay=%+lldns FreqOff=%+.3fppb\n",
                   ptp_stats.locked ? "YES" : "NO",
                   (long long)ptp_stats.offset_from_master_ns,
                   (long long)ptp_stats.mean_path_delay_ns,
                   ptp_stats.freq_offset_ppb);
            printf("         Crystal: err=%+lldns (%+.3fppm) scale=%.9f\n\n",
                   (long long)ptp_stats.crystal_error_ns,
                   ptp_stats.crystal_ppm,
                   ptp_stats.scale_factor);

            last_stats_time_us = now_us;
        }

        // Minimal delay
        sleep_us(100);
    }
}

/**
 * Core 0 Main: PTP Discipline ONLY
 */
int main() {
    // Overclock to 250 MHz
    set_sys_clock_khz(250000, true);

    // Initialize stdio
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== W5500 PTP Slave - DUAL CORE ===\n");
    printf("System clock: %lu MHz\n\n", clock_get_hz(clk_sys) / 1000000);

    printf("[Core 0] PTP discipline core starting...\n");

    // Initialize PTP discipline system
    printf("[Core 0] Initializing PTP discipline...\n");
    if (!ptp_discipline_init()) {
        printf("[Core 0] FATAL: PTP discipline init failed\n");
        while (1) { sleep_ms(1000); }
    }
    printf("[Core 0] PTP discipline ready\n\n");

    // Signal Core 1 that timing is ready
    core0_ready = true;

    // Launch Core 1 for network processing
    printf("[Core 0] Launching Core 1 for network/PTP...\n");
    multicore_launch_core1(core1_network_entry);

    printf("[Core 0] Entering PTP discipline loop...\n\n");

    // MINIMAL LOOP - Only PTP discipline updates
    while (true) {
        // Update discipline when new timestamp set available
        ptp_discipline_update();

        // Sleep to minimize CPU interference
        sleep_ms(10);
    }

    return 0;
}

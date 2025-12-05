/**
 * W5500 + GPS-Disciplined PTP Grandmaster - DUAL CORE
 *
 * Core 0: GPS discipline ONLY (minimal interference, ±200-400ns accuracy)
 * Core 1: W5500 Ethernet + PTP protocol (heavy workload isolated)
 *
 * Combines proven components:
 * - GPS discipline from grandmaster_minimal_test (sub-microsecond accuracy)
 * - W5500 + PTP from w5500_ptp_test (working network stack)
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "w5500_simple.h"
#include "eth_simple.h"
#include "ptp_protocol.h"
#include "gps.h"
#include "discipline_v3.h"
#include "shared_state.h"
#include "pps_scheduler.h"

// Network configuration
#define MY_MAC          {0x00, 0x08, 0xDC, 0x12, 0x34, 0x01}
#define MY_IP_ADDR      "192.168.1.100"
#define MY_NETMASK      "255.255.255.0"
#define MY_GATEWAY      "192.168.1.1"

// PTP configuration
#define PTP_EVENT_PORT  319
#define PTP_GENERAL_PORT 320
#define PTP_DOMAIN      0

// GPS configuration (Core 0)
#define GPS_UART_ID     uart0
#define GPS_TX_PIN      0
#define GPS_RX_PIN      1
#define GPS_PPS_PIN     2
#define GPS_PIO         pio0  // Use PIO0 for GPS discipline
#define GPS_SM          0
#define GPS_PPS_CAPTURE_SM 1  // SM1 for PPS edge capture
#define DEBUG_PPS_PIN   4
#define DEBUG_LOCK_PIN  5

// 1PPS output pin (for external measurement / phase comparison with slave)
// Moved to GPIO 15 to avoid crosstalk from W5500 SPI signals (GPIO 16-22)
#define PPS_OUTPUT_PIN  15
#define PPS_SCHEDULER_SM 2  // SM2 for scheduled 1PPS generation

// Export debug pins for discipline module
uint debug_lock_pin = DEBUG_LOCK_PIN;
uint debug_pps_pin = DEBUG_PPS_PIN;

// Core synchronization
volatile bool core0_ready = false;

// Scale factor for 1PPS scheduler (GPS is always 1.0 - no crystal correction needed)
static double gps_get_scale_factor(void) {
    return 1.0;
}

// Convert IP string to uint32 (host byte order)
static uint32_t ip_str_to_u32(const char *ip_str) {
    uint32_t a, b, c, d;
    sscanf(ip_str, "%lu.%lu.%lu.%lu", &a, &b, &c, &d);
    return (a << 24) | (b << 16) | (c << 8) | d;
}

// PTP state (Core 1 only)
static struct {
    ptp_clock_identity_t clock_id;
    uint16_t sync_sequence;
    uint32_t sync_count;
    uint32_t followup_count;
    uint32_t delay_resp_count;
    uint32_t delay_req_count;
    uint8_t slave_mac[6];
    uint32_t slave_ip;
    bool slave_known;
} ptp_state = {0};

/**
 * Send PTP Sync + Follow_Up messages (Core 1)
 */
static void send_sync_and_followup(void) {
    if (!ptp_state.slave_known) {
        return;
    }

    uint8_t buffer[256];
    uint16_t frame_len;

    // Capture GPS-disciplined timestamp for Sync
    uint64_t sync_timestamp_ns = get_gps_time_ns();

    // Build Sync message
    ptp_sync_msg_t sync_msg;
    ptp_build_sync(&sync_msg, &ptp_state.clock_id, PTP_DOMAIN,
                   ptp_state.sync_sequence, sync_timestamp_ns);

    // Send Sync
    frame_len = eth_build_udp(buffer, ptp_state.slave_mac, ptp_state.slave_ip,
                              PTP_EVENT_PORT, PTP_EVENT_PORT,
                              (uint8_t*)&sync_msg, sizeof(sync_msg));
    w5500_send_frame(buffer, frame_len);
    ptp_state.sync_count++;

    // Capture precise GPS-disciplined timestamp for Follow_Up
    uint64_t followup_timestamp_ns = get_gps_time_ns();

    // Build Follow_Up message
    ptp_follow_up_msg_t followup_msg;
    ptp_build_follow_up(&followup_msg, &ptp_state.clock_id, PTP_DOMAIN,
                        ptp_state.sync_sequence, followup_timestamp_ns);

    // Send Follow_Up
    frame_len = eth_build_udp(buffer, ptp_state.slave_mac, ptp_state.slave_ip,
                              PTP_GENERAL_PORT, PTP_GENERAL_PORT,
                              (uint8_t*)&followup_msg, sizeof(followup_msg));
    w5500_send_frame(buffer, frame_len);
    ptp_state.followup_count++;

    ptp_state.sync_sequence++;
}

/**
 * Handle received PTP Delay_Req message (Core 1)
 */
static void handle_delay_req(const udp_packet_t *udp,
                             const uint8_t *src_mac, uint32_t src_ip) {
    if (udp->payload_len < sizeof(ptp_delay_req_msg_t)) {
        return;
    }

    ptp_delay_req_msg_t *delay_req = (ptp_delay_req_msg_t*)udp->payload;

    // Capture RX timestamp (t4) - GPS-disciplined from Core 0
    uint64_t rx_timestamp_ns = get_gps_time_ns();

    // Sequence ID byte swap
    uint16_t req_seq = (delay_req->header.sequence_id >> 8) |
                       ((delay_req->header.sequence_id & 0xFF) << 8);

    // Learn slave address from first Delay_Req
    if (!ptp_state.slave_known) {
        memcpy(ptp_state.slave_mac, src_mac, 6);
        ptp_state.slave_ip = src_ip;
        ptp_state.slave_known = true;
        printf("[Core 1] PTP slave discovered: %u.%u.%u.%u\n",
               (src_ip >> 24) & 0xFF, (src_ip >> 16) & 0xFF,
               (src_ip >> 8) & 0xFF, src_ip & 0xFF);
    }

    ptp_state.delay_req_count++;

    // Build Delay_Resp message
    ptp_delay_resp_msg_t delay_resp;
    ptp_build_delay_resp(&delay_resp, &ptp_state.clock_id, PTP_DOMAIN,
                         req_seq, rx_timestamp_ns,
                         &delay_req->header.source_port_identity);

    // Send Delay_Resp
    uint8_t buffer[256];
    uint16_t frame_len = eth_build_udp(buffer, src_mac, src_ip,
                                       PTP_GENERAL_PORT, PTP_GENERAL_PORT,
                                       (uint8_t*)&delay_resp, sizeof(delay_resp));
    w5500_send_frame(buffer, frame_len);
    ptp_state.delay_resp_count++;
}

/**
 * Core 1 Entry Point: Network and PTP Processing
 */
void core1_network_entry(void) {
    // Wait for Core 0 GPS discipline to be ready
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
    ptp_init_clock_identity(&ptp_state.clock_id, my_mac);

    printf("[Core 1] Network ready: IP=%s\n", MY_IP_ADDR);
    printf("[Core 1] Waiting for PTP slave...\n\n");

    uint64_t last_sync_time_us = 0;
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
                // Check if PTP event port (Delay_Req on port 319)
                if (udp.dest_port == PTP_EVENT_PORT) {
                    if (udp.payload_len >= 1) {
                        uint8_t msg_type = udp.payload[0] & 0x0F;
                        if (msg_type == PTP_MSGTYPE_DELAY_REQ) {
                            handle_delay_req(&udp, src_mac, src_ip);
                        }
                    }
                }
            }
        }

        // Send Sync + Follow_Up every 1 second
        if (now_us - last_sync_time_us >= 1000000) {
            send_sync_and_followup();
            last_sync_time_us = now_us;
        }

        // Print stats every 30 seconds
        if (now_us - last_stats_time_us >= 30000000) {
            // Get GPS discipline stats from Core 0
            extern volatile int32_t crystal_error_ns;
            extern volatile int64_t interpolation_error_ns;
            double crystal_ppm = (double)crystal_error_ns / 1000000.0;

            printf("[Core 1] PTP: Sync=%lu FollowUp=%lu DelayReq=%lu DelayResp=%lu Slave=%s\n",
                   ptp_state.sync_count, ptp_state.followup_count,
                   ptp_state.delay_req_count, ptp_state.delay_resp_count,
                   ptp_state.slave_known ? "YES" : "NO");
            printf("[Core 0] GPS: Lock=%s PPS=%lu crystal_err=%+ldns (%+.3fppm) interp_err=%+lldns\n\n",
                   core1_stats.locked ? "YES" : "NO",
                   core1_stats.pps_count,
                   (long)crystal_error_ns,
                   crystal_ppm,
                   (long long)interpolation_error_ns);
            last_stats_time_us = now_us;
        }

        // Minimal delay
        sleep_us(100);
    }
}

/**
 * Core 0 Main: GPS Discipline ONLY (Ultra-minimal like minimal_test)
 */
int main() {
    // Overclock to 250 MHz
    set_sys_clock_khz(250000, true);

    // Initialize stdio (for startup messages)
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== W5500 + GPS PTP Grandmaster - DUAL CORE ===\n");
    printf("System clock: %lu MHz\n\n", clock_get_hz(clk_sys) / 1000000);

    printf("[Core 0] GPS discipline core starting...\n");

    // Initialize debug GPIOs
    gpio_init(DEBUG_PPS_PIN);
    gpio_set_dir(DEBUG_PPS_PIN, GPIO_OUT);
    gpio_put(DEBUG_PPS_PIN, 0);
    gpio_init(DEBUG_LOCK_PIN);
    gpio_set_dir(DEBUG_LOCK_PIN, GPIO_OUT);
    gpio_put(DEBUG_LOCK_PIN, 0);

    // Initialize GPS module (NMEA parsing only, no PPS capture)
    printf("[Core 0] Initializing GPS...\n");
    gps_init(GPS_UART_ID, GPS_TX_PIN, GPS_RX_PIN, GPS_PPS_PIN, GPS_PIO, GPS_SM);

    // Initialize GPS discipline system
    printf("[Core 0] Initializing GPS discipline...\n");
    if (!discipline_init_v3()) {
        printf("[Core 0] FATAL: GPS discipline init failed\n");
        while (1) { sleep_ms(1000); }
    }
    printf("[Core 0] GPS discipline ready\n\n");

    // Initialize 1PPS scheduler for external measurement
    printf("[Core 0] Initializing 1PPS scheduler...\n");
    pps_scheduler_t pps_sched = {
        .pio = GPS_PIO,
        .sm = PPS_SCHEDULER_SM,
        .pin = PPS_OUTPUT_PIN,
        .get_time_ns = get_gps_time_ns,
        .get_scale_factor = gps_get_scale_factor  // GPS is always 1.0 (no crystal correction)
    };
    if (!pps_scheduler_init(&pps_sched)) {
        printf("[Core 0] WARNING: 1PPS scheduler init failed\n");
    }
    printf("[Core 0] 1PPS output on GPIO%d\n\n", PPS_OUTPUT_PIN);

    // Signal Core 1 that timing is ready
    core0_ready = true;

    // Launch Core 1 for network processing
    printf("[Core 0] Launching Core 1 for network/PTP...\n");
    multicore_launch_core1(core1_network_entry);

    printf("[Core 0] Entering minimal GPS discipline loop...\n\n");

    // ULTRA-MINIMAL LOOP (like minimal_test)
    // GPS NMEA processing + 1PPS scheduling
    uint64_t last_pps_schedule_us = 0;
    while (true) {
        // Process GPS NMEA sentences
        gps_process();

        // Schedule 1PPS only when close to second boundary
        uint64_t now_us = time_us_64();
        uint64_t gps_time_ns = get_gps_time_ns();
        uint64_t ns_in_second = gps_time_ns % 1000000000ULL;

        // Only schedule when in last 100ms of second AND at least 800ms since last schedule
        if (ns_in_second > 900000000ULL && (now_us - last_pps_schedule_us >= 800000)) {
            pps_scheduler_schedule_next(&pps_sched);
            last_pps_schedule_us = now_us;
        }

        // Sleep to minimize CPU interference
        sleep_ms(100);
    }

    return 0;
}

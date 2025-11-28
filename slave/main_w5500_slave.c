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

// W5500 INT pin (from w5500_simple.h)
#define W5500_PIN_INT 21

// PIO configuration (from ptp_discipline.c)
#define DISCIPLINE_PIO pio0
#define COUNTER_SM 0
#define INT_TIMESTAMP_SM 1

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

    // Enable W5500 interrupts for hardware timestamping
    w5500_enable_interrupts();

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

        // Drain hardware timestamp FIFO FIRST (so correlation can find them)
        extern bool read_int_hardware_timestamp(uint32_t *counter_value);
        uint32_t temp_counter;
        while (read_int_hardware_timestamp(&temp_counter)) {
            // Just drain into buffer - correlation will find them
        }

        // Clear and log W5500 interrupts (to release INT pin for PIO edge detection)
        uint8_t ir_flags = w5500_read_clear_interrupts();
        static uint32_t recv_int_count = 0;
        static uint32_t sendok_int_count = 0;
        if (ir_flags & 0x04) recv_int_count++;    // RECV interrupt
        if (ir_flags & 0x10) sendok_int_count++;  // SENDOK interrupt

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
                // Capture t3 BEFORE sending (software timestamp)
                extern uint64_t get_ptp_time_ns(void);
                extern bool find_hw_timestamp_for_rx(uint32_t, uint64_t, uint32_t*, int64_t*);
                extern int64_t counter_delta_to_ns(uint32_t, uint32_t);

                uint64_t t3_ptp_sw = get_ptp_time_ns();
                uint64_t t3_slave_us = time_us_64();

                // Atomically read counter before sending
                pio_sm_set_enabled(DISCIPLINE_PIO, COUNTER_SM, false);
                pio_sm_exec(DISCIPLINE_PIO, COUNTER_SM, pio_encode_mov(pio_isr, pio_x));
                pio_sm_exec(DISCIPLINE_PIO, COUNTER_SM, pio_encode_push(false, false));
                uint32_t counter_before = pio_sm_get(DISCIPLINE_PIO, COUNTER_SM);
                pio_sm_set_enabled(DISCIPLINE_PIO, COUNTER_SM, true);

                // Send the frame
                w5500_send_frame(tx_buffer, frame_len);

                // Small delay to let W5500 process and trigger INT
                sleep_us(100);

                // Try to find hardware timestamp for TX complete
                // Look in buffer for recent HW timestamp that occurred AFTER our software timestamp
                // Since counter counts down, HW counter will be SMALLER than our counter_before
                uint32_t counter_hw;
                int64_t tx_latency_ns = 0;
                bool hw_ts_valid = false;

                // Simple approach: assume most recent HW timestamp is our TX event
                extern bool read_int_hardware_timestamp(uint32_t *counter_value);
                uint32_t temp_counter;
                uint32_t latest_hw_counter = counter_before;
                while (read_int_hardware_timestamp(&temp_counter)) {
                    // Find the one closest to (but after) our transmission
                    if (temp_counter < counter_before) {
                        latest_hw_counter = temp_counter;
                        hw_ts_valid = true;
                    }
                }

                if (hw_ts_valid) {
                    tx_latency_ns = counter_delta_to_ns(counter_before, latest_hw_counter);
                    // Sanity check
                    if (tx_latency_ns > 0 && tx_latency_ns < 1000000) {  // < 1ms
                        ptp_sync_data.t3_ptp_ns = t3_ptp_sw + tx_latency_ns;  // Move forward to wire time
                    } else {
                        ptp_sync_data.t3_ptp_ns = t3_ptp_sw;  // Use software timestamp
                        hw_ts_valid = false;
                    }
                } else {
                    ptp_sync_data.t3_ptp_ns = t3_ptp_sw;  // Use software timestamp
                }

                ptp_sync_data.t3_slave_us = t3_slave_us;
                ptp_sync_data.tx_latency_ns = tx_latency_ns;
                ptp_sync_data.tx_hw_timestamp_valid = hw_ts_valid;
            }

            last_delay_req_time_us = now_us;
        }

        // Print stats every 30 seconds
        if (now_us - last_stats_time_us >= 30000000) {
            uint32_t sync_count, followup_count, delay_req_count, delay_resp_count;
            bool gm_known;
            ptp_slave_w5500_get_stats(&sync_count, &followup_count,
                                     &delay_req_count, &delay_resp_count, &gm_known);

            uint32_t outliers = ptp_discipline_get_outliers_rejected();
            printf("\n=== Stats: Sync=%lu FUp=%lu DReq=%lu DResp=%lu INT_RX=%lu Outliers=%lu ===\n",
                   sync_count, followup_count, delay_req_count, delay_resp_count, recv_int_count,
                   outliers);
            printf("    Lock=%s Off=%+lldns PD=%+lldns FreqOff=%+.1fppb Scale=%.6f\n\n",
                   ptp_stats.locked ? "YES" : "NO",
                   (long long)ptp_stats.offset_from_master_ns,
                   (long long)ptp_stats.mean_path_delay_ns,
                   ptp_stats.freq_offset_ppb,
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
    // (Core 1 now handles HW timestamp FIFO draining)

    while (true) {
        // Update discipline when new timestamp set available
        ptp_discipline_update();

        // Sleep to minimize CPU interference
        sleep_ms(10);
    }

    return 0;
}

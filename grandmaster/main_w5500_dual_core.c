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
#include "hw_timestamp.h"

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
    // HW timestamp stats
    uint32_t hw_ts_tx_success;
    uint32_t hw_ts_tx_fail;
    uint32_t hw_ts_rx_success;
    uint32_t hw_ts_rx_fail;
    uint32_t hw_ts_rx_fallback_count;  // Count of times we used average instead of actual HW
    int64_t last_tx_latency_ns;
    int64_t last_rx_latency_ns;
    // Running average of RX HW latencies (for fallback when HW correlation fails)
    int64_t rx_latency_avg_ns;
    uint32_t rx_latency_sample_count;
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

    // Capture HW counter BEFORE sending Sync
    uint32_t counter_before = hw_timestamp_read_counter();

    // Capture GPS-disciplined SW timestamp for Sync
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

    // Small delay to let W5500 complete TX and trigger INT
    sleep_us(150);

    // Drain PIO FIFO directly (same approach as slave)
    // Find most recent HW timestamp that occurred after our counter_before
    extern int64_t hw_timestamp_counter_to_ns(uint32_t, uint32_t);

    int64_t tx_latency_ns = 0;
    bool hw_ts_valid = false;
    uint32_t latest_hw_counter = counter_before;

    // Drain FIFO in loop (same as slave does for TX)
    // But also preserve any RX timestamps we find for later use
    extern void hw_timestamp_store_in_buffer(uint32_t counter_value);

    while (!pio_sm_is_rx_fifo_empty(pio1, 1)) {  // PIO1 SM1 = INT_TIMESTAMP_SM
        (void)pio_sm_get(pio1, 1);  // Discard marker

        // Read counter atomically
        pio_sm_set_enabled(pio1, 0, false);  // PIO1 SM0 = COUNTER_SM
        pio_sm_exec(pio1, 0, pio_encode_mov(pio_isr, pio_x));
        pio_sm_exec(pio1, 0, pio_encode_push(false, false));
        uint32_t temp_counter = pio_sm_get(pio1, 0);
        pio_sm_set_enabled(pio1, 0, true);

        // For TX: HW counter should be SMALLER than SW counter (counter counts down)
        if (temp_counter < counter_before) {
            latest_hw_counter = temp_counter;
            hw_ts_valid = true;
        } else {
            // This might be an RX timestamp - put it in buffer for later
            hw_timestamp_store_in_buffer(temp_counter);
        }
    }

    if (hw_ts_valid) {
        tx_latency_ns = hw_timestamp_counter_to_ns(counter_before, latest_hw_counter);
        if (tx_latency_ns > 0 && tx_latency_ns < 1000000) {  // Sanity: < 1ms
            sync_timestamp_ns += tx_latency_ns;
            ptp_state.hw_ts_tx_success++;
            ptp_state.last_tx_latency_ns = tx_latency_ns;
        } else {
            hw_ts_valid = false;
            ptp_state.hw_ts_tx_fail++;
        }
    } else {
        ptp_state.hw_ts_tx_fail++;
    }

    // Log every 10th Sync
    if (ptp_state.sync_count % 10 == 0) {
        printf("[GM] Sync #%u: TX_TS=%s (lat=%+lld ns)\n",
               ptp_state.sync_count,
               (tx_latency_ns != 0) ? "HW" : "SW",
               (long long)tx_latency_ns);
    }

    // PHASE 3: Detailed diagnostic every 100th Sync
    if (ptp_state.sync_count % 100 == 0) {
        printf("\n=== GM SYNC/FOLLOW_UP DIAGNOSTIC ===\n");
        printf("Sync correction_field: %lld ns (should be 0)\n",
               (long long)(sync_msg.header.correction_field >> 16));
        printf("Sync timestamp (t1 corrected): %llu ns\n", sync_timestamp_ns);
        printf("TX latency applied: %+lld ns\n", (long long)tx_latency_ns);
        printf("===================================\n\n");
    }

    // Build Follow_Up message with HW-corrected Sync timestamp
    ptp_follow_up_msg_t followup_msg;
    ptp_build_follow_up(&followup_msg, &ptp_state.clock_id, PTP_DOMAIN,
                        ptp_state.sync_sequence, sync_timestamp_ns);

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

    // Capture HW counter AFTER receiving Delay_Req
    uint32_t counter_after = hw_timestamp_read_counter();

    // Capture RX SW timestamp (t4) - GPS-disciplined from Core 0
    uint64_t rx_timestamp_ns = get_gps_time_ns();

    // Try to correlate with W5500 INT pin HW timestamp
    int64_t rx_latency_ns = 0;
    bool hw_ts_valid = hw_timestamp_find_rx(counter_after, &rx_latency_ns);
    bool used_average = false;

    if (hw_ts_valid) {
        // HW timestamp found - correct to wire time
        // rx_latency_ns is POSITIVE (processing delay from wire to SW capture)
        // Subtract to move timestamp BACKWARD to wire arrival time
        rx_timestamp_ns -= rx_latency_ns;
        ptp_state.hw_ts_rx_success++;
        ptp_state.last_rx_latency_ns = rx_latency_ns;

        // Update running average using exponential moving average (EMA)
        // alpha = 0.1 gives good balance between stability and responsiveness
        if (ptp_state.rx_latency_sample_count == 0) {
            // First sample: initialize average
            ptp_state.rx_latency_avg_ns = rx_latency_ns;
        } else {
            // EMA: avg_new = alpha * sample + (1-alpha) * avg_old
            // Using fixed-point: avg_new = (sample + 9*avg_old) / 10
            ptp_state.rx_latency_avg_ns = (rx_latency_ns + 9 * ptp_state.rx_latency_avg_ns) / 10;
        }
        ptp_state.rx_latency_sample_count++;
    } else {
        // HW timestamp correlation failed
        ptp_state.hw_ts_rx_fail++;

        // Use average latency as fallback (if we have samples)
        if (ptp_state.rx_latency_sample_count > 0) {
            rx_latency_ns = ptp_state.rx_latency_avg_ns;
            rx_timestamp_ns -= rx_latency_ns;
            ptp_state.hw_ts_rx_fallback_count++;
            used_average = true;
        }
        // else: no samples yet, fall back to SW timestamp (no correction)
    }

    // Log every 10th Delay_Req with debug info
    if (ptp_state.delay_req_count % 10 == 0) {
        const char *ts_type = hw_ts_valid ? "HW" : (used_average ? "AVG" : "SW");
        printf("[GM] Delay_Req #%u: RX_TS=%s (lat=%+lld ns) counter_after=%lu valid=%d\n",
               ptp_state.delay_req_count,
               ts_type,
               (long long)rx_latency_ns,
               counter_after,
               hw_ts_valid);
    }

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

    // PHASE 3: Log correction field for Delay_Resp
    static uint32_t dresp_log_count = 0;
    dresp_log_count++;
    if (dresp_log_count % 100 == 0) {
        printf("\n=== GM DELAY_RESP DIAGNOSTIC ===\n");
        printf("Delay_Resp correction_field: %lld ns (should be 0)\n",
               (long long)(delay_resp.header.correction_field >> 16));
        printf("Delay_Resp timestamp (t4 corrected): %llu ns\n", rx_timestamp_ns);
        printf("RX latency applied: %+lld ns\n", -(long long)rx_latency_ns);
        printf("================================\n\n");
    }

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

    // Enable W5500 interrupts for hardware timestamping
    printf("[Core 1] Enabling W5500 interrupts...\n");
    w5500_enable_interrupts();

    // Initialize HW timestamp system (requires W5500 INT pin enabled)
    printf("[Core 1] Initializing HW timestamp correlation...\n");
    if (!hw_timestamp_init()) {
        printf("[Core 1] WARNING: HW timestamp init failed, using SW timestamps only\n");
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

        // Drain HW timestamp FIFO FIRST (before processing packets)
        // This ensures timestamps are captured with minimal latency
        extern void hw_timestamp_poll_fifo(void);
        hw_timestamp_poll_fifo();

        // Clear W5500 interrupts to release INT pin for PIO edge detection
        // (INT pin stays LOW until interrupt flags are cleared)
        w5500_read_clear_interrupts();

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

            // HW timestamp statistics
            uint32_t tx_total = ptp_state.hw_ts_tx_success + ptp_state.hw_ts_tx_fail;
            uint32_t rx_total = ptp_state.hw_ts_rx_success + ptp_state.hw_ts_rx_fail;
            uint32_t tx_pct = tx_total > 0 ? (ptp_state.hw_ts_tx_success * 100) / tx_total : 0;
            uint32_t rx_pct = rx_total > 0 ? (ptp_state.hw_ts_rx_success * 100) / rx_total : 0;
            printf("[Core 1] HW_TS: TX=%lu/%lu (%lu%%) RX=%lu/%lu (%lu%%) | Last: TX=%+lld ns RX=%+lld ns\n",
                   ptp_state.hw_ts_tx_success, tx_total, tx_pct,
                   ptp_state.hw_ts_rx_success, rx_total, rx_pct,
                   (long long)ptp_state.last_tx_latency_ns,
                   (long long)ptp_state.last_rx_latency_ns);
            printf("[Core 1] RX_AVG: Fallback=%lu/%lu Avg=%+lld ns Samples=%lu\n",
                   ptp_state.hw_ts_rx_fallback_count, rx_total,
                   (long long)ptp_state.rx_latency_avg_ns,
                   ptp_state.rx_latency_sample_count);

            // Debug stats from HW timestamp system
            uint32_t debug_poll_count, debug_fifo_hits, debug_buffer_count;
            hw_timestamp_get_debug_stats(&debug_poll_count, &debug_fifo_hits, &debug_buffer_count);
            printf("[Core 1] HW_TS_DEBUG: Polls=%lu FIFO_hits=%lu Buffer=%lu\n",
                   debug_poll_count, debug_fifo_hits, debug_buffer_count);

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

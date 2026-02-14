/**
 * LTSP Grandmaster - Dual Core
 *
 * Core 0: GPS discipline + LTSP regression (same as PTP grandmaster)
 * Core 1: W5500 Ethernet + LTSP PDU transmission (1 Hz broadcast)
 *
 * Based on grandmaster/main_w5500_dual_core.c with PTP replaced by LTSP.
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
#include "gps.h"
#include "discipline_v3.h"
#include "shared_state.h"
#include "pps_scheduler.h"
#include "hw_timestamp.h"

/* LTSP protocol */
#include "../common/ltsp_pdu.h"
#include "../common/ltsp_pio_timestamp.h"

// Network configuration
#define MY_MAC          {0x00, 0x08, 0xDC, 0x12, 0x34, 0x01}
#define MY_IP_ADDR      "192.168.1.100"
#define MY_NETMASK      "255.255.255.0"
#define MY_GATEWAY      "192.168.1.1"

// GPS configuration (Core 0)
#define GPS_UART_ID     uart0
#define GPS_TX_PIN      0
#define GPS_RX_PIN      1
#define GPS_PPS_PIN     2
#define GPS_PIO         pio0
#define GPS_SM          0
#define GPS_PPS_CAPTURE_SM 1
#define DEBUG_PPS_PIN   4
#define DEBUG_LOCK_PIN  5

// 1PPS output pin
#define PPS_OUTPUT_PIN  15
#define PPS_SCHEDULER_SM 2

// Export debug pins for discipline module
uint debug_lock_pin = DEBUG_LOCK_PIN;
uint debug_pps_pin = DEBUG_PPS_PIN;

// Core synchronization
volatile bool core0_ready = false;

// Scale factor for 1PPS scheduler (GPS is always 1.0)
static double gps_get_scale_factor(void) {
    return 1.0;
}

// Convert IP string to uint32 (host byte order)
static uint32_t ip_str_to_u32(const char *ip_str) {
    uint32_t a, b, c, d;
    sscanf(ip_str, "%lu.%lu.%lu.%lu", &a, &b, &c, &d);
    return (a << 24) | (b << 16) | (c << 8) | d;
}

// LTSP TX state (Core 1)
static struct {
    uint16_t sequence;
    uint32_t tx_count;
    int64_t  prev_tx_timestamp_ns;   // GPS ns of previous packet's TX (deferred)
    // HW timestamp stats
    uint32_t hw_ts_tx_success;
    uint32_t hw_ts_tx_fail;
    int64_t  last_tx_latency_ns;
} ltsp_state = {0};

/**
 * Send one LTSP PDU (Core 1)
 *
 * Deferred timestamp model: prev_tx_timestamp carries the TX time of
 * packet N-1, determined AFTER sending N-1 (HW timestamp correlation).
 */
static void send_ltsp_pdu(const uint8_t *my_mac) {
    uint8_t frame[128];  // 14 + 56 = 70 bytes, pad to 128

    // --- Gather crystal model from Core 0 discipline ---
    double a0;
    float a1_ppb, sigma_ns;
    bool regression_valid = discipline_get_regression(&a0, &a1_ppb, &sigma_ns);

    int64_t last_1pps_counter;
    uint32_t last_1pps_interval;
    discipline_get_1pps_info(&last_1pps_counter, &last_1pps_interval);

    // --- Build PDU ---
    ltsp_pdu_t pdu = {
        .version            = LTSP_VERSION,
        .flags              = LTSP_FLAG_CRYSTAL_CLASS,
        .sequence           = ltsp_state.sequence,
        .prev_tx_timestamp  = ltsp_state.prev_tx_timestamp_ns,
        .model_epoch        = regression_valid ? (int64_t)(ltsp_pio_ticks_to_ns(
                                  (int64_t)last_1pps_counter)) : 0,
        .source_phase_bias  = regression_valid ? (int64_t)(a0 * LTSP_PIO_TICK_NS) : 0,
        .source_freq_drift  = regression_valid ? a1_ppb : 0.0f,
        .model_uncertainty  = regression_valid ? sigma_ns : 999999.0f,
        .last_1pps_count    = ltsp_pio_ticks_to_ns((int64_t)last_1pps_counter),
        .last_1pps_interval = last_1pps_interval,
        .gm_local_processing_mean = (uint32_t)hw_timestamp_get_tx_latency_mean_ns(),
    };

    if (!regression_valid) {
        pdu.flags &= ~LTSP_FLAG_CRYSTAL_CLASS;  // Not yet characterised
    }

    // --- Capture HW counter BEFORE sending ---
    uint32_t counter_before = hw_timestamp_read_counter();

    // --- Capture SW timestamp at point of send ---
    uint64_t sw_tx_ns = get_gps_time_ns();

    // --- Build and send frame ---
    uint8_t pdu_wire[LTSP_PDU_SIZE];
    ltsp_pdu_pack(&pdu, pdu_wire);
    uint16_t frame_len = eth_build_ltsp(frame, pdu_wire, LTSP_PDU_SIZE);
    w5500_send_frame(frame, frame_len);
    ltsp_state.tx_count++;
    ltsp_state.sequence++;

    // --- Determine actual TX time via HW timestamp ---
    sleep_us(150);  // Let W5500 complete TX and trigger INT

    int64_t tx_latency_ns = 0;
    bool hw_ts_valid = false;
    uint32_t latest_hw_counter = counter_before;

    // Drain FIFO for TX timestamps
    while (!pio_sm_is_rx_fifo_empty(pio1, 1)) {
        (void)pio_sm_get(pio1, 1);  // Discard marker

        pio_sm_set_enabled(pio1, 0, false);
        pio_sm_exec(pio1, 0, pio_encode_mov(pio_isr, pio_x));
        pio_sm_exec(pio1, 0, pio_encode_push(false, false));
        uint32_t temp_counter = pio_sm_get(pio1, 0);
        pio_sm_set_enabled(pio1, 0, true);

        if (temp_counter < counter_before) {
            latest_hw_counter = temp_counter;
            hw_ts_valid = true;
        } else {
            hw_timestamp_store_in_buffer(temp_counter);
        }
    }

    if (hw_ts_valid) {
        tx_latency_ns = hw_timestamp_counter_to_ns(counter_before, latest_hw_counter);
        if (tx_latency_ns > 0 && tx_latency_ns < 1000000) {
            sw_tx_ns += tx_latency_ns;
            ltsp_state.hw_ts_tx_success++;
            ltsp_state.last_tx_latency_ns = tx_latency_ns;
        } else {
            hw_ts_valid = false;
            ltsp_state.hw_ts_tx_fail++;
        }
    } else {
        ltsp_state.hw_ts_tx_fail++;
    }

    // Store TX timestamp for NEXT packet's prev_tx_timestamp field
    ltsp_state.prev_tx_timestamp_ns = (int64_t)sw_tx_ns;

    // Log every 10th PDU
    if (ltsp_state.tx_count % 10 == 0) {
        printf("[LTSP-GM] PDU #%u seq=%u TX_TS=%s (lat=%+lld ns) a1=%.3f ppb sigma=%.1f ns\n",
               ltsp_state.tx_count,
               (uint16_t)(ltsp_state.sequence - 1),
               hw_ts_valid ? "HW" : "SW",
               (long long)tx_latency_ns,
               (double)pdu.source_freq_drift,
               (double)pdu.model_uncertainty);
    }
}

/**
 * Core 1 Entry Point: Network and LTSP Processing
 */
void core1_network_entry(void) {
    while (!core0_ready) {
        tight_loop_contents();
    }

    printf("\n[Core 1] LTSP network core starting...\n");

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

    w5500_enable_interrupts();

    // Initialize HW timestamp system
    printf("[Core 1] Initializing HW timestamp correlation...\n");
    if (!hw_timestamp_init()) {
        printf("[Core 1] WARNING: HW timestamp init failed\n");
    }

    // Initialize Ethernet handler (for ARP/ICMP)
    eth_init(&net_cfg);

    printf("[Core 1] LTSP GM ready: IP=%s (LTSP broadcast on EtherType 0x88B5)\n", MY_IP_ADDR);

    uint64_t last_ltsp_time_us = 0;
    uint64_t last_stats_time_us = 0;
    uint8_t rx_buffer[W5500_MAX_FRAME_SIZE];
    bool link_reported = false;

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

        // Drain HW timestamp FIFO
        hw_timestamp_poll_fifo();
        w5500_read_clear_interrupts();

        // Process received frames (ARP/ICMP only, LTSP is one-way)
        uint16_t rx_len;
        if (w5500_recv_frame(rx_buffer, sizeof(rx_buffer), &rx_len)) {
            udp_packet_t udp;
            uint8_t src_mac[6];
            uint32_t src_ip;
            eth_handle_frame(rx_buffer, rx_len, &udp, src_mac, &src_ip);
        }

        // Send LTSP PDU every 1 second
        if (now_us - last_ltsp_time_us >= 1000000) {
            send_ltsp_pdu(my_mac);
            last_ltsp_time_us = now_us;
        }

        // Print stats every 30 seconds
        if (now_us - last_stats_time_us >= 30000000) {
            extern volatile int32_t crystal_error_ns;
            extern volatile int64_t interpolation_error_ns;

            double a0;
            float a1_ppb, sigma_ns;
            bool reg_valid = discipline_get_regression(&a0, &a1_ppb, &sigma_ns);

            printf("[LTSP-GM] TX: %lu PDUs | HW_TS: %lu/%lu success\n",
                   ltsp_state.tx_count,
                   ltsp_state.hw_ts_tx_success,
                   ltsp_state.hw_ts_tx_success + ltsp_state.hw_ts_tx_fail);

            if (reg_valid) {
                printf("[LTSP-GM] Regression: a0=%.1f ticks a1=%.3f ppb sigma=%.1f ns (valid)\n",
                       a0, (double)a1_ppb, (double)sigma_ns);
            } else {
                printf("[LTSP-GM] Regression: not yet valid\n");
            }

            printf("[Core 0] GPS: Lock=%s PPS=%lu crystal_err=%+ld ns interp_err=%+lld ns\n\n",
                   core1_stats.locked ? "YES" : "NO",
                   core1_stats.pps_count,
                   (long)crystal_error_ns,
                   (long long)interpolation_error_ns);
            last_stats_time_us = now_us;
        }

        sleep_us(100);
    }
}

/**
 * Core 0 Main: GPS Discipline + LTSP Regression
 */
int main() {
    set_sys_clock_khz(250000, true);

    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== LTSP Grandmaster - Dual Core ===\n");
    printf("System clock: %lu MHz\n\n", clock_get_hz(clk_sys) / 1000000);

    // Initialize debug GPIOs
    gpio_init(DEBUG_PPS_PIN);
    gpio_set_dir(DEBUG_PPS_PIN, GPIO_OUT);
    gpio_put(DEBUG_PPS_PIN, 0);
    gpio_init(DEBUG_LOCK_PIN);
    gpio_set_dir(DEBUG_LOCK_PIN, GPIO_OUT);
    gpio_put(DEBUG_LOCK_PIN, 0);

    // Initialize GPS
    printf("[Core 0] Initializing GPS...\n");
    gps_init(GPS_UART_ID, GPS_TX_PIN, GPS_RX_PIN, GPS_PPS_PIN, GPS_PIO, GPS_SM);

    // Initialize GPS discipline (includes LTSP regression init)
    printf("[Core 0] Initializing GPS discipline...\n");
    if (!discipline_init_v3()) {
        printf("[Core 0] FATAL: GPS discipline init failed\n");
        while (1) { sleep_ms(1000); }
    }

    // Initialize 1PPS scheduler
    printf("[Core 0] Initializing 1PPS scheduler...\n");
    pps_scheduler_t pps_sched = {
        .pio = GPS_PIO,
        .sm = PPS_SCHEDULER_SM,
        .pin = PPS_OUTPUT_PIN,
        .get_time_ns = get_gps_time_ns,
        .get_scale_factor = gps_get_scale_factor
    };
    if (!pps_scheduler_init(&pps_sched)) {
        printf("[Core 0] WARNING: 1PPS scheduler init failed\n");
    }
    printf("[Core 0] 1PPS output on GPIO%d\n\n", PPS_OUTPUT_PIN);

    // Signal Core 1
    core0_ready = true;

    // Launch Core 1
    printf("[Core 0] Launching Core 1 for network/LTSP...\n");
    multicore_launch_core1(core1_network_entry);

    printf("[Core 0] Entering GPS discipline + regression loop...\n\n");

    uint64_t last_pps_schedule_us = 0;
    while (true) {
        gps_process();

        // Recompute regression if new 1PPS sample pending
        discipline_recompute_regression();

        // Schedule 1PPS near second boundary
        uint64_t now_us = time_us_64();
        uint64_t gps_time_ns = get_gps_time_ns();
        uint64_t ns_in_second = gps_time_ns % 1000000000ULL;

        if (ns_in_second > 900000000ULL && (now_us - last_pps_schedule_us >= 800000)) {
            pps_scheduler_schedule_next(&pps_sched);
            last_pps_schedule_us = now_us;
        }

        sleep_ms(100);
    }

    return 0;
}

/**
 * LTSP Receiver - Dual Core
 *
 * Core 0: PIO counter management + HW timestamp FIFO drain
 * Core 1: W5500 Ethernet + LTSP PDU reception + CSV output
 *
 * v0.2: Drift-compensated min filter. No clock discipline yet.
 *
 * Key insight: HW timestamps on both GM and receiver already capture
 * wire-departure and wire-arrival times, so d_total is wire-to-wire.
 * No need to decompose into d_gm_local / d_rx_local for the offset
 * calculation. A regression on d_total characterises the relative
 * crystal drift, and de-trending before the min filter gives clean
 * jitter measurements.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "w5500_simple.h"
#include "eth_simple.h"

/* LTSP protocol */
#include "../common/ltsp_pdu.h"
#include "../common/ltsp_sequence.h"
#include "../common/ltsp_min_filter.h"
#include "../common/ltsp_pio_timestamp.h"
#include "../common/ltsp_regression.h"

/* PIO programs (same as slave: counter + INTn timestamp) */
#include "counter_simple.pio.h"
#include "int_timestamp.pio.h"

// Network configuration
#define MY_MAC          {0x00, 0x08, 0xDC, 0x12, 0x34, 0x02}
#define MY_IP_ADDR      "192.168.1.102"
#define MY_NETMASK      "255.255.255.0"
#define MY_GATEWAY      "192.168.1.1"

// PIO configuration (PIO0 for both counter and INTn capture)
#define RX_PIO          pio0
#define COUNTER_SM      0
#define INT_TIMESTAMP_SM 1

// W5500 INT pin
#define W5500_PIN_INT   21

// NS per PIO tick
#define NS_PER_TICK     12

// Core synchronization
volatile bool core0_ready = false;

// HW timestamp circular buffer (shared between cores)
#define TS_BUFFER_SIZE 16
static volatile uint32_t ts_buffer[TS_BUFFER_SIZE];
static volatile uint8_t ts_write_idx = 0;
static volatile uint8_t ts_read_idx = 0;

// PIO 64-bit timestamp extension for RX
static ltsp_pio_ts_t rx_pio_ts;

// Per-packet state for deferred timestamp processing
typedef struct {
    uint16_t sequence;
    int64_t  t_rx_ns;           // Monotonically increasing receiver time (ns)
    bool     valid;
} rx_record_t;

// We keep the previous packet's RX record for deferred processing
static rx_record_t prev_rx = {0};

// LTSP state
static ltsp_seq_state_t seq_state;
static int64_t min_filter_buf[LTSP_MIN_FILTER_DEFAULT_WINDOW];
static ltsp_min_filter_t min_filter;

// Drift regression: fits d_total = a0 + a1 * sample_index
// a1 gives relative crystal drift in ns/s
static ltsp_regression_sample_t drift_reg_buf[LTSP_REGRESSION_DEFAULT_WINDOW];
static ltsp_regression_t drift_reg;
static int64_t d_total_ref_ns = 0;     // First d_total, for numerical stability
static bool d_total_ref_set = false;
static uint32_t drift_sample_count = 0;

// Stats
static uint32_t pdu_count = 0;
static uint32_t csv_line_count = 0;
static uint32_t seq_gap_count = 0;
static uint32_t seq_dup_count = 0;
static uint32_t deferred_skip_count = 0;

/**
 * Read current PIO counter (atomic snapshot)
 */
static uint32_t read_counter(void) {
    pio_sm_set_enabled(RX_PIO, COUNTER_SM, false);
    pio_sm_exec(RX_PIO, COUNTER_SM, pio_encode_mov(pio_isr, pio_x));
    pio_sm_exec(RX_PIO, COUNTER_SM, pio_encode_push(false, false));
    uint32_t counter = pio_sm_get(RX_PIO, COUNTER_SM);
    pio_sm_set_enabled(RX_PIO, COUNTER_SM, true);
    return counter;
}

/**
 * Drain INTn HW timestamp FIFO into circular buffer.
 * Called frequently from Core 0.
 */
static void drain_hw_timestamp_fifo(void) {
    while (!pio_sm_is_rx_fifo_empty(RX_PIO, INT_TIMESTAMP_SM)) {
        // Discard marker from FIFO
        (void)pio_sm_get(RX_PIO, INT_TIMESTAMP_SM);

        // Atomic counter snapshot
        uint32_t counter = read_counter();

        // Store in circular buffer
        ts_buffer[ts_write_idx] = counter;
        ts_write_idx = (ts_write_idx + 1) % TS_BUFFER_SIZE;
        if (ts_write_idx == ts_read_idx) {
            ts_read_idx = (ts_read_idx + 1) % TS_BUFFER_SIZE;
        }
    }
}

/**
 * Find most recent HW timestamp from buffer.
 * Returns true if found, writes counter value.
 */
static bool find_rx_hw_timestamp(uint32_t sw_counter, uint32_t *hw_counter) {
    uint8_t wr = ts_write_idx;
    uint8_t rd = ts_read_idx;

    if (wr == rd) return false;

    // Search backwards for most recent timestamp with correct direction
    // For RX: HW timestamp BEFORE SW (counter counts down, so HW > SW)
    uint32_t best = 0;
    int64_t best_delta = INT64_MAX;
    bool found = false;

    uint8_t count = (wr >= rd) ? (wr - rd) : (TS_BUFFER_SIZE - rd + wr);
    for (uint8_t i = 0; i < count; i++) {
        uint8_t idx = (wr - 1 - i + TS_BUFFER_SIZE) % TS_BUFFER_SIZE;
        uint32_t ts = ts_buffer[idx];

        // HW should be before SW (higher counter value since counting down)
        if (ts >= sw_counter) {
            uint32_t delta = ts - sw_counter;
            int64_t delta_ns = (int64_t)delta * NS_PER_TICK;
            // Sanity: < 10ms
            if (delta_ns < 10000000 && delta_ns < best_delta) {
                best = ts;
                best_delta = delta_ns;
                found = true;
            }
        }
    }

    if (found) {
        *hw_counter = best;
    }
    return found;
}

// Convert IP string to uint32 (host byte order)
static uint32_t ip_str_to_u32(const char *ip_str) {
    uint32_t a, b, c, d;
    sscanf(ip_str, "%lu.%lu.%lu.%lu", &a, &b, &c, &d);
    return (a << 24) | (b << 16) | (c << 8) | d;
}

/**
 * Process a received LTSP PDU.
 * Handles deferred timestamp model, drift regression, and CSV output.
 */
static void process_ltsp_pdu(const uint8_t *pdu_payload, uint32_t hw_counter_at_rx) {
    ltsp_pdu_t pdu;
    if (!ltsp_pdu_unpack(pdu_payload, &pdu)) {
        printf("ERR: PDU unpack failed\n");
        return;
    }

    pdu_count++;

    // Validate sequence
    uint16_t gap_size = 0;
    ltsp_seq_result_t seq_result = ltsp_seq_validate(&seq_state, pdu.sequence, &gap_size);
    if (seq_result == LTSP_SEQ_DUPLICATE) {
        seq_dup_count++;
        return;
    }
    if (seq_result == LTSP_SEQ_GAP) {
        seq_gap_count++;
        // Continue processing — don't discard
    }

    // Extend PIO counter to monotonically increasing 64-bit tick count
    ltsp_pio_ts_extend(&rx_pio_ts, hw_counter_at_rx);
    int64_t t_rx_ns = (int64_t)ltsp_pio_ts_to_monotonic(&rx_pio_ts) * NS_PER_TICK;

    // --- Deferred timestamp processing ---
    // PDU N contains Prev_Tx_Timestamp = TX time of packet N-1 (GPS ns)
    // We need our stored RX record for packet N-1 to compute delay
    if (prev_rx.valid && pdu.prev_tx_timestamp != 0) {
        // d_total is wire-to-wire: HW timestamps already captured
        // GM side (prev_tx_timestamp = sw_time + HW tx_latency)
        // RX side (t_rx_ns = PIO counter at INTn edge)
        int64_t d_total_ns = prev_rx.t_rx_ns - pdu.prev_tx_timestamp;

        // Reference d_total for numerical stability in regression
        if (!d_total_ref_set) {
            d_total_ref_ns = d_total_ns;
            d_total_ref_set = true;
        }
        int64_t d_total_rel = d_total_ns - d_total_ref_ns;

        // Feed into drift regression: fits d_total_rel = a0 + a1 * k
        // a1 gives relative crystal drift in ns/sample (≈ ns/second)
        ltsp_regression_add_sample(&drift_reg, (int64_t)drift_sample_count, d_total_rel);
        if (drift_reg.count >= 2) {
            ltsp_regression_compute(&drift_reg);
        }

        // De-trend d_total: remove the linear drift to isolate jitter
        // Only valid once regression window is full (60 samples)
        int64_t d_detrended = 0;
        int64_t offset_ns = 0;
        bool detrend_valid = drift_reg.result.valid &&
                             drift_reg.count >= LTSP_REGRESSION_DEFAULT_WINDOW;
        if (detrend_valid) {
            double predicted = ltsp_regression_a0_at(&drift_reg, (int64_t)drift_sample_count);
            d_detrended = d_total_rel - (int64_t)predicted;

            // Min filter on de-trended values: minimum ≈ 0 (best-case jitter)
            // offset = current - min ≈ excess jitter on this packet
            ltsp_min_filter_update(&min_filter, d_detrended);
            int64_t d_min = ltsp_min_filter_get_min(&min_filter);
            offset_ns = d_detrended - d_min;
        }

        // Drift rate from regression (ns/s, approximately ppm * 1000)
        double drift_ns_per_s = drift_reg.result.valid ? drift_reg.result.a1 : 0.0;
        // Regression sigma (residual std dev) — already in ns since inputs are ns
        double drift_sigma_ns = drift_reg.result.valid ? drift_reg.result.sigma : 999999.0;

        // CSV output
        // seq,d_total_ns,d_detrended_ns,offset_ns,drift_ns_per_s,drift_sigma_ns,
        // gm_sigma_ns,gm_a1_ppb,1pps_interval_ticks
        printf("%u,%lld,%lld,%lld,%.1f,%.1f,%.1f,%.3f,%lu\n",
               prev_rx.sequence,
               (long long)d_total_ns,
               (long long)d_detrended,
               (long long)offset_ns,
               drift_ns_per_s,
               drift_sigma_ns,
               (double)pdu.model_uncertainty,
               (double)pdu.source_freq_drift,
               (unsigned long)pdu.last_1pps_interval);
        csv_line_count++;
        drift_sample_count++;
    } else {
        if (pdu.prev_tx_timestamp == 0) {
            deferred_skip_count++;
        }
    }

    // Store this packet's RX record for deferred processing with NEXT packet
    prev_rx.sequence = pdu.sequence;
    prev_rx.t_rx_ns = t_rx_ns;
    prev_rx.valid = true;
}

/**
 * Core 1 Entry Point: Network and LTSP Reception
 */
void core1_network_entry(void) {
    while (!core0_ready) {
        tight_loop_contents();
    }

    printf("\n[Core 1] LTSP receiver network core starting...\n");

    uint8_t my_mac[] = MY_MAC;
    net_config_t net_cfg = {
        .ip = ip_str_to_u32(MY_IP_ADDR),
        .netmask = ip_str_to_u32(MY_NETMASK),
        .gateway = ip_str_to_u32(MY_GATEWAY)
    };
    memcpy(net_cfg.mac, my_mac, 6);

    // Initialize W5500
    if (!w5500_simple_init(my_mac)) {
        printf("[Core 1] FATAL: W5500 init failed\n");
        while (1) { sleep_ms(1000); }
    }
    w5500_enable_interrupts();
    eth_init(&net_cfg);

    // Initialize drift regression
    ltsp_regression_init(&drift_reg, drift_reg_buf, LTSP_REGRESSION_DEFAULT_WINDOW);

    printf("[Core 1] LTSP Receiver ready: IP=%s\n", MY_IP_ADDR);

    // Print CSV header
    printf("# seq,d_total_ns,d_detrended_ns,offset_ns,"
           "drift_ns_per_s,drift_sigma_ns,gm_sigma_ns,gm_a1_ppb,"
           "1pps_interval_ticks\n");

    uint64_t last_stats_time_us = 0;
    uint8_t rx_buffer[W5500_MAX_FRAME_SIZE];
    bool link_reported = false;

    while (true) {
        uint64_t now_us = time_us_64();

        // Check link
        bool link_up = w5500_link_up();
        if (link_up && !link_reported) {
            printf("# Link UP\n");
            link_reported = true;
        } else if (!link_up && link_reported) {
            printf("# Link DOWN\n");
            link_reported = false;
        }
        if (!link_up) { sleep_ms(100); continue; }

        // Drain HW timestamp FIFO
        drain_hw_timestamp_fifo();
        w5500_read_clear_interrupts();

        // Check for received frames
        uint16_t rx_len;
        if (w5500_recv_frame(rx_buffer, sizeof(rx_buffer), &rx_len)) {
            // Capture SW counter immediately after read
            uint32_t sw_counter = read_counter();

            // Try HW timestamp first
            uint32_t hw_counter;
            uint32_t rx_counter;
            if (find_rx_hw_timestamp(sw_counter, &hw_counter)) {
                rx_counter = hw_counter;
            } else {
                rx_counter = sw_counter;  // Fallback
            }

            // Check if LTSP frame (EtherType 0x88B5)
            const uint8_t *pdu_payload = ltsp_frame_check(rx_buffer, rx_len);
            if (pdu_payload) {
                process_ltsp_pdu(pdu_payload, rx_counter);
            } else {
                // Handle ARP/ICMP
                udp_packet_t udp;
                uint8_t src_mac[6];
                uint32_t src_ip;
                eth_handle_frame(rx_buffer, rx_len, &udp, src_mac, &src_ip);
            }
        }

        // Stats every 60 seconds (prefixed with # so CSV parsers ignore)
        if (now_us - last_stats_time_us >= 60000000) {
            int64_t d_min = ltsp_min_filter_get_min(&min_filter);
            double drift = drift_reg.result.valid ? drift_reg.result.a1 : 0.0;
            double sigma = drift_reg.result.valid ? drift_reg.result.sigma : 999999.0;
            printf("# STATS: PDUs=%lu CSV=%lu Gaps=%lu Dups=%lu Skip=%lu "
                   "d_min=%lld ns drift=%.1f ns/s sigma=%.1f ns\n",
                   pdu_count, csv_line_count, seq_gap_count, seq_dup_count,
                   deferred_skip_count, (long long)d_min,
                   drift, sigma);
            last_stats_time_us = now_us;
        }

        sleep_us(100);
    }
}

/**
 * Core 0: PIO initialization + FIFO drain loop
 */
int main() {
    set_sys_clock_khz(250000, true);

    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== LTSP Receiver v0.2 - Dual Core ===\n");
    printf("System clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);

    // Initialize PIO counter (free-running 83.33 MHz)
    uint counter_offset = pio_add_program(RX_PIO, &counter_simple_program);
    counter_simple_program_init(RX_PIO, COUNTER_SM, counter_offset);
    printf("[Core 0] PIO counter initialized (12ns resolution)\n");

    // Initialize INTn timestamp capture
    uint int_ts_offset = pio_add_program(RX_PIO, &int_timestamp_program);
    int_timestamp_program_init(RX_PIO, INT_TIMESTAMP_SM, int_ts_offset, W5500_PIN_INT);
    printf("[Core 0] INTn timestamp capture on GPIO %d\n", W5500_PIN_INT);

    // Initialize LTSP modules
    ltsp_pio_ts_init(&rx_pio_ts);
    ltsp_seq_init(&seq_state);
    ltsp_min_filter_init(&min_filter, min_filter_buf, LTSP_MIN_FILTER_DEFAULT_WINDOW);
    printf("[Core 0] LTSP modules initialized\n");

    // Signal Core 1
    core0_ready = true;

    // Launch Core 1
    multicore_launch_core1(core1_network_entry);

    printf("[Core 0] Entering FIFO drain loop...\n\n");

    // Core 0 minimal loop: just drain HW timestamp FIFO
    while (true) {
        drain_hw_timestamp_fifo();
        sleep_ms(1);
    }

    return 0;
}

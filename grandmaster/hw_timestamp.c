/**
 * Hardware Timestamp Correlation for Grandmaster
 * Uses polling approach (same as slave)
 */

#include "hw_timestamp.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "counter_simple.pio.h"
#include "int_timestamp.pio.h"
#include "w5500_simple.h"
#include "pico/stdlib.h"
#include <stdio.h>

// PIO configuration - use PIO1 (PIO0 used by GPS discipline)
#define HW_TS_PIO pio1
#define COUNTER_SM 0
#define INT_TIMESTAMP_SM 1

// Circular buffer for INT timestamps
#define TS_BUFFER_SIZE 16
#define TS_MAX_AGE_TICKS (8333333)  // 100ms at 12ns/tick = ~8.3M ticks

static volatile uint32_t ts_buffer[TS_BUFFER_SIZE];
static volatile uint8_t ts_write_idx = 0;
static volatile uint8_t ts_read_idx = 0;

// Debug: polling stats
static volatile uint32_t poll_count = 0;
static volatile uint32_t fifo_hits = 0;

// Counter resolution: 12ns per tick (83.33 MHz = system clock / 3)
#define NS_PER_TICK 12

/**
 * Store a timestamp directly into the circular buffer
 * Used when timestamps are read outside the normal polling flow
 */
void hw_timestamp_store_in_buffer(uint32_t counter_value) {
    ts_buffer[ts_write_idx] = counter_value;
    ts_write_idx = (ts_write_idx + 1) % TS_BUFFER_SIZE;

    // If buffer full, advance read pointer (overwrite oldest)
    if (ts_write_idx == ts_read_idx) {
        ts_read_idx = (ts_read_idx + 1) % TS_BUFFER_SIZE;
    }
}

/**
 * Poll for INT timestamps and store in buffer
 * Public function - call this frequently from main loop
 */
void hw_timestamp_poll_fifo(void) {
    poll_count++;

    // Check if INT timestamp available
    if (!pio_sm_is_rx_fifo_empty(HW_TS_PIO, INT_TIMESTAMP_SM)) {
        fifo_hits++;

        // Discard marker from FIFO
        (void)pio_sm_get(HW_TS_PIO, INT_TIMESTAMP_SM);

        // Atomically read counter SM (same method as slave)
        pio_sm_set_enabled(HW_TS_PIO, COUNTER_SM, false);
        pio_sm_exec(HW_TS_PIO, COUNTER_SM, pio_encode_mov(pio_isr, pio_x));
        pio_sm_exec(HW_TS_PIO, COUNTER_SM, pio_encode_push(false, false));
        uint32_t counter_value = pio_sm_get(HW_TS_PIO, COUNTER_SM);
        pio_sm_set_enabled(HW_TS_PIO, COUNTER_SM, true);

        // Store in circular buffer
        ts_buffer[ts_write_idx] = counter_value;
        ts_write_idx = (ts_write_idx + 1) % TS_BUFFER_SIZE;

        // If buffer full, advance read pointer (overwrite oldest)
        if (ts_write_idx == ts_read_idx) {
            ts_read_idx = (ts_read_idx + 1) % TS_BUFFER_SIZE;
        }
    }
}

/**
 * Initialize hardware timestamp system
 */
bool hw_timestamp_init(void) {
    printf("[HW_TS] ========================================\n");
    printf("[HW_TS] Initializing hardware timestamp system\n");
    printf("[HW_TS] ========================================\n");
    printf("[HW_TS] PIO: PIO1\n");
    printf("[HW_TS] INT Pin: GPIO %d\n", W5500_PIN_INT);
    printf("[HW_TS] Mode: POLLING (same as slave)\n");

    // Load PIO programs
    uint counter_offset = pio_add_program(HW_TS_PIO, &counter_simple_program);
    uint int_ts_offset = pio_add_program(HW_TS_PIO, &int_timestamp_program);

    printf("[HW_TS] Counter SM @ offset %d (12ns resolution)\n", counter_offset);
    printf("[HW_TS] INT capture SM @ offset %d\n", int_ts_offset);

    // Initialize counter SM (free-running 12ns counter)
    counter_simple_program_init(HW_TS_PIO, COUNTER_SM, counter_offset);

    // Initialize INT timestamp SM (captures counter on INT pin edge)
    int_timestamp_program_init(HW_TS_PIO, INT_TIMESTAMP_SM, int_ts_offset, W5500_PIN_INT);

    // Initialize buffer
    ts_write_idx = 0;
    ts_read_idx = 0;

    printf("[HW_TS] ========================================\n");
    printf("[HW_TS] HARDWARE TIMESTAMP SYSTEM READY!\n");
    printf("[HW_TS] Monitoring GPIO %d for W5500 INT\n", W5500_PIN_INT);
    printf("[HW_TS] ========================================\n");

    // Give user time to see this important message
    sleep_ms(2000);

    return true;
}

/**
 * Read current PIO counter value
 * NOTE: Does NOT poll for new timestamps - caller should poll separately
 */
uint32_t hw_timestamp_read_counter(void) {
    // Atomic counter snapshot: Pause SM, read X register, resume SM
    pio_sm_set_enabled(HW_TS_PIO, COUNTER_SM, false);
    pio_sm_exec(HW_TS_PIO, COUNTER_SM, pio_encode_mov(pio_isr, pio_x));
    pio_sm_exec(HW_TS_PIO, COUNTER_SM, pio_encode_push(false, false));
    uint32_t counter = pio_sm_get(HW_TS_PIO, COUNTER_SM);
    pio_sm_set_enabled(HW_TS_PIO, COUNTER_SM, true);

    return counter;
}

/**
 * Convert counter delta to nanoseconds
 * Counter counts DOWN
 */
int64_t hw_timestamp_counter_to_ns(uint32_t counter_before, uint32_t counter_after) {
    uint32_t delta_ticks;

    if (counter_before > counter_after) {
        // Normal case: counter counted down
        delta_ticks = counter_before - counter_after;
    } else {
        // Wraparound case
        delta_ticks = counter_before + (0xFFFFFFFF - counter_after) + 1;
    }

    return (int64_t)(delta_ticks * NS_PER_TICK);
}

/**
 * Find closest hardware timestamp to software timestamp
 * Searches all timestamps in buffer and picks the one with minimum absolute delta
 *
 * @param counter_sw Software counter value
 * @param counter_hw Output: Hardware counter value if found
 * @param delta_ns Output: Delta in nanoseconds (positive or negative)
 * @param expect_positive_delta true for RX (HW before SW), false for TX (HW after SW)
 * @return true if suitable timestamp found
 */
static bool find_closest_hw_timestamp(uint32_t counter_sw, uint32_t *counter_hw,
                                      int64_t *delta_ns, bool expect_positive_delta) {
    // Poll for new timestamps (should already be done in main loop, but doesn't hurt)
    hw_timestamp_poll_fifo();

    // Age out old timestamps (older than 100ms)
    // Counter counts DOWN: newer values are SMALLER
    // Use same delta calculation as timestamp correlation (handles wraparound)
    while (ts_read_idx != ts_write_idx) {
        uint32_t oldest_ts = ts_buffer[ts_read_idx];

        // Calculate age in nanoseconds (oldest_ts is "before", counter_sw is "after")
        int64_t age_ns = hw_timestamp_counter_to_ns(oldest_ts, counter_sw);

        // Remove if older than 100ms
        // age_ns is always positive (wraparound handled in hw_timestamp_counter_to_ns)
        if (age_ns > (TS_MAX_AGE_TICKS * NS_PER_TICK)) {
            ts_read_idx = (ts_read_idx + 1) % TS_BUFFER_SIZE;
        } else {
            break;  // Rest of buffer is newer (buffer is FIFO)
        }
    }

    // Calculate buffer count
    uint8_t count;
    if (ts_write_idx >= ts_read_idx) {
        count = ts_write_idx - ts_read_idx;
    } else {
        count = TS_BUFFER_SIZE - ts_read_idx + ts_write_idx;
    }

    if (count == 0) {
        // Debug empty buffer for TX searches (they're failing)
        if (!expect_positive_delta) {
            static uint32_t tx_empty_count = 0;
            tx_empty_count++;
            if (tx_empty_count % 10 == 0) {
                printf("[FIND_CLOSEST] TX search: buffer EMPTY (count=0)\n");
            }
        }
        return false;  // Buffer empty
    }

    static uint32_t search_count = 0;
    search_count++;
    bool debug_this = (search_count % 20 == 0);  // Debug every 20th search

    if (debug_this) {
        printf("[FIND_CLOSEST] counter_sw=0x%08lX buffer_count=%u expect=%s\n",
               counter_sw, count, expect_positive_delta ? "RX(+)" : "TX(-)");
    }

    // Search all timestamps for closest match with correct direction
    uint32_t best_counter = 0;
    int64_t best_delta_abs = INT64_MAX;
    int64_t best_delta_signed = 0;
    bool found = false;

    for (uint8_t i = 0; i < count; i++) {
        // Search backwards from write index
        uint8_t idx = (ts_write_idx - 1 - i + TS_BUFFER_SIZE) % TS_BUFFER_SIZE;
        uint32_t ts_counter = ts_buffer[idx];

        // Calculate delta (can be positive or negative)
        int64_t delta;
        if (ts_counter >= counter_sw) {
            // HW before SW (normal case for RX) - counter counts down
            delta = hw_timestamp_counter_to_ns(ts_counter, counter_sw);
        } else {
            // HW after SW (normal case for TX)
            delta = -hw_timestamp_counter_to_ns(counter_sw, ts_counter);
        }

        // DIRECTION FILTER: Skip timestamps with wrong sign
        // This prevents TX searches from finding RX timestamps and vice versa
        if (expect_positive_delta && delta <= 0) {
            if (debug_this) {
                printf("[FIND_CLOSEST]   [%u] ts=0x%08lX delta=%+lld ns (SKIP: wrong sign)\n",
                       i, ts_counter, (long long)delta);
            }
            continue;  // Skip negative deltas when expecting positive (RX)
        }
        if (!expect_positive_delta && delta >= 0) {
            if (debug_this) {
                printf("[FIND_CLOSEST]   [%u] ts=0x%08lX delta=%+lld ns (SKIP: wrong sign)\n",
                       i, ts_counter, (long long)delta);
            }
            continue;  // Skip positive deltas when expecting negative (TX)
        }

        int64_t delta_abs = (delta < 0) ? -delta : delta;

        if (debug_this) {
            printf("[FIND_CLOSEST]   [%u] ts=0x%08lX delta=%+lld ns (%s)\n",
                   i, ts_counter, (long long)delta,
                   (delta_abs < best_delta_abs) ? "BETTER" : "worse");
        }

        // Sanity check: delta should be reasonable (< 10ms)
        if (delta_abs < 10000000) {
            // Closer than previous best?
            if (delta_abs < best_delta_abs) {
                best_delta_abs = delta_abs;
                best_delta_signed = delta;
                best_counter = ts_counter;
                found = true;
            }
        }
    }

    if (found) {
        *counter_hw = best_counter;
        *delta_ns = best_delta_signed;

        // DON'T clear buffer - let timestamps accumulate and age out naturally
        // This prevents TX searches from clearing RX timestamps that arrive close together
        return true;
    }

    return false;
}

/**
 * Find hardware timestamp for TX event
 */
bool hw_timestamp_find_tx(uint32_t counter_before, int64_t *latency_ns) {
    uint32_t counter_hw;
    int64_t delta_ns;

    // For TX: expect NEGATIVE delta (HW after SW, since we capture SW before sending)
    // Counter counts down, so HW counter will be SMALLER than SW counter
    if (!find_closest_hw_timestamp(counter_before, &counter_hw, &delta_ns, false)) {
        return false;  // No suitable hardware timestamp available
    }

    // Delta should be negative (HW after SW)
    // Convert to positive latency value
    *latency_ns = -delta_ns;

    // Debug logging every 10 calls
    static uint32_t tx_call_count = 0;
    tx_call_count++;
    if (tx_call_count % 10 == 0) {
        printf("[HW_TS_TX] delta_ns=%+lld latency_ns=%+lld valid=%d\n",
               (long long)delta_ns,
               (long long)*latency_ns,
               (*latency_ns >= 1000 && *latency_ns < 1000000) ? 1 : 0);
    }

    // Sanity check: latency should be positive and reasonable (1µs to 1ms)
    // Lowered from 10µs to 1µs because with 150µs delay, we poll very quickly
    if (*latency_ns >= 1000 && *latency_ns < 1000000) {
        return true;
    }

    return false;
}

/**
 * Find hardware timestamp for RX event
 */
bool hw_timestamp_find_rx(uint32_t counter_after, int64_t *latency_ns) {
    uint32_t counter_hw;
    int64_t delta_ns;

    // For RX: expect POSITIVE delta (HW before SW, packet arrives then we detect it)
    // Counter counts down, so HW counter will be LARGER than SW counter
    if (!find_closest_hw_timestamp(counter_after, &counter_hw, &delta_ns, true)) {
        return false;  // No suitable hardware timestamp available
    }

    // Debug logging every 10 calls
    static uint32_t rx_call_count = 0;
    rx_call_count++;
    if (rx_call_count % 10 == 0) {
        uint32_t delta_ticks = (counter_hw > counter_after) ?
                               (counter_hw - counter_after) :
                               (counter_after - counter_hw);
        printf("[HW_TS_RX] BEST: counter_hw=0x%08lX counter_sw=0x%08lX delta_ticks=%lu delta_ns=%+lld\n",
               counter_hw, counter_after, delta_ticks, (long long)delta_ns);
        printf("[HW_TS_RX] Will %s (delta_ns=%+lld, range check: %d)\n",
               (delta_ns >= 10000 && delta_ns < 1000000) ? "ACCEPT" : "REJECT",
               (long long)delta_ns,
               (delta_ns >= 10000 && delta_ns < 1000000));
    }

    // Delta should be positive (HW before SW) - already filtered by find_closest
    // Sanity check: latency should be reasonable (between 10µs and 1ms)
    if (delta_ns >= 10000 && delta_ns < 1000000) {
        *latency_ns = delta_ns;
        return true;
    }

    return false;
}

/**
 * Get debug statistics
 */
void hw_timestamp_get_debug_stats(uint32_t *poll_count_out, uint32_t *fifo_hits_out, uint32_t *buffer_count_out) {
    *poll_count_out = poll_count;
    *fifo_hits_out = fifo_hits;

    // Calculate current buffer occupancy
    if (ts_write_idx >= ts_read_idx) {
        *buffer_count_out = ts_write_idx - ts_read_idx;
    } else {
        *buffer_count_out = TS_BUFFER_SIZE - ts_read_idx + ts_write_idx;
    }
}

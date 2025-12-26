/**
 * PTP Discipline Module - IEEE 1588 Slave Clock Discipline
 *
 * Reuses GPS discipline architecture but synchronized to PTP grandmaster
 * instead of GPS PPS.
 */

#include "ptp_discipline.h"
#include "shared_state_slave.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

// Use same PIO program as GPS discipline
#include "counter_simple.pio.h"
#include "int_timestamp.pio.h"

// PIO configuration
#define DISCIPLINE_PIO pio0
#define COUNTER_SM 0
#define INT_TIMESTAMP_SM 1

// W5500 INT pin (from w5500_simple.h)
#define W5500_PIN_INT 21

// Expected ticks per second at system clock / 3
// At 250 MHz: 250000000 / 3 = 83333333 ticks/sec (12ns resolution)
#define EXPECTED_TICKS_PER_SECOND 83333333ULL

// Debug verbosity control
#define DEBUG_VERBOSE 0  // Set to 1 for detailed debug output

// PI servo controller gains (reduced for stability)
#define SERVO_KP 0.01    // Proportional gain (was 0.1)
#define SERVO_KI 0.0001  // Integral gain (was 0.001)

// Lock thresholds (relaxed to match Ethernet performance)
#define LOCK_THRESHOLD_NS 100000     // 100 microseconds (matches grandmaster)
#define LOCK_SAMPLES_REQUIRED 5
#define UNLOCK_THRESHOLD_NS 1000000  // 1 millisecond

// Step threshold (industry standard approach)
// Applied during initial acquisition to handle boot offset
#define STEP_THRESHOLD_NS 500000     // 500 microseconds
#define STEP_UPDATES_MAX 100         // Allow stepping for first 100 updates

// Asymmetry correction (empirical from crossover cable test 2024-12-24)
// Accounts for total discrepancy between PTP calculated offset and 1PPS measured offset
// Includes both path asymmetry (96µs) and systematic bias (85µs)
// See docs/ASYMMETRY_CORRECTION.md for detailed analysis
#define ASYMMETRY_CORRECTION_NS 181000  // 181 microseconds

// PI Servo Parameters (industry standard values similar to LinuxPTP)
#define PI_KP 0.7                    // Proportional gain (standard)
#define PI_KI 0.0012                 // Integral gain (adjusted for 1Hz updates, 300x smaller)
#define PI_MAX_INTEGRAL 1000000000   // Anti-windup: ±1 second
#define PI_MAX_FREQ_ADJ 100000       // Max frequency adj: ±100µs per update

// Path delay filter
#define PATH_DELAY_ALPHA 0.1         // EMA filter: 10 second time constant

// EMA alpha values for different time windows (α = 1 - exp(-1/N) where N = window in seconds)
#define EMA_ALPHA_5MIN  0.00333      // ~5 minute time constant (300 sec)
#define EMA_ALPHA_15MIN 0.00111      // ~15 minute time constant (900 sec)
#define EMA_ALPHA_30MIN 0.000556     // ~30 minute time constant (1800 sec)

// Running statistics (Welford's algorithm)
typedef struct {
    uint32_t count;
    double mean;
    double M2;  // Sum of squared differences from mean
    double min;
    double max;
} running_stats_t;

// EMA (Exponential Moving Average) for variance tracking
typedef struct {
    double mean;     // EMA of values
    double var;      // EMA of variance
    bool initialized;
} ema_stats_t;

// Discipline state
typedef struct {
    // PIO counter tracking (DISABLED - kept for future use)
    uint32_t prev_counter_value;
    uint64_t prev_t1_master_ns;
    bool first_sync;
    uint32_t syncs_since_boot;     // Count syncs to establish time base

    // Free-running PTP clock (like WiFi slave)
    uint64_t ptp_clock_ns;         // Current PTP time estimate
    uint64_t ptp_clock_update_us;  // Last system time when we updated PTP clock

    // Path delay and offset (IEEE 1588)
    int64_t mean_path_delay_ns;
    int64_t offset_from_master_ns;

    // PI Servo state
    double pi_integral;         // Integral term (accumulated error) in nanoseconds
    uint64_t last_update_us;    // Last discipline update time for dt calculation
    double freq_offset_ppb;     // Derived from PI integral for stats/logging

    // Lock detection
    uint32_t lock_sample_count;
    bool locked;

    // Statistics
    uint32_t sync_count;
    uint32_t discipline_updates;
    uint32_t outliers_rejected;

    // Crystal characterization (derived from Kalman frequency estimate)
    double scale_factor;
    int64_t crystal_error_ns;

    // Running statistics for stability analysis
    running_stats_t offset_stats;
    running_stats_t path_delay_stats;
    running_stats_t scale_factor_stats;
    uint32_t hw_ts_rx_success;
    uint32_t hw_ts_rx_total;
    uint32_t hw_ts_tx_success;
    uint32_t hw_ts_tx_total;

    // EMA statistics for offset (5min, 15min, 30min windows)
    ema_stats_t offset_ema_5min;
    ema_stats_t offset_ema_15min;
    ema_stats_t offset_ema_30min;
} ptp_discipline_state_t;

static ptp_discipline_state_t state = {0};

// Global shared state instances
ptp_sync_data_t ptp_sync_data = {0};
ptp_discipline_stats_t ptp_stats = {0};

// Hardware timestamp buffer for correlation
#define HW_TS_BUFFER_SIZE 16
typedef struct {
    uint32_t counter_value;
    uint64_t system_us;  // When we read it from FIFO
} hw_timestamp_t;

static struct {
    hw_timestamp_t buffer[HW_TS_BUFFER_SIZE];
    uint8_t write_idx;
    uint8_t count;
} hw_ts_buffer = {0};

/**
 * Initialize PTP discipline system
 */
bool ptp_discipline_init(void) {
    printf("Initializing PTP discipline...\n");

    // Load PIO program (simple free-running counter)
    uint counter_offset = pio_add_program(DISCIPLINE_PIO, &counter_simple_program);
    printf("  PIO program loaded (counter @ %d)\n", counter_offset);

    // Initialize counter state machine (SM0)
    counter_simple_program_init(DISCIPLINE_PIO, COUNTER_SM, counter_offset);
    printf("  Counter SM0 initialized (83.33 MHz free-running)\n");

    // Load INT timestamp capture PIO program
    uint int_offset = pio_add_program(DISCIPLINE_PIO, &int_timestamp_program);
    printf("  PIO program loaded (INT timestamp @ %d)\n", int_offset);

    // Initialize INT timestamp state machine (SM1)
    int_timestamp_program_init(DISCIPLINE_PIO, INT_TIMESTAMP_SM, int_offset, W5500_PIN_INT);
    printf("  INT timestamp SM1 initialized (monitoring GPIO%d)\n", W5500_PIN_INT);

    // Initialize state
    state.first_sync = true;
    state.scale_factor = 1.0;           // Fixed at 1.0 (PI servo handles frequency)
    state.mean_path_delay_ns = 0;
    state.offset_from_master_ns = 0;
    state.freq_offset_ppb = 0.0;
    state.pi_integral = 0.0;
    state.last_update_us = 0;
    state.locked = false;
    state.lock_sample_count = 0;

    // Mark as running
    ptp_stats.discipline_running = true;

    printf("PTP discipline ready (12ns resolution counter)\n");
    return true;
}

/**
 * Update free-running PTP clock (PI Servo - Simplified)
 * Call before modifying ptp_clock_ns to bring it up to date
 *
 * No scale_factor needed - PI servo handles frequency via corrections
 */
static void update_ptp_clock(void) {
    if (state.ptp_clock_update_us == 0) {
        // Not initialized yet
        return;
    }

    uint64_t now_us = time_us_64();
    uint64_t elapsed_us = now_us - state.ptp_clock_update_us;

    if (elapsed_us == 0) {
        return;
    }

    // Simple nominal rate: 1µs crystal time = 1µs PTP time
    // PI servo corrects frequency via small adjustments each cycle
    uint64_t ptp_elapsed_ns = elapsed_us * 1000;
    state.ptp_clock_ns += ptp_elapsed_ns;
    state.ptp_clock_update_us = now_us;
}

/**
 * Store hardware timestamp in buffer
 */
static void store_hw_timestamp(uint32_t counter_value, uint64_t system_us) {
    hw_ts_buffer.buffer[hw_ts_buffer.write_idx].counter_value = counter_value;
    hw_ts_buffer.buffer[hw_ts_buffer.write_idx].system_us = system_us;
    hw_ts_buffer.write_idx = (hw_ts_buffer.write_idx + 1) % HW_TS_BUFFER_SIZE;
    if (hw_ts_buffer.count < HW_TS_BUFFER_SIZE) {
        hw_ts_buffer.count++;
    }
}

/**
 * Find most recent hardware timestamp that occurred before software timestamp
 *
 * @param counter_sw Software counter snapshot
 * @param system_us_sw When software captured the timestamp (currently unused)
 * @param counter_hw Output: Hardware counter value if found
 * @param latency_ns Output: Calculated latency in nanoseconds
 * @return true if suitable hardware timestamp found, false otherwise
 */
bool find_hw_timestamp_for_rx(uint32_t counter_sw, uint64_t system_us_sw,
                              uint32_t *counter_hw, int64_t *latency_ns) {
    if (hw_ts_buffer.count == 0) {
        return false;
    }

    // Search backwards through buffer for closest HW timestamp to SW timestamp
    // W5500 INT might fire before OR after we detect/read the frame via polling
    // Find the timestamp with minimum absolute difference

    uint32_t best_counter = 0;
    int64_t best_delta_abs = INT64_MAX;
    int64_t best_delta_signed = 0;
    bool found = false;

    for (int i = 0; i < hw_ts_buffer.count; i++) {
        int idx = (hw_ts_buffer.write_idx - 1 - i + HW_TS_BUFFER_SIZE) % HW_TS_BUFFER_SIZE;
        hw_timestamp_t *ts = &hw_ts_buffer.buffer[idx];

        // Calculate delta (can be positive or negative)
        int64_t delta;
        if (ts->counter_value >= counter_sw) {
            // HW before SW (normal case for RX)
            delta = counter_delta_to_ns(ts->counter_value, counter_sw);
        } else {
            // HW after SW (INT fired after we started reading)
            delta = -counter_delta_to_ns(counter_sw, ts->counter_value);
        }

        int64_t delta_abs = llabs(delta);

        // Sanity check: delta should be reasonable (< 10ms)
        if (delta_abs < 10000000) {
            // Closer than previous best?
            if (delta_abs < best_delta_abs) {
                best_counter = ts->counter_value;
                best_delta_abs = delta_abs;
                best_delta_signed = delta;
                found = true;
            }
        }
    }

    if (found) {
        *counter_hw = best_counter;
        // For RX: negative delta means INT fired after SW read (use absolute value)
        // For now, return absolute value as "latency"
        *latency_ns = best_delta_abs;
        return true;
    }

    return false;
}

/**
 * Read hardware timestamp from INT PIO FIFO
 *
 * SM1 pushes a marker when INT pin falls. We then atomically
 * read SM0's counter to get the precise timestamp.
 *
 * Returns true if timestamp available, false if FIFO empty
 */
bool read_int_hardware_timestamp(uint32_t *counter_value) {
    if (!pio_sm_is_rx_fifo_empty(DISCIPLINE_PIO, INT_TIMESTAMP_SM)) {
        // Discard marker from SM1 FIFO
        (void)pio_sm_get(DISCIPLINE_PIO, INT_TIMESTAMP_SM);

        // Atomically read SM0 counter (same method as Sync handler)
        pio_sm_set_enabled(DISCIPLINE_PIO, COUNTER_SM, false);
        pio_sm_exec(DISCIPLINE_PIO, COUNTER_SM, pio_encode_mov(pio_isr, pio_x));
        pio_sm_exec(DISCIPLINE_PIO, COUNTER_SM, pio_encode_push(false, false));
        *counter_value = pio_sm_get(DISCIPLINE_PIO, COUNTER_SM);
        pio_sm_set_enabled(DISCIPLINE_PIO, COUNTER_SM, true);

        // Store in buffer with current time for correlation
        store_hw_timestamp(*counter_value, time_us_64());

        return true;
    }
    return false;
}

/**
 * Convert PIO counter value to nanoseconds
 * Counter runs at 83.33 MHz (12ns per tick, counting DOWN from 0xFFFFFFFF)
 */
uint64_t counter_to_ns(uint32_t counter_value) {
    // Counter counts down, so invert to get elapsed time
    uint32_t elapsed_ticks = 0xFFFFFFFF - counter_value;
    return (uint64_t)elapsed_ticks * 12ULL;  // 12ns per tick
}

/**
 * Convert PIO counter delta to nanoseconds with crystal correction
 *
 * Counter counts DOWN, so delta = earlier_counter - later_counter
 * Applies crystal characterization for accuracy
 */
int64_t counter_delta_to_ns(uint32_t earlier_counter, uint32_t later_counter) {
    // Counter counts down, so earlier value is larger
    int64_t delta_ticks;

    if (earlier_counter >= later_counter) {
        // Normal case: no wraparound
        delta_ticks = (int64_t)(earlier_counter - later_counter);
    } else {
        // Wraparound case (very unlikely in short windows, but handle it)
        delta_ticks = (int64_t)(earlier_counter + (0xFFFFFFFFUL - later_counter) + 1);
    }

    // Base conversion: 12ns per tick at perfect 83.33 MHz
    double delta_ns_raw = (double)delta_ticks * 12.0;

    // Apply crystal characterization if available and stable
    if (state.scale_factor != 0.0 && state.discipline_updates > 10) {
        delta_ns_raw *= state.scale_factor;
    }

    return (int64_t)delta_ns_raw;
}

/**
 * Get count of outliers rejected
 */
uint32_t ptp_discipline_get_outliers_rejected(void) {
    return state.outliers_rejected;
}

/**
 * Get current scale factor
 * Derived from PI integral (frequency offset in ppb)
 * Used by 1PPS scheduler to compensate for crystal frequency error
 */
double ptp_discipline_get_scale_factor(void) {
    // Convert frequency offset (ppb) to scale factor for scheduler
    // freq_offset_ppb > 0 means slave clock runs fast (ahead)
    // Scheduler divides by scale_factor, so:
    // - Crystal fast: scale_factor < 1.0 → more ticks after division
    // - Crystal slow: scale_factor > 1.0 → fewer ticks after division
    // scale_factor = 1.0 - (freq_offset / 1e9)
    // Example: +34000 ppb (fast) → 0.999966 → ticks/0.999966 = more ticks ✓
    return 1.0 - (state.freq_offset_ppb / 1000000000.0);
}

/**
 * Get standard deviation from running stats (helper function)
 */
static double get_stddev(const running_stats_t *stats) {
    if (stats->count < 2) return 0.0;
    return sqrt(stats->M2 / stats->count);
}

/**
 * Get running statistics for stability analysis
 */
void ptp_discipline_get_stats(double *offset_mean_us, double *offset_stddev_us,
                              double *offset_min_us, double *offset_max_us,
                              double *pd_mean_us, double *pd_stddev_us,
                              double *sf_stddev_ppm,
                              uint32_t *hw_rx_pct, uint32_t *hw_tx_pct,
                              double *offset_ema_5m, double *offset_ema_15m, double *offset_ema_30m) {
    if (offset_mean_us) *offset_mean_us = state.offset_stats.mean;
    if (offset_stddev_us) *offset_stddev_us = get_stddev(&state.offset_stats);
    if (offset_min_us) *offset_min_us = state.offset_stats.min;
    if (offset_max_us) *offset_max_us = state.offset_stats.max;
    if (pd_mean_us) *pd_mean_us = state.path_delay_stats.mean;
    if (pd_stddev_us) *pd_stddev_us = get_stddev(&state.path_delay_stats);
    if (sf_stddev_ppm) *sf_stddev_ppm = get_stddev(&state.scale_factor_stats);
    if (hw_rx_pct) *hw_rx_pct = state.hw_ts_rx_total > 0 ?
        (state.hw_ts_rx_success * 100) / state.hw_ts_rx_total : 0;
    if (hw_tx_pct) *hw_tx_pct = state.hw_ts_tx_total > 0 ?
        (state.hw_ts_tx_success * 100) / state.hw_ts_tx_total : 0;

    // EMA standard deviations for time windows
    if (offset_ema_5m) *offset_ema_5m = sqrt(state.offset_ema_5min.var);
    if (offset_ema_15m) *offset_ema_15m = sqrt(state.offset_ema_15min.var);
    if (offset_ema_30m) *offset_ema_30m = sqrt(state.offset_ema_30min.var);
}

/**
 * Get current PTP time (PI Servo - Simplified)
 *
 * Returns absolute PTP time by interpolating between Sync boundaries.
 * PI servo corrects frequency via discipline loop, not via scale_factor.
 *
 * CRITICAL: Placed in SRAM for deterministic execution (no flash cache misses)
 */
uint64_t __time_critical_func(get_ptp_time_ns)(void) {
    if (state.ptp_clock_update_us == 0) {
        return 0;  // Not initialized yet
    }

    // How far since last Sync? (in system timer microseconds)
    uint64_t now_us = time_us_64();
    uint64_t elapsed_us = now_us - state.ptp_clock_update_us;

    // Simple nominal rate: 1µs crystal time = 1µs PTP time
    uint64_t ptp_elapsed_ns = elapsed_us * 1000;

    // PTP time = last Sync boundary + elapsed time
    return state.ptp_clock_ns + ptp_elapsed_ns;
}

/**
 * PI Servo Controller
 *
 * Classic Proportional-Integral controller for frequency control.
 * Industry-standard approach used by LinuxPTP, PTPd, and other implementations.
 *
 * Controller equation:
 *   u(t) = Kp·e(t) + Ki·∫e(τ)dτ
 *
 * where:
 *   e(t) = offset from master (error signal)
 *   u(t) = control output (frequency adjustment in nanoseconds)
 *
 * @param offset_ns Current offset from master in nanoseconds (positive = slave ahead)
 * @param dt_sec Time since last update in seconds
 * @return Frequency adjustment to apply (nanoseconds to subtract from clock)
 */
static int64_t pi_servo_update(double offset_ns, double dt_sec) {
    // Proportional term: immediate response to current error
    double p_term = PI_KP * offset_ns;

    // Integral term: accumulate error over time
    // This acts as frequency offset estimator
    state.pi_integral += PI_KI * offset_ns * dt_sec;

    // Anti-windup: prevent integral from growing unbounded
    if (state.pi_integral > PI_MAX_INTEGRAL) {
        state.pi_integral = PI_MAX_INTEGRAL;
    } else if (state.pi_integral < -PI_MAX_INTEGRAL) {
        state.pi_integral = -PI_MAX_INTEGRAL;
    }

    // Total control output (frequency adjustment)
    double freq_adj = p_term + state.pi_integral;

    // Limit maximum adjustment per cycle (safety/stability)
    if (freq_adj > PI_MAX_FREQ_ADJ) {
        freq_adj = PI_MAX_FREQ_ADJ;
    } else if (freq_adj < -PI_MAX_FREQ_ADJ) {
        freq_adj = -PI_MAX_FREQ_ADJ;
    }

    // Derive frequency offset in ppb for statistics
    // integral term represents steady-state frequency correction in ns per cycle
    // Since PTP cycles are ~1 second, this directly equals frequency offset in ppb
    // (1 ns per second = 1 ppb = 1 ns/1e9 s = 1e-9 fractional frequency)
    state.freq_offset_ppb = state.pi_integral;

    return (int64_t)freq_adj;
}

/**
 * Update running statistics (Welford's algorithm for numerically stable variance)
 */
static void update_stats(running_stats_t *stats, double value) {
    stats->count++;

    // Update min/max
    if (stats->count == 1) {
        stats->min = value;
        stats->max = value;
    } else {
        if (value < stats->min) stats->min = value;
        if (value > stats->max) stats->max = value;
    }

    // Welford's online algorithm for mean and variance
    double delta = value - stats->mean;
    stats->mean += delta / stats->count;
    double delta2 = value - stats->mean;
    stats->M2 += delta * delta2;
}

/**
 * Update EMA statistics for variance tracking
 * Uses exponentially weighted variance calculation
 */
static void update_ema_stats(ema_stats_t *ema, double value, double alpha) {
    if (!ema->initialized) {
        ema->mean = value;
        ema->var = 0.0;
        ema->initialized = true;
    } else {
        // Update mean: mean_new = α × value + (1-α) × mean_old
        double delta = value - ema->mean;
        ema->mean += alpha * delta;

        // Update variance: var_new = (1-α) × (var_old + α × delta²)
        ema->var = (1.0 - alpha) * (ema->var + alpha * delta * delta);
    }
}

/**
 * Update discipline (IEEE 1588 algorithm with Kalman filter)
 */
void ptp_discipline_update(void) {
    // Update free-running PTP clock
    update_ptp_clock();

    // Need both Sync/Follow_Up and Delay_Req/Resp for complete cycle
    if (!ptp_sync_data.sync_followup_ready) {
        return;
    }

    state.syncs_since_boot++;

    // Handle first Sync - initialize free-running PTP clock
    if (state.first_sync) {
        // Initialize PTP clock to master time + nominal path delay
        state.ptp_clock_ns = ptp_sync_data.t1_master_ns + 50000;  // Assume 50µs path delay
        state.ptp_clock_update_us = ptp_sync_data.t2_slave_us;
        state.first_sync = false;

        // Initialize PI servo
        state.pi_integral = 0.0;
        state.last_update_us = ptp_sync_data.t2_slave_us;
        state.freq_offset_ppb = 0.0;

        printf("First Sync received (PI servo initialized)\n");

        ptp_sync_data.sync_followup_ready = false;
        ptp_sync_data.delay_resp_ready = false;
        return;
    }

    // Discard early Delay_Resp until we have stable timing
    if (state.syncs_since_boot < 3) {
        ptp_sync_data.sync_followup_ready = false;
        ptp_sync_data.delay_resp_ready = false;
        return;
    }

    // Now require Delay_Resp to complete the cycle
    if (!ptp_sync_data.delay_resp_ready) {
        return;
    }

    // ========================================================================
    // IEEE 1588-2008 COMPLIANT ALGORITHM
    // ========================================================================

    // Use timestamps directly from our free-running disciplined PTP clock
    // No time domain conversion needed - t2 and t3 are already in PTP time domain!
    uint64_t t2_ptp_ns = ptp_sync_data.t2_ptp_ns;
    uint64_t t3_ptp_ns = ptp_sync_data.t3_ptp_ns;

    // CRITICAL: Ensure t3 (Delay_Req) came AFTER t2 (Sync)
    // If t3 < t2, this Delay_Resp is from a previous cycle - discard it
    if (t3_ptp_ns < t2_ptp_ns) {
#if DEBUG_VERBOSE
        printf("Discarding Delay_Resp: t3 (%llu) before t2 (%llu)\n", t3_ptp_ns, t2_ptp_ns);
#endif
        ptp_sync_data.delay_resp_ready = false;
        return;
    }

    // 1. Calculate Mean Path Delay (IEEE 1588-2008 Eq. 3)
    // path_delay = ((t2 - t1) - correctionSync + (t4 - t3) - correctionDelayResp) / 2
    int64_t term1 = (int64_t)(t2_ptp_ns - ptp_sync_data.t1_master_ns) - ptp_sync_data.correction_sync_ns;
    int64_t term2 = (int64_t)(ptp_sync_data.t4_master_ns - t3_ptp_ns) - ptp_sync_data.correction_delay_resp_ns;
    int64_t path_delay_raw = (term1 + term2) / 2;

    // Verbose debug output (first few cycles only)
#if DEBUG_VERBOSE
    if (state.discipline_updates < 5) {
        printf("PATH DEBUG: term1(t2-t1)=%+lld term2(t4-t3)=%+lld path_delay=%+lld\n",
               term1, term2, path_delay_raw);
        printf("            t2_ptp=%llu t1_master=%llu\n", t2_ptp_ns, ptp_sync_data.t1_master_ns);
        printf("            t4_master=%llu t3_ptp=%llu\n", ptp_sync_data.t4_master_ns, t3_ptp_ns);
        const char *rx_type = ptp_sync_data.rx_hw_timestamp_valid ? "HW" :
                              (ptp_sync_data.rx_used_average ? "AVG" : "SW");
        const char *tx_type = ptp_sync_data.tx_hw_timestamp_valid ? "HW" : "SW";
        printf("HW LATENCY: RX=%+lldns (%s) TX=%+lldns (%s)\n",
               (long long)ptp_sync_data.rx_latency_ns,
               rx_type,
               (long long)ptp_sync_data.tx_latency_ns,
               tx_type);
    }
#endif

    // Compact stats output every cycle
    if (state.discipline_updates % 10 == 0 || state.discipline_updates < 10) {
        const char *rx_ts_type = ptp_sync_data.rx_hw_timestamp_valid ? "HW" :
                                 (ptp_sync_data.rx_used_average ? "AVG" : "SW");
        const char *tx_ts_type = ptp_sync_data.tx_hw_timestamp_valid ? "HW" : "SW";
        printf("D#%03lu: t1=%+6lld t2=%+6lld pd=%+6lld | RX=%3s/%6lld TX=%3s/%6lld | off=%+7lld lock=%c | scale=%.6f\n",
               state.discipline_updates,
               term1 / 1000,  // Convert to microseconds
               term2 / 1000,
               path_delay_raw / 1000,
               rx_ts_type,
               (long long)(ptp_sync_data.rx_latency_ns / 1000),
               tx_ts_type,
               (long long)(ptp_sync_data.tx_latency_ns / 1000),
               (long long)(state.offset_from_master_ns),
               state.locked ? 'Y' : 'N',
               state.scale_factor);

        // PHASE 5: Detailed offset calculation breakdown
        if (state.discipline_updates % 100 == 0) {
            printf("\n=== OFFSET DIAGNOSTIC (Update #%lu) ===\n", state.discipline_updates);
            printf("T1 (GM TX):    %llu ns\n", ptp_sync_data.t1_master_ns);
            printf("T2 (Slave RX): %llu ns (corrected by %s %+lld ns)\n",
                   t2_ptp_ns, rx_ts_type, -(long long)ptp_sync_data.rx_latency_ns);
            printf("T3 (Slave TX): %llu ns (corrected by %s %+lld ns)\n",
                   t3_ptp_ns, tx_ts_type, (long long)ptp_sync_data.tx_latency_ns);
            printf("T4 (GM RX):    %llu ns\n", ptp_sync_data.t4_master_ns);
            printf("\nForward path:  (t2-t1) = %llu - %llu = %+lld ns\n",
                   t2_ptp_ns, ptp_sync_data.t1_master_ns,
                   (int64_t)(t2_ptp_ns - ptp_sync_data.t1_master_ns));
            printf("  - correction_sync = %+lld ns\n", ptp_sync_data.correction_sync_ns);
            printf("  = term1 = %+lld ns\n", term1);
            printf("\nReverse path:  (t4-t3) = %llu - %llu = %+lld ns\n",
                   ptp_sync_data.t4_master_ns, t3_ptp_ns,
                   (int64_t)(ptp_sync_data.t4_master_ns - t3_ptp_ns));
            printf("  - correction_delay_resp = %+lld ns\n", ptp_sync_data.correction_delay_resp_ns);
            printf("  = term2 = %+lld ns\n", term2);
            printf("\nPath delay:    (term1 + term2) / 2 = (%+lld + %+lld) / 2 = %+lld ns\n",
                   term1, term2, path_delay_raw);
            printf("Mean path delay (filtered): %+lld ns\n", state.mean_path_delay_ns);
            printf("\nOffset calculation:\n");
            printf("  (t2 - t1) - mean_path_delay - correction_sync\n");
            printf("  = %+lld - %+lld - %+lld\n",
                   (int64_t)(t2_ptp_ns - ptp_sync_data.t1_master_ns),
                   state.mean_path_delay_ns,
                   ptp_sync_data.correction_sync_ns);
            printf("  = %+lld ns (%.1f µs)\n",
                   (long long)state.offset_from_master_ns,
                   (double)state.offset_from_master_ns / 1000.0);
            printf("\nExpected asymmetry from latencies:\n");
            printf("  Slave RX latency: %+lld ns\n", ptp_sync_data.rx_latency_ns);
            printf("  Slave TX latency: %+lld ns\n", ptp_sync_data.tx_latency_ns);
            printf("  Net effect on offset: ~%+lld ns\n",
                   -(ptp_sync_data.rx_latency_ns - ptp_sync_data.tx_latency_ns) / 2);
            printf("=====================================\n\n");
        }
    }

    // Filter path delay with exponential moving average
    // Use PATH_DELAY_ALPHA (α = 0.1) for 10 second time constant
    if (state.mean_path_delay_ns == 0) {
        state.mean_path_delay_ns = path_delay_raw;
    } else {
        state.mean_path_delay_ns = (int64_t)((double)state.mean_path_delay_ns * (1.0 - PATH_DELAY_ALPHA) +
                                              (double)path_delay_raw * PATH_DELAY_ALPHA);
    }

    // 2. Calculate Offset from Master (IEEE 1588-2008 Eq. 4)
    // offset = (t2 - t1) - mean_path_delay - correctionSync
    // This tells us how much our slave PTP clock is ahead of master clock
    // ASYMMETRY CORRECTION: Add empirical constant to align with 1PPS measurements
    double measured_offset =
        (double)((int64_t)(t2_ptp_ns - ptp_sync_data.t1_master_ns) -
        state.mean_path_delay_ns -
        ptp_sync_data.correction_sync_ns +
        ASYMMETRY_CORRECTION_NS);

    // 3. Crystal Characterization (PIO counter)
    uint32_t elapsed_ticks;
    if (ptp_sync_data.counter_at_sync > state.prev_counter_value) {
        // Counter wrapped around (counts down)
        elapsed_ticks = state.prev_counter_value + (0xFFFFFFFF - ptp_sync_data.counter_at_sync) + 1;
    } else {
        elapsed_ticks = state.prev_counter_value - ptp_sync_data.counter_at_sync;
    }

    uint64_t master_delta_ns = ptp_sync_data.t1_master_ns - state.prev_t1_master_ns;
    uint64_t expected_ticks = (master_delta_ns * EXPECTED_TICKS_PER_SECOND) / 1000000000ULL;

#if DEBUG_VERBOSE
    // Debug output for first few cycles
    if (state.discipline_updates < 5) {
        printf("DEBUG: master_delta_ns=%llu expected_ticks=%llu elapsed_ticks=%lu\n",
               master_delta_ns, expected_ticks, elapsed_ticks);
        printf("       prev_counter=0x%08lX current_counter=0x%08lX\n",
               state.prev_counter_value, ptp_sync_data.counter_at_sync);
    }
#endif

    int64_t crystal_error_ticks = (int64_t)elapsed_ticks - (int64_t)expected_ticks;
    state.crystal_error_ns = crystal_error_ticks * 12; // 12ns per tick

    // 4. PI Servo - Industry Standard Control Algorithm
    // Calculate time delta for PI servo
    uint64_t current_time_us = ptp_sync_data.t2_slave_us;
    double dt_sec = (double)(current_time_us - state.last_update_us) / 1000000.0;

    // Run PI servo to get frequency adjustment
    int64_t freq_adj_ns = pi_servo_update(measured_offset, dt_sec);

    // Update timestamp for next iteration
    state.last_update_us = current_time_us;

    // Update state variables for logging/stats
    state.offset_from_master_ns = (int64_t)measured_offset;
    // freq_offset_ppb already updated by pi_servo_update()

    // 5. Update Free-Running PTP Clock with Step Threshold (Industry Standard)

    // Always bring clock up to date first
    update_ptp_clock();

    // Step threshold: During initial acquisition (first STEP_UPDATES_MAX updates)
    bool allow_step = (state.discipline_updates < STEP_UPDATES_MAX);

    // Check if offset exceeds step threshold AND stepping is allowed
    if (allow_step && llabs(state.offset_from_master_ns) > STEP_THRESHOLD_NS) {
        // STEP: Large offset - jump clock directly (industry standard approach)
        // Positive offset means we're ahead, so subtract to bring us back
        state.ptp_clock_ns -= state.offset_from_master_ns;

        printf("STEP: Jumped clock by %+.1f µs (offset %+.1f µs exceeded threshold %.1f µs)\n",
               -(double)state.offset_from_master_ns / 1000.0,  // Negative because we subtract
               (double)state.offset_from_master_ns / 1000.0,
               STEP_THRESHOLD_NS / 1000.0);

        // Reset PI servo integral after step (clear accumulated error)
        state.pi_integral = 0.0;
        state.freq_offset_ppb = 0.0;
        printf("STEP: Reset PI integral to 0\n");

        // Correlation-friendly log: show step occurred
        // Format: DISC|timestamp_us|seq|offset_ns|correction_ns|pi_integral|freq_ppb|hw_rx|hw_tx
        printf("DISC|%llu|%lu|%+lld|%+lld|%+.1f|%+.1f|%d|%d|STEP\n",
               time_us_64(),
               state.discipline_updates,
               (long long)state.offset_from_master_ns,
               -(long long)state.offset_from_master_ns,  // Show correction as negative (we subtract)
               state.pi_integral,
               state.freq_offset_ppb,
               ptp_sync_data.rx_hw_timestamp_valid ? 1 : 0,
               ptp_sync_data.tx_hw_timestamp_valid ? 1 : 0);

    } else {
        // SLEW: Small offset - use PI servo frequency adjustment

        // Apply PI servo output as correction
        int64_t correction = freq_adj_ns;

        state.ptp_clock_ns -= correction;  // Subtract to bring us closer to master

        // Correlation-friendly log: parseable format for alignment with measurement data
        // Format: DISC|timestamp_us|seq|offset_ns|correction_ns|pi_integral|freq_ppb|hw_rx|hw_tx
        printf("DISC|%llu|%lu|%+lld|%+lld|%+.1f|%+.1f|%d|%d\n",
               time_us_64(),                              // Monotonic timestamp (µs since boot)
               state.discipline_updates,                  // Sequence number
               (long long)state.offset_from_master_ns,    // Calculated offset
               (long long)correction,                     // Correction applied (from PI servo)
               state.pi_integral,                         // PI integral term (ns)
               state.freq_offset_ppb,                     // Frequency offset (ppb)
               ptp_sync_data.rx_hw_timestamp_valid ? 1 : 0,  // RX HW timestamp
               ptp_sync_data.tx_hw_timestamp_valid ? 1 : 0); // TX HW timestamp
    }

    // 6. Lock Detection
    bool was_locked = state.locked;
    if (llabs(state.offset_from_master_ns) < LOCK_THRESHOLD_NS) {
        state.lock_sample_count++;
        if (state.lock_sample_count >= LOCK_SAMPLES_REQUIRED) {
            state.locked = true;
        }
    } else if (llabs(state.offset_from_master_ns) > UNLOCK_THRESHOLD_NS) {
        state.lock_sample_count = 0;
        state.locked = false;
    }

    // Update counters
    state.sync_count++;
    state.discipline_updates++;

    // Track hardware timestamp success rates
    state.hw_ts_rx_total++;
    state.hw_ts_tx_total++;
    if (ptp_sync_data.rx_hw_timestamp_valid) state.hw_ts_rx_success++;
    if (ptp_sync_data.tx_hw_timestamp_valid) state.hw_ts_tx_success++;

    // Update running statistics
    update_stats(&state.offset_stats, (double)state.offset_from_master_ns / 1000.0);  // Convert to µs
    update_stats(&state.path_delay_stats, (double)state.mean_path_delay_ns / 1000.0); // Convert to µs
    update_stats(&state.scale_factor_stats, state.freq_offset_ppb);  // Store frequency offset (ppb)

    // Update EMA statistics for offset (5min, 15min, 30min windows)
    double offset_us = (double)state.offset_from_master_ns / 1000.0;
    update_ema_stats(&state.offset_ema_5min, offset_us, EMA_ALPHA_5MIN);
    update_ema_stats(&state.offset_ema_15min, offset_us, EMA_ALPHA_15MIN);
    update_ema_stats(&state.offset_ema_30min, offset_us, EMA_ALPHA_30MIN);

    // Update shared statistics
    ptp_stats.locked = state.locked;
    ptp_stats.offset_from_master_ns = state.offset_from_master_ns;
    ptp_stats.mean_path_delay_ns = state.mean_path_delay_ns;
    ptp_stats.freq_offset_ppb = state.freq_offset_ppb;
    ptp_stats.scale_factor = 1.0;  // No longer used (PI servo controls frequency directly)
    ptp_stats.crystal_error_ns = state.crystal_error_ns;
    ptp_stats.crystal_ppm = (double)state.crystal_error_ns / 1000000.0;
    ptp_stats.sync_count = state.sync_count;
    ptp_stats.discipline_updates = state.discipline_updates;
    ptp_stats.ptp_time_ns = get_ptp_time_ns();
    ptp_stats.last_update_us = time_us_64();

    // Update previous values for next cycle
    state.prev_counter_value = ptp_sync_data.counter_at_sync;
    state.prev_t1_master_ns = ptp_sync_data.t1_master_ns;

    // Clear flags
    ptp_sync_data.sync_followup_ready = false;
    ptp_sync_data.delay_resp_ready = false;
}

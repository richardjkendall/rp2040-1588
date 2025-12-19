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

// Kalman filter noise parameters
// PHASE 1 IMPROVEMENT: Fixed measurement noise to match actual Ethernet jitter
#define KALMAN_Q_OFFSET 1e6          // Process noise: offset variance (1 µs std dev)
#define KALMAN_Q_FREQ 1e-4           // Process noise: frequency variance (0.0001 ppb std dev)
#define KALMAN_R_MEASUREMENT_HW 1e8  // Measurement noise with HW timestamps (10 µs std dev)
#define KALMAN_R_MEASUREMENT_SW 1e12 // Measurement noise with SW timestamps (1 ms std dev)
#define KALMAN_ALPHA_LPF 0.1         // Low-pass filter for path delay (10 sec time constant)
#define KALMAN_OUTLIER_THRESHOLD 1000000  // Reject measurements > 1ms from prediction (3σ)

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

    // Servo state (legacy PI servo - being replaced by Kalman)
    double freq_offset_ppb;
    int64_t integral_term;

    // Kalman filter state (2-state: offset, frequency)
    double kalman_x[2];      // State: [offset_ns, freq_offset_ppb]
    double kalman_P[2][2];   // Covariance matrix
    bool kalman_initialized;

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
    state.scale_factor = 1.0;
    state.mean_path_delay_ns = 0;
    state.offset_from_master_ns = 0;
    state.freq_offset_ppb = 0.0;
    state.integral_term = 0;
    state.locked = false;
    state.lock_sample_count = 0;

    // Mark as running
    ptp_stats.discipline_running = true;

    printf("PTP discipline ready (12ns resolution counter)\n");
    return true;
}

/**
 * Update free-running PTP clock using scale factor (GPS-style)
 * Call before modifying ptp_clock_ns to bring it up to date
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

    // Scale to PTP nanoseconds using characterized crystal scale factor
    double scale = state.scale_factor;
    if (scale == 0.0 || state.discipline_updates < 3) {
        scale = 1.0;  // Use nominal rate until characterized
    }

    uint64_t ptp_elapsed_ns = (uint64_t)((double)elapsed_us * 1000.0 * scale);
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
 */
double ptp_discipline_get_scale_factor(void) {
    return state.scale_factor;
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
 * Get current PTP time (GPS-style interpolation with scale factor)
 *
 * Returns absolute PTP time by interpolating between Sync boundaries using
 * characterized crystal scale factor (like GPS grandmaster approach).
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

    // Scale to PTP nanoseconds using characterized crystal scale factor
    // System timer runs at same crystal rate as PIO counter
    // scale_factor = expected_ticks / measured_ticks = how to convert crystal time to PTP time
    double scale = state.scale_factor;
    if (scale == 0.0 || state.discipline_updates < 3) {
        scale = 1.0;  // Use nominal rate until characterized
    }

    uint64_t ptp_elapsed_ns = (uint64_t)((double)elapsed_us * 1000.0 * scale);

    // PTP time = last Sync boundary + scaled elapsed time
    return state.ptp_clock_ns + ptp_elapsed_ns;
}

/**
 * Initialize Kalman filter
 */
static void kalman_init(void) {
    // Initial state: zero offset, zero frequency offset
    state.kalman_x[0] = 0.0;  // offset_ns
    state.kalman_x[1] = 0.0;  // freq_offset_ppb

    // Initial covariance: high uncertainty
    state.kalman_P[0][0] = 1e12;  // High uncertainty in offset
    state.kalman_P[0][1] = 0.0;
    state.kalman_P[1][0] = 0.0;
    state.kalman_P[1][1] = 1e6;   // High uncertainty in frequency

    state.kalman_initialized = true;
}

/**
 * Kalman filter predict step
 *
 * State transition:
 *   offset(k+1) = offset(k) + freq_offset(k) * dt
 *   freq_offset(k+1) = freq_offset(k)
 *
 * @param dt Time step in seconds (typically 1.0 for PTP)
 */
static void kalman_predict(double dt) {
    // State prediction: x = F * x
    // F = [1  dt]
    //     [0   1]
    double x_pred[2];
    x_pred[0] = state.kalman_x[0] + state.kalman_x[1] * dt;
    x_pred[1] = state.kalman_x[1];

    // Covariance prediction: P = F * P * F' + Q
    double P_pred[2][2];

    // F * P
    double FP[2][2];
    FP[0][0] = state.kalman_P[0][0] + dt * state.kalman_P[1][0];
    FP[0][1] = state.kalman_P[0][1] + dt * state.kalman_P[1][1];
    FP[1][0] = state.kalman_P[1][0];
    FP[1][1] = state.kalman_P[1][1];

    // F * P * F'
    P_pred[0][0] = FP[0][0] + dt * FP[0][1];
    P_pred[0][1] = FP[0][1];
    P_pred[1][0] = FP[1][0] + dt * FP[1][1];
    P_pred[1][1] = FP[1][1];

    // Add process noise Q
    P_pred[0][0] += KALMAN_Q_OFFSET;
    P_pred[1][1] += KALMAN_Q_FREQ;

    // Update state
    state.kalman_x[0] = x_pred[0];
    state.kalman_x[1] = x_pred[1];
    state.kalman_P[0][0] = P_pred[0][0];
    state.kalman_P[0][1] = P_pred[0][1];
    state.kalman_P[1][0] = P_pred[1][0];
    state.kalman_P[1][1] = P_pred[1][1];
}

/**
 * Kalman filter update step
 *
 * Measurement model: z = offset + noise
 * H = [1  0]  (we measure offset directly)
 *
 * @param measured_offset Measured offset in nanoseconds
 * @param measurement_noise_r Measurement noise variance (depends on timestamp quality)
 */
static void kalman_update(double measured_offset, double measurement_noise_r) {
    // Measurement residual: y = z - H * x
    double y = measured_offset - state.kalman_x[0];

    // Residual covariance: S = H * P * H' + R
    // Since H = [1 0], this simplifies to:
    double S = state.kalman_P[0][0] + measurement_noise_r;

    // Kalman gain: K = P * H' * inv(S)
    // K is a 2x1 vector
    double K[2];
    K[0] = state.kalman_P[0][0] / S;
    K[1] = state.kalman_P[1][0] / S;

    // State update: x = x + K * y
    state.kalman_x[0] += K[0] * y;
    state.kalman_x[1] += K[1] * y;

    // Covariance update: P = (I - K * H) * P
    // Since H = [1 0]:
    double P_new[2][2];
    P_new[0][0] = (1.0 - K[0]) * state.kalman_P[0][0];
    P_new[0][1] = (1.0 - K[0]) * state.kalman_P[0][1];
    P_new[1][0] = state.kalman_P[1][0] - K[1] * state.kalman_P[0][0];
    P_new[1][1] = state.kalman_P[1][1] - K[1] * state.kalman_P[0][1];

    state.kalman_P[0][0] = P_new[0][0];
    state.kalman_P[0][1] = P_new[0][1];
    state.kalman_P[1][0] = P_new[1][0];
    state.kalman_P[1][1] = P_new[1][1];
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

        // Save for crystal characterization (disabled)
        state.prev_counter_value = ptp_sync_data.counter_at_sync;
        state.prev_t1_master_ns = ptp_sync_data.t1_master_ns;
        state.scale_factor = 1.0;

        // Initialize Kalman filter
        kalman_init();

        printf("First Sync received (Kalman filter initialized)\n");

        ptp_sync_data.sync_followup_ready = false;
        ptp_sync_data.delay_resp_ready = false;
        return;
    }

    // Discard early Delay_Resp until we have stable timing
    if (state.syncs_since_boot < 3) {
        // Update crystal characterization (disabled - just save state)
        state.prev_counter_value = ptp_sync_data.counter_at_sync;
        state.prev_t1_master_ns = ptp_sync_data.t1_master_ns;

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
        printf("HW LATENCY: RX=%+lldns (%s) TX=%+lldns (%s)\n",
               (long long)ptp_sync_data.rx_latency_ns,
               ptp_sync_data.rx_hw_timestamp_valid ? "HW" : "SW",
               (long long)ptp_sync_data.tx_latency_ns,
               ptp_sync_data.tx_hw_timestamp_valid ? "HW" : "SW");
    }
#endif

    // Compact stats output every cycle
    if (state.discipline_updates % 10 == 0 || state.discipline_updates < 10) {
        printf("D#%03lu: t1=%+6lld t2=%+6lld pd=%+6lld | RX=%3s/%6lld TX=%3s/%6lld | off=%+7lld lock=%c | scale=%.6f\n",
               state.discipline_updates,
               term1 / 1000,  // Convert to microseconds
               term2 / 1000,
               path_delay_raw / 1000,
               ptp_sync_data.rx_hw_timestamp_valid ? "HW" : "SW",
               (long long)(ptp_sync_data.rx_latency_ns / 1000),
               ptp_sync_data.tx_hw_timestamp_valid ? "HW" : "SW",
               (long long)(ptp_sync_data.tx_latency_ns / 1000),
               (long long)(state.offset_from_master_ns),
               state.locked ? 'Y' : 'N',
               state.scale_factor);
    }

    // Filter path delay with exponential moving average
    // Use KALMAN_ALPHA_LPF (α = 0.1) for 10 second time constant
    if (state.mean_path_delay_ns == 0) {
        state.mean_path_delay_ns = path_delay_raw;
    } else {
        state.mean_path_delay_ns = (int64_t)((double)state.mean_path_delay_ns * (1.0 - KALMAN_ALPHA_LPF) +
                                              (double)path_delay_raw * KALMAN_ALPHA_LPF);
    }

    // 2. Calculate Offset from Master (IEEE 1588-2008 Eq. 4)
    // offset = (t2 - t1) - mean_path_delay - correctionSync
    // This tells us how much our slave PTP clock is ahead of master clock
    double measured_offset =
        (double)((int64_t)(t2_ptp_ns - ptp_sync_data.t1_master_ns) -
        state.mean_path_delay_ns -
        ptp_sync_data.correction_sync_ns);

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

    // Update scale factor for interpolation with HEAVY filtering
    // Trust local crystal for short periods, only correct long-term drift
    // Time constant: ~50 seconds (alpha = 0.02)
    if (elapsed_ticks > 0) {
        double new_scale = (double)expected_ticks / (double)elapsed_ticks;

        if (state.scale_factor == 0.0 || state.discipline_updates < 3) {
            // Bootstrap: Use measured value directly
            state.scale_factor = new_scale;
        } else {
            // Low-pass filter: 98% old, 2% new (50 second time constant)
            // This filters out network jitter, tracks only crystal drift
            state.scale_factor = state.scale_factor * 0.98 + new_scale * 0.02;
        }
    }

    // 4. Kalman Filter - Optimal state estimation
    // Predict step: Estimate where we should be based on previous state
    double dt = (double)master_delta_ns / 1e9;  // Time since last sync in seconds
    kalman_predict(dt);

    // Outlier rejection: Check if measurement is reasonable
    // After predict, kalman_x[0] contains our best prediction
    double predicted_offset = state.kalman_x[0];
    double innovation = measured_offset - predicted_offset;  // How far off is measurement?
    bool is_outlier = false;

    // Adaptive outlier threshold: larger during initial convergence
    // This allows system to converge from large initial offsets (e.g., 10ms at startup)
    // without rejecting legitimate measurements during convergence
    double outlier_threshold;
    if (state.discipline_updates < 30) {
        // Initial convergence: 10ms threshold (handles large startup offsets)
        outlier_threshold = 10000000;
    } else if (state.discipline_updates < 100) {
        // Medium convergence: 3ms threshold
        outlier_threshold = 3000000;
    } else {
        // Steady state: 1ms threshold (original value)
        outlier_threshold = KALMAN_OUTLIER_THRESHOLD;
    }

    if (state.kalman_initialized && state.discipline_updates > 5) {
        // Only reject outliers after initial Kalman startup
        if (fabs(innovation) > outlier_threshold) {
            is_outlier = true;
            state.outliers_rejected++;

            // Log first few outliers for debugging
            if (state.outliers_rejected <= 10) {
                printf("OUTLIER REJECTED: measured=%+.0fns predicted=%+.0fns innovation=%+.0fns (thresh=%.0fns)\n",
                       measured_offset, predicted_offset, innovation, outlier_threshold);
            }
        }
    }

    // Determine timestamp quality for Kalman and correction factor adjustment
    bool hw_timestamps_used = ptp_sync_data.rx_hw_timestamp_valid && ptp_sync_data.tx_hw_timestamp_valid;
    double measurement_noise_r = hw_timestamps_used ? KALMAN_R_MEASUREMENT_HW : KALMAN_R_MEASUREMENT_SW;

    // Update step: Correct prediction with noisy measurement (unless outlier)
    if (!is_outlier) {
        kalman_update(measured_offset, measurement_noise_r);
    }

    // Extract Kalman estimates (use prediction if we rejected measurement)
    double filtered_offset_ns = state.kalman_x[0];
    double filtered_freq_ppb = state.kalman_x[1];

    // Update state variables for logging/stats
    state.offset_from_master_ns = (int64_t)filtered_offset_ns;
    state.freq_offset_ppb = filtered_freq_ppb;

    // 5. Update Free-Running PTP Clock
    if (state.discipline_updates < 5) {
        // First few cycles: Jump clock to establish rough alignment
        state.ptp_clock_ns = ptp_sync_data.t1_master_ns + state.mean_path_delay_ns +
                            ptp_sync_data.correction_sync_ns;
        state.ptp_clock_update_us = ptp_sync_data.t2_slave_us;
#if DEBUG_VERBOSE
        printf("Jumping PTP clock to %llu ns (cycle %lu)\n", state.ptp_clock_ns, state.discipline_updates);
#endif
    } else {
        // Adaptive correction: more aggressive during initial convergence
        update_ptp_clock();  // Bring clock up to date first

        double correction_factor;

        // Reduce correction factor for software timestamps (low confidence)
        if (!hw_timestamps_used) {
            // Software timestamp - minimal correction (2%)
            correction_factor = 0.02;
        } else if (state.discipline_updates < 30) {
            // Initial convergence: 50% correction for faster settling
            correction_factor = 0.5;
        } else if (state.discipline_updates < 100) {
            // Medium convergence: 30% correction
            correction_factor = 0.3;
        } else {
            // Steady state: 20% correction (gentle, stable)
            correction_factor = 0.2;
        }

        int64_t correction = (int64_t)(filtered_offset_ns * correction_factor);
        state.ptp_clock_ns -= correction;  // Subtract offset to bring us closer to master
    }

    // Derive scale_factor from Kalman frequency estimate
    // scale_factor = 1.0 + (freq_offset_ppb / 1e9)
    // But also apply heavy filtering to keep it stable
    double kalman_scale = 1.0 + (filtered_freq_ppb / 1e9);
    if (state.scale_factor == 0.0 || state.discipline_updates < 3) {
        state.scale_factor = kalman_scale;
    } else {
        // Heavy filtering: 95% old, 5% new (20 second time constant)
        state.scale_factor = state.scale_factor * 0.95 + kalman_scale * 0.05;
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
    update_stats(&state.scale_factor_stats, state.scale_factor * 1e6);  // Convert to ppm deviation from 1.0

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
    ptp_stats.scale_factor = state.scale_factor;
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

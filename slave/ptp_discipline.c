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

// Use same PIO program as GPS discipline
#include "counter_simple.pio.h"

// PIO configuration
#define DISCIPLINE_PIO pio0
#define COUNTER_SM 0

// Expected ticks per second at system clock / 3
// At 250 MHz: 250000000 / 3 = 83333333 ticks/sec (12ns resolution)
#define EXPECTED_TICKS_PER_SECOND 83333333ULL

// PI servo controller gains (reduced for stability)
#define SERVO_KP 0.01    // Proportional gain (was 0.1)
#define SERVO_KI 0.0001  // Integral gain (was 0.001)

// Lock thresholds
#define LOCK_THRESHOLD_NS 10000     // 10 microseconds
#define LOCK_SAMPLES_REQUIRED 5
#define UNLOCK_THRESHOLD_NS 100000  // 100 microseconds

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

    // Servo state
    double freq_offset_ppb;
    int64_t integral_term;

    // Lock detection
    uint32_t lock_sample_count;
    bool locked;

    // Statistics
    uint32_t sync_count;
    uint32_t discipline_updates;

    // Crystal characterization (DISABLED - kept for future use)
    double scale_factor;
    int64_t crystal_error_ns;
} ptp_discipline_state_t;

static ptp_discipline_state_t state = {0};

// Global shared state instances
ptp_sync_data_t ptp_sync_data = {0};
ptp_discipline_stats_t ptp_stats = {0};

/**
 * Initialize PTP discipline system
 */
bool ptp_discipline_init(void) {
    printf("Initializing PTP discipline...\n");

    // Load PIO program (simple free-running counter)
    uint counter_offset = pio_add_program(DISCIPLINE_PIO, &counter_simple_program);
    printf("  PIO program loaded (counter @ %d)\n", counter_offset);

    // Initialize counter state machine
    counter_simple_program_init(DISCIPLINE_PIO, COUNTER_SM, counter_offset);
    printf("  Counter SM initialized (83.33 MHz free-running)\n");

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
 * Update free-running PTP clock (call frequently)
 */
static void update_ptp_clock(void) {
    uint64_t now_us = time_us_64();

    if (state.ptp_clock_update_us == 0) {
        // Not initialized yet
        return;
    }

    uint64_t elapsed_us = now_us - state.ptp_clock_update_us;
    if (elapsed_us == 0) {
        return;
    }

    // Convert to nanoseconds
    uint64_t elapsed_ns = elapsed_us * 1000ULL;

    // Apply frequency offset
    int64_t correction_ns = ((int64_t)elapsed_ns * (int64_t)state.freq_offset_ppb) / 1000000000LL;
    int64_t corrected_elapsed_ns = (int64_t)elapsed_ns + correction_ns;

    if (corrected_elapsed_ns > 0) {
        state.ptp_clock_ns += (uint64_t)corrected_elapsed_ns;
    }

    state.ptp_clock_update_us = now_us;
}

/**
 * Get current PTP time (from free-running clock)
 */
uint64_t __time_critical_func(get_ptp_time_ns)(void) {
    // Update clock with current frequency offset
    uint64_t now_us = time_us_64();

    if (state.ptp_clock_update_us == 0) {
        return 0;  // Not initialized yet
    }

    uint64_t elapsed_us = now_us - state.ptp_clock_update_us;
    uint64_t elapsed_ns = elapsed_us * 1000ULL;

    // Apply frequency offset
    int64_t correction_ns = ((int64_t)elapsed_ns * (int64_t)state.freq_offset_ppb) / 1000000000LL;
    int64_t corrected_elapsed_ns = (int64_t)elapsed_ns + correction_ns;

    if (corrected_elapsed_ns > 0) {
        return state.ptp_clock_ns + (uint64_t)corrected_elapsed_ns;
    }

    return state.ptp_clock_ns;
}

/**
 * Update discipline (IEEE 1588 algorithm)
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

        printf("First Sync received, PTP clock initialized\n");

        ptp_sync_data.sync_followup_ready = false;
        ptp_sync_data.delay_resp_ready = false;
        return;
    }

    // Discard early Delay_Resp until we have stable timing
    if (state.syncs_since_boot < 3) {
        printf("Sync #%lu received, waiting for Delay_Resp\n", state.syncs_since_boot);

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
        printf("Discarding Delay_Resp: t3 (%llu) before t2 (%llu)\n", t3_ptp_ns, t2_ptp_ns);
        ptp_sync_data.delay_resp_ready = false;
        return;
    }

    // 1. Calculate Mean Path Delay (IEEE 1588-2008 Eq. 3)
    // path_delay = ((t2 - t1) - correctionSync + (t4 - t3) - correctionDelayResp) / 2
    int64_t term1 = (int64_t)(t2_ptp_ns - ptp_sync_data.t1_master_ns) - ptp_sync_data.correction_sync_ns;
    int64_t term2 = (int64_t)(ptp_sync_data.t4_master_ns - t3_ptp_ns) - ptp_sync_data.correction_delay_resp_ns;
    int64_t path_delay_raw = (term1 + term2) / 2;

    // Debug first few cycles
    if (state.discipline_updates < 3) {
        printf("PATH DEBUG: term1(t2-t1)=%+lld term2(t4-t3)=%+lld path_delay=%+lld\n",
               term1, term2, path_delay_raw);
        printf("            t2_ptp=%llu t1_master=%llu\n", t2_ptp_ns, ptp_sync_data.t1_master_ns);
        printf("            t4_master=%llu t3_ptp=%llu\n", ptp_sync_data.t4_master_ns, t3_ptp_ns);
    }

    // Filter path delay with exponential moving average (α = 0.125)
    if (state.mean_path_delay_ns == 0) {
        state.mean_path_delay_ns = path_delay_raw;
    } else {
        state.mean_path_delay_ns = (state.mean_path_delay_ns * 7 + path_delay_raw) / 8;
    }

    // 2. Calculate Offset from Master (IEEE 1588-2008 Eq. 4)
    // offset = (t2 - t1) - mean_path_delay - correctionSync
    // This tells us how much our slave PTP clock is ahead of master clock
    state.offset_from_master_ns =
        (int64_t)(t2_ptp_ns - ptp_sync_data.t1_master_ns) -
        state.mean_path_delay_ns -
        ptp_sync_data.correction_sync_ns;

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

    // Debug output for first few cycles
    if (state.discipline_updates < 5) {
        printf("DEBUG: master_delta_ns=%llu expected_ticks=%llu elapsed_ticks=%lu\n",
               master_delta_ns, expected_ticks, elapsed_ticks);
        printf("       prev_counter=0x%08lX current_counter=0x%08lX\n",
               state.prev_counter_value, ptp_sync_data.counter_at_sync);
    }

    int64_t crystal_error_ticks = (int64_t)elapsed_ticks - (int64_t)expected_ticks;
    state.crystal_error_ns = crystal_error_ticks * 12; // 12ns per tick

    // Update scale factor for interpolation
    if (elapsed_ticks > 0) {
        state.scale_factor = (double)expected_ticks / (double)elapsed_ticks;
    }

    // 4. PI Servo Controller (NEGATIVE feedback)
    // If offset > 0, we're ahead, so need to slow down (negative frequency adjustment)
    double proportional = -(double)state.offset_from_master_ns * SERVO_KP;
    state.integral_term -= (int64_t)((double)state.offset_from_master_ns * SERVO_KI);
    state.freq_offset_ppb = proportional + (double)state.integral_term;

    // Limit integral windup
    if (state.integral_term > 1000000) state.integral_term = 1000000;
    if (state.integral_term < -1000000) state.integral_term = -1000000;

    // 5. Update Free-Running PTP Clock
    if (state.discipline_updates < 5) {
        // First few cycles: Jump clock to establish rough alignment
        state.ptp_clock_ns = ptp_sync_data.t1_master_ns + state.mean_path_delay_ns +
                            ptp_sync_data.correction_sync_ns;
        state.ptp_clock_update_us = ptp_sync_data.t2_slave_us;
        printf("Jumping PTP clock to %llu ns (cycle %lu)\n", state.ptp_clock_ns, state.discipline_updates);
    } else {
        // Steady state: Apply servo correction gradually (no big jumps)
        // Adjust clock by a fraction of the measured offset
        update_ptp_clock();  // Bring clock up to date first
        int64_t correction = state.offset_from_master_ns / 2;  // 50% correction per cycle
        state.ptp_clock_ns -= correction;  // Subtract offset to bring us closer to master
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

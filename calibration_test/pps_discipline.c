/**
 * PPS Discipline Module - Calibration Device
 *
 * Disciplines local crystal to external GM 1PPS reference.
 * Simplified from GPS discipline - just counts PPS edges and characterizes crystal.
 */

#include "pps_discipline.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// PIO configuration (must match main_calibration_device.c)
#define DISCIPLINE_PIO pio0
#define COUNTER_SM 0
#define PPS_CAPTURE_SM 1

// Expected ticks per second at system clock / 3
// At 250 MHz: 250000000 / 3 = 83333333 ticks/sec (12ns resolution)
#define EXPECTED_TICKS_PER_SECOND 83333333ULL

// EMA filter for scale factor (2% new, 98% old = 50 second time constant)
#define SCALE_FACTOR_ALPHA 0.02

// Lock thresholds
#define LOCK_THRESHOLD_NS 1000      // 1 microsecond
#define LOCK_SAMPLES_REQUIRED 10
#define UNLOCK_THRESHOLD_NS 10000   // 10 microseconds

// Discipline state
typedef struct {
    // PPS tracking
    uint32_t prev_counter_value;
    uint64_t pps_count;
    uint64_t reference_time_s;      // Time in whole seconds (PPS count)
    uint64_t last_pps_time_us;      // System time when last PPS occurred
    bool first_pps;

    // Crystal characterization
    double scale_factor;
    int64_t crystal_error_ns;

    // Lock detection
    uint32_t lock_sample_count;
    bool locked;

    // Statistics
    uint64_t discipline_updates;
} pps_discipline_state_t;

static pps_discipline_state_t state = {0};

// Global statistics
pps_discipline_stats_t pps_stats = {0};

/**
 * PPS IRQ handler - TIME CRITICAL
 *
 * Called when PPS edge detected. Reads counter value from FIFO immediately
 * and processes discipline. Placed in SRAM for deterministic execution.
 */
static void __time_critical_func(pps_discipline_irq_handler)(void) {
    // *** CRITICAL: Capture system time IMMEDIATELY ***
    uint64_t pps_system_us = time_us_64();

    // Check if PIO0 IRQ 1 triggered (from SM1)
    if (!pio_interrupt_get(DISCIPLINE_PIO, 1)) {
        return;
    }

    // Clear interrupt
    pio_interrupt_clear(DISCIPLINE_PIO, 1);

    // Read counter value from SM0 FIFO
    // SM0 pushed this value when SM1 signaled via trigger pin
    // This is the counter value at the EXACT moment of PPS edge
    if (pio_sm_is_rx_fifo_empty(DISCIPLINE_PIO, COUNTER_SM)) {
        // FIFO empty - should never happen, bail out
        return;
    }

    uint32_t counter_at_pps = pio_sm_get(DISCIPLINE_PIO, COUNTER_SM);

    // Process discipline with precise counter value
    pps_discipline_on_edge(counter_at_pps);
}

/**
 * Initialize PPS discipline system
 */
bool pps_discipline_init(uint pps_input_pin) {
    printf("Initializing PPS discipline...\n");
    printf("  PPS input pin: GPIO%d\n", pps_input_pin);

    // Initialize state
    state.first_pps = true;
    state.scale_factor = 1.0;
    state.locked = false;
    state.lock_sample_count = 0;
    state.reference_time_s = 0;
    state.pps_count = 0;
    state.discipline_updates = 0;

    // Note: PIO counter and PPS capture are initialized in main
    // This function sets up the IRQ handler

    // Set up PIO IRQ handler
    uint pio_irq = PIO0_IRQ_1;  // Using IRQ 1 (SM1 triggers this)
    irq_set_exclusive_handler(pio_irq, pps_discipline_irq_handler);

    // CRITICAL: Set PPS IRQ to HIGHEST priority (0) for deterministic timing
    // Lower number = higher priority on ARM Cortex-M0+
    irq_set_priority(pio_irq, 0);

    irq_set_enabled(pio_irq, true);
    pio_set_irq1_source_enabled(DISCIPLINE_PIO, pis_interrupt1, true);

    printf("  IRQ handler installed (priority 0 - highest)\n");
    printf("PPS discipline ready (interrupt-driven)\n");
    return true;
}

/**
 * Handle PPS edge event
 */
void pps_discipline_on_edge(uint32_t counter_value) {
    uint64_t now_us = time_us_64();
    state.pps_count++;

    if (state.first_pps) {
        // First PPS - just save counter value and initialize
        state.prev_counter_value = counter_value;
        state.reference_time_s = 0;
        state.last_pps_time_us = now_us;
        state.first_pps = false;
        state.scale_factor = 1.0;

        printf("First PPS edge detected (counter=0x%08lX)\n", counter_value);
        return;
    }

    // Calculate elapsed ticks (counter counts DOWN)
    uint32_t elapsed_ticks;
    if (state.prev_counter_value >= counter_value) {
        elapsed_ticks = state.prev_counter_value - counter_value;
    } else {
        // Wraparound
        elapsed_ticks = state.prev_counter_value + (0xFFFFFFFF - counter_value) + 1;
    }

    // Calculate crystal error
    // GM 1PPS should be exactly 1 second apart = EXPECTED_TICKS_PER_SECOND
    int64_t error_ticks = (int64_t)elapsed_ticks - (int64_t)EXPECTED_TICKS_PER_SECOND;
    state.crystal_error_ns = error_ticks * 12;  // 12ns per tick

    // Update scale factor (EMA filter for stability)
    if (elapsed_ticks > 0) {
        double measured_scale = (double)EXPECTED_TICKS_PER_SECOND / (double)elapsed_ticks;

        if (state.discipline_updates < 3) {
            // Bootstrap: use measured value directly
            state.scale_factor = measured_scale;
        } else {
            // EMA filter: 98% old, 2% new (50 second time constant)
            state.scale_factor = state.scale_factor * (1.0 - SCALE_FACTOR_ALPHA) +
                                measured_scale * SCALE_FACTOR_ALPHA;
        }
    }

    // Lock detection
    bool was_locked = state.locked;
    if (llabs(state.crystal_error_ns) < LOCK_THRESHOLD_NS) {
        state.lock_sample_count++;
        if (state.lock_sample_count >= LOCK_SAMPLES_REQUIRED) {
            state.locked = true;
        }
    } else if (llabs(state.crystal_error_ns) > UNLOCK_THRESHOLD_NS) {
        state.lock_sample_count = 0;
        state.locked = false;
    }

    // Log lock transitions
    if (!was_locked && state.locked) {
        printf("PPS discipline LOCKED (error=%+lldns, scale=%.9f)\n",
               (long long)state.crystal_error_ns, state.scale_factor);
    } else if (was_locked && !state.locked) {
        printf("PPS discipline UNLOCKED (error=%+lldns)\n",
               (long long)state.crystal_error_ns);
    }

    // Update reference time (just count seconds)
    state.reference_time_s++;
    state.last_pps_time_us = now_us;
    state.discipline_updates++;

    // Update previous counter
    state.prev_counter_value = counter_value;

    // Update global statistics
    pps_stats.locked = state.locked;
    pps_stats.crystal_error_ns = state.crystal_error_ns;
    pps_stats.crystal_ppm = (double)state.crystal_error_ns / 1000000.0;
    pps_stats.scale_factor = state.scale_factor;
    pps_stats.pps_count = state.pps_count;
    pps_stats.discipline_updates = state.discipline_updates;
    pps_stats.reference_time_ns = get_reference_time_ns();

    // Periodic logging (every 10 PPS edges)
    if (state.discipline_updates % 10 == 0) {
        printf("PPS#%03llu: Lock=%c Error=%+6lldns Scale=%.9f PPM=%+.3f\n",
               state.pps_count,
               state.locked ? 'Y' : 'N',
               (long long)state.crystal_error_ns,
               state.scale_factor,
               pps_stats.crystal_ppm);
    }
}

/**
 * Get current reference time in nanoseconds
 *
 * Interpolates between PPS edges using characterized crystal scale factor.
 * Returns PURE reference time with NO offset added.
 * Main files can add offsets as needed for their specific use case.
 */
uint64_t get_reference_time_ns(void) {
    if (state.first_pps) {
        return 0;  // Not initialized yet
    }

    // Current second boundary (in nanoseconds)
    uint64_t base_time_ns = state.reference_time_s * 1000000000ULL;

    // Microseconds elapsed since last PPS edge
    uint64_t now_us = time_us_64();
    uint64_t elapsed_us = now_us - state.last_pps_time_us;

    // Scale to true nanoseconds using characterized crystal scale factor
    // scale_factor = expected_ticks / measured_ticks = how to convert crystal time to true time
    double scale = state.scale_factor;
    if (scale == 0.0 || state.discipline_updates < 3) {
        scale = 1.0;  // Use nominal rate until characterized
    }

    uint64_t elapsed_ns = (uint64_t)((double)elapsed_us * 1000.0 * scale);

    // Clamp to avoid going past next second (handle clock adjustments)
    if (elapsed_ns > 1000000000ULL) {
        elapsed_ns = 999999999ULL;
    }

    return base_time_ns + elapsed_ns;
}

/**
 * Get current scale factor
 */
double pps_discipline_get_scale_factor(void) {
    return state.scale_factor;
}

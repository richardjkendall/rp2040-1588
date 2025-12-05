/**
 * GPS Disciplined Clock - V3 with GPIO Pin Synchronization
 *
 * Eliminates bus contention by using GPIO pin signaling instead of continuous DMA
 * SM0 counter only pushes value when GPS PPS arrives (1 Hz vs 250 MHz)
 *
 * Architecture:
 * - PIO0 SM0: 250 MHz counter, pushes only when trigger pin goes high
 * - PIO0 SM1: GPS PPS detector, signals SM0 via GPIO 16 pulse
 * - NO DMA: CPU reads from SM0 FIFO in IRQ handler
 * - Bus usage: 250M times less than continuous DMA approach
 */

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "gps.h"
#include "shared_state.h"
#include "../common/discipline.h"
#include "gps_discipline_v2.pio.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// PIO configuration
#define DISCIPLINE_PIO pio0
#define COUNTER_SM 0
#define PPS_CAPTURE_SM 1

// GPS PPS pin
#define GPS_PPS_PIN 2

// Internal trigger pin (GPIO 22 - connects SM1 to SM0)
// CHANGED from GPIO 16 to avoid conflict with W5500 MISO
#define TRIGGER_PIN 22

// LED output pin
#define LED_OUTPUT_PIN 3

// Debug GPIOs
extern uint debug_lock_pin;
extern uint debug_pps_pin;

// Expected ticks per second
// Counter runs at system_clock / 3 due to PIO program structure (3 cycles per decrement)
// At 250 MHz: 250000000 / 3 = 83333333 ticks/sec (12ns resolution)
#define EXPECTED_TICKS_PER_SECOND 83333333ULL

// Software time correction - we accumulate phase error directly
// since we can't adjust the actual clock frequency
// P term provides damping, no separate I term needed

// Lock thresholds (relaxed for testing)
#define LOCK_THRESHOLD_TICKS 25000      // 100μs @ 250 MHz
#define LOCK_SAMPLES_REQUIRED 5
#define UNLOCK_THRESHOLD_TICKS 250000   // 1ms @ 250 MHz

// Previous counter value for elapsed calculation
static uint32_t prev_counter_value = 0;
static bool first_pps = true;

// GPS Nanosecond Counter State
volatile uint64_t gps_ns_counter = 0;           // Absolute GPS time in nanoseconds
static volatile uint64_t last_pps_system_us = 0;       // System timer reading at last GPS PPS
static volatile uint32_t measured_ticks_last_second = EXPECTED_TICKS_PER_SECOND;  // Actual ticks in last GPS second
static uint32_t lock_sample_count = 0;

// Performance metrics (exported for monitoring)
volatile int32_t crystal_error_ns = 0;          // Crystal error this second (ns)
volatile int64_t interpolation_error_ns = 0;    // Interpolation accuracy at PPS boundary (ns)

// Get crystal error in nanoseconds (for monitoring)
int32_t discipline_get_crystal_error_ns(void) {
    return crystal_error_ns;
}

// Get interpolation error in nanoseconds (for monitoring)
int64_t discipline_get_interpolation_error_ns(void) {
    return interpolation_error_ns;
}

// Get GPS nanosecond counter value (for monitoring)
uint64_t discipline_get_gps_ns(void) {
    return gps_ns_counter;
}

// Get scale factor (measured ticks / expected ticks)
double discipline_get_scale_factor(void) {
    return (double)measured_ticks_last_second / (double)EXPECTED_TICKS_PER_SECOND;
}

// Shared state for reporting
core1_stats_t core1_stats = {0};

// 100 PPS state
static volatile uint64_t gps_pps_timestamp_us = 0;  // Timestamp when GPS PPS arrived
static bool output_100pps_enabled = false;
static uint64_t last_pulse_time_us = 0;

/**
 * Get GPS time in nanoseconds
 * Returns absolute GPS time by interpolating between GPS PPS boundaries
 *
 * CRITICAL: Placed in SRAM for deterministic execution (no flash cache misses)
 */
uint64_t __time_critical_func(get_gps_time_ns)(void) {
    if (last_pps_system_us == 0) {
        // No GPS PPS yet, return 0
        return 0;
    }

    // How far through current second? (in system timer microseconds)
    uint64_t now_us = time_us_64();
    uint64_t elapsed_system_us = now_us - last_pps_system_us;

    // Scale to GPS nanoseconds using measured tick rate
    // System timer runs at same crystal rate as PIO counter
    // scale = expected_ticks / measured_ticks = how to convert crystal time to GPS time
    double scale = (double)EXPECTED_TICKS_PER_SECOND / (double)measured_ticks_last_second;
    uint64_t gps_elapsed_ns = (uint64_t)((double)elapsed_system_us * 1000.0 * scale);

    // GPS time = last GPS second boundary + scaled elapsed time
    return gps_ns_counter + gps_elapsed_ns;
}

// Legacy function for compatibility - returns GPS time in microseconds
uint64_t __time_critical_func(get_disciplined_time_us)(void) {
    return get_gps_time_ns() / 1000;
}

/**
 * PIO IRQ handler - called when GPS PPS edge detected
 *
 * CRITICAL: Placed in SRAM for deterministic, low-jitter execution
 */
static void __time_critical_func(discipline_pps_irq_handler)(void) {
    // *** CRITICAL: Capture system time IMMEDIATELY ***
    // This is our time reference for GPS PPS - must be first instruction to minimize latency
    uint64_t pps_system_us = time_us_64();

    // Check if PIO0 IRQ 1 triggered (from SM1)
    if (!pio_interrupt_get(DISCIPLINE_PIO, 1)) {
        return;
    }

    // Clear interrupt
    pio_interrupt_clear(DISCIPLINE_PIO, 1);

    // REMOVED: GPIO toggle uses spinlocks that can cause contention with Core 1
    // gpio_put(debug_pps_pin, !gpio_get(debug_pps_pin));

    // Read counter value from SM0 FIFO
    // SM0 pushed this value when SM1 signaled via IRQ flag
    // This is the counter value at the EXACT moment of GPS PPS edge
    if (pio_sm_is_rx_fifo_empty(DISCIPLINE_PIO, COUNTER_SM)) {
        // FIFO empty - should never happen, bail out
        return;
    }

    uint32_t counter_at_pps = pio_sm_get(DISCIPLINE_PIO, COUNTER_SM);

    // Skip first PPS
    if (first_pps) {
        prev_counter_value = counter_at_pps;
        first_pps = false;
        core1_stats.first_pps_received = true;

        // Initialize GPS counter at second 0
        gps_ns_counter = 0;
        last_pps_system_us = pps_system_us;  // Use timestamp captured at IRQ entry
        measured_ticks_last_second = EXPECTED_TICKS_PER_SECOND;

        // Enable 100 PPS output
        output_100pps_enabled = true;
        gps_pps_timestamp_us = last_pps_system_us;
        last_pulse_time_us = 0;

        return;
    }

    // Calculate elapsed ticks (handle wraparound)
    uint32_t elapsed_ticks;
    if (counter_at_pps > prev_counter_value) {
        elapsed_ticks = prev_counter_value + (0xFFFFFFFF - counter_at_pps) + 1;
    } else {
        elapsed_ticks = prev_counter_value - counter_at_pps;
    }

    // ========================================================================
    // GPS NANOSECOND COUNTER MODEL
    // Maintain absolute GPS time, measure performance retrospectively
    // ========================================================================

    // 1. Calculate crystal error THIS second
    int64_t phase_error_ticks = (int64_t)elapsed_ticks - (int64_t)EXPECTED_TICKS_PER_SECOND;
    crystal_error_ns = (int32_t)(phase_error_ticks * 12);  // Convert ticks to nanoseconds (12ns/tick)

    // Calculate frequency offset in PPB for stats
    double freq_offset_ppb = ((double)phase_error_ticks * 1e9) / (double)EXPECTED_TICKS_PER_SECOND;

    // 2. Measure interpolation error using the timestamp captured at IRQ entry
    // Calculate what our interpolation would return at pps_system_us (not time_us_64() now)
    uint64_t elapsed_system_us = pps_system_us - last_pps_system_us;
    double scale = (double)EXPECTED_TICKS_PER_SECOND / (double)measured_ticks_last_second;
    uint64_t gps_elapsed_ns = (uint64_t)((double)elapsed_system_us * 1000.0 * scale);

    // GPS time at pps_system_us = last GPS boundary + scaled elapsed
    uint64_t interpolated_gps_ns = gps_ns_counter + gps_elapsed_ns;

    // Should equal exactly the next second boundary
    uint64_t expected_gps_ns = gps_ns_counter + 1000000000ULL;
    interpolation_error_ns = (int64_t)(interpolated_gps_ns - expected_gps_ns);

    // 3. Update GPS nanosecond counter - hard sync to GPS second boundary
    gps_ns_counter += 1000000000ULL;

    // 4. Record system time at this GPS boundary (captured at IRQ entry)
    last_pps_system_us = pps_system_us;

    // 5. Store measured ticks for next interpolation
    measured_ticks_last_second = elapsed_ticks;

    // Lock detection
    bool was_locked = core1_stats.locked;
    if (abs((int)phase_error_ticks) < LOCK_THRESHOLD_TICKS) {
        lock_sample_count++;
        if (lock_sample_count >= LOCK_SAMPLES_REQUIRED) {
            if (!was_locked) {
                core1_stats.lock_event_count++;
            }
            core1_stats.locked = true;
        }
    } else if (abs((int)phase_error_ticks) > UNLOCK_THRESHOLD_TICKS) {
        lock_sample_count = 0;
        if (was_locked) {
            core1_stats.unlock_event_count++;
        }
        core1_stats.locked = false;
    }

    // Update statistics
    core1_stats.pps_count++;
    core1_stats.phase_error_ns = phase_error_ticks * 12;  // 12ns per tick
    core1_stats.freq_offset_ppb = (int32_t)freq_offset_ppb;
    core1_stats.last_update_us = pps_system_us;  // Use captured timestamp (avoid second time_us_64() call)

    // Update previous counter
    prev_counter_value = counter_at_pps;

    // Record GPS PPS timestamp for 100 PPS generation
    // Use last_pps_system_us which is exactly at GPS second boundary
    gps_pps_timestamp_us = last_pps_system_us;
}

/**
 * Initialize GPS discipline system V3
 */
bool discipline_init_v3(void) {
    printf("Initializing GPS discipline (V3 - GPIO pin sync)...\n");

    // Load PIO programs
    uint counter_offset = pio_add_program(DISCIPLINE_PIO, &counter_with_pin_trigger_program);
    uint pps_offset = pio_add_program(DISCIPLINE_PIO, &pps_edge_with_pin_signal_program);

    printf("  PIO programs loaded (counter @ %d, pps @ %d)\n", counter_offset, pps_offset);

    // Initialize counter SM (SM0)
    counter_with_pin_trigger_program_init(DISCIPLINE_PIO, COUNTER_SM, counter_offset, TRIGGER_PIN);
    printf("  Counter SM initialized (250 MHz, pin-triggered push on GPIO %d)\n", TRIGGER_PIN);

    // Initialize PPS capture SM (SM1)
    pps_edge_with_pin_signal_program_init(DISCIPLINE_PIO, PPS_CAPTURE_SM, pps_offset, GPS_PPS_PIN, TRIGGER_PIN);
    printf("  PPS capture SM initialized (GPIO %d PPS -> GPIO %d trigger)\n", GPS_PPS_PIN, TRIGGER_PIN);

    // NO DMA NEEDED! SM0 and SM1 communicate via GPIO pin

    // Set up PIO IRQ handler
    uint pio_irq = PIO0_IRQ_1;  // Using IRQ 1 (SM1 triggers this)
    irq_set_exclusive_handler(pio_irq, discipline_pps_irq_handler);

    // CRITICAL: Set GPS PPS IRQ to HIGHEST priority (0) so it cannot be blocked
    // by network/lwIP interrupts. Lower number = higher priority on ARM Cortex-M0+
    irq_set_priority(pio_irq, 0);

    irq_set_enabled(pio_irq, true);
    pio_set_irq1_source_enabled(DISCIPLINE_PIO, pis_interrupt1, true);

    printf("  IRQ handler installed (priority 0 - highest)\n");

    // Mark system as running
    core1_stats.discipline_running = true;

    printf("GPS discipline ready (V3 - 12ns resolution, no bus contention)\n");
    return true;
}

// Debug counters
static uint32_t debug_pulse_fired_count = 0;
static uint32_t debug_out_of_range_count = 0;
static uint32_t debug_too_early_count = 0;
static uint32_t debug_too_late_count = 0;
static uint32_t debug_guard_blocked_count = 0;

/**
 * Generate 100 PPS output
 */
void discipline_generate_100pps(void) {
    if (!output_100pps_enabled) {
        return;
    }

    // Use GPS-disciplined time for accurate pulse timing
    uint64_t now_us = get_disciplined_time_us();

    // Read GPS PPS timestamp (volatile, may be updated by IRQ)
    uint64_t pps_timestamp = gps_pps_timestamp_us;

    // Calculate elapsed time since last GPS PPS
    int64_t elapsed = (int64_t)(now_us - pps_timestamp);

    // Calculate which pulse number we're at (0-99)
    int32_t pulse_num = elapsed / 10000;

    // Outside valid range? Wait for next GPS PPS
    if (pulse_num < 0 || pulse_num > 99) {
        debug_out_of_range_count++;
        return;
    }

    // Calculate when this pulse should fire (relative to GPS PPS timestamp)
    uint64_t pulse_time = pps_timestamp + ((uint64_t)pulse_num * 10000);

    // Already fired this pulse? (within last 8ms)
    if (last_pulse_time_us > 0 && (now_us - last_pulse_time_us) < 8000) {
        debug_guard_blocked_count++;
        return;
    }

    // Too early for this pulse? (more than 100μs early)
    if (now_us + 100 < pulse_time) {
        debug_too_early_count++;
        return;
    }

    // Too late for this pulse? (more than 2ms late)
    if (now_us > pulse_time + 2000) {
        debug_too_late_count++;
        return;
    }

    // Fire the pulse!
    gpio_put(LED_OUTPUT_PIN, 1);
    busy_wait_us(100);  // 100μs pulse width
    gpio_put(LED_OUTPUT_PIN, 0);

    // Remember when we fired
    last_pulse_time_us = now_us;
    debug_pulse_fired_count++;
}

/**
 * Get debug stats for 100 PPS generation
 */
void discipline_get_100pps_stats(uint32_t *fired, uint32_t *out_of_range, uint32_t *too_early, uint32_t *too_late, uint32_t *guard_blocked) {
    *fired = debug_pulse_fired_count;
    *out_of_range = debug_out_of_range_count;
    *too_early = debug_too_early_count;
    *too_late = debug_too_late_count;
    *guard_blocked = debug_guard_blocked_count;
}

/**
 * Update discipline stats
 */
void discipline_update_stats(void) {
    uint64_t disciplined_us = get_disciplined_time_us();
    core1_stats.continuous_time_ns = disciplined_us * 1000ULL;
    core1_stats.disciplined_time_ns = (disciplined_us % 1000000) * 1000ULL;
}

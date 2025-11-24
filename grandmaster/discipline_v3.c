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

// Internal trigger pin (GPIO 16 - connects SM1 to SM0)
#define TRIGGER_PIN 16

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

// Discipline state - PI controller for frequency estimate
static volatile double freq_offset_ppm = 0.0;     // Estimated frequency offset (integral term)
static volatile uint64_t last_pps_raw_us = 0;     // Raw time at last GPS PPS
static uint32_t lock_sample_count = 0;

// Integral gain for frequency-locked loop
#define FREQ_KI 0.1  // How quickly to adjust frequency estimate (0.1 = 10% per GPS PPS)

// Disciplined time accuracy measurement
static volatile int64_t disciplined_error_ns = 0;  // Error in disciplined time at GPS PPS

// Expose frequency offset for monitoring
int64_t discipline_get_correction_us(void) {
    // Return current frequency offset in μs/sec
    return (int64_t)freq_offset_ppm;
}

// Debug: get detailed compensation info
void discipline_get_debug_info(int64_t *elapsed_raw, double *freq_ppm, int64_t *freq_correction_calc) {
    uint64_t raw_us = time_us_64();

    if (last_pps_raw_us == 0) {
        *elapsed_raw = 0;
        *freq_ppm = 0.0;
        *freq_correction_calc = 0;
        return;
    }

    *elapsed_raw = (int64_t)(raw_us - last_pps_raw_us);
    *freq_ppm = freq_offset_ppm;

    double freq_corr = (double)(*elapsed_raw) * freq_offset_ppm / 1000000.0;
    *freq_correction_calc = (int64_t)freq_corr;
}


int64_t discipline_get_disciplined_error_ns(void) {
    return disciplined_error_ns;
}

// Shared state for reporting
core1_stats_t core1_stats = {0};

// 100 PPS state
static volatile uint64_t gps_pps_timestamp_us = 0;  // Timestamp when GPS PPS arrived
static bool output_100pps_enabled = false;
static uint64_t last_pulse_time_us = 0;

/**
 * Get disciplined time in microseconds
 * Uses PI-controlled frequency estimate to compensate for crystal drift
 */
uint64_t get_disciplined_time_us(void) {
    uint64_t raw_us = time_us_64();

    if (last_pps_raw_us == 0) {
        // No GPS PPS yet, return raw time
        return raw_us;
    }

    // Calculate how much time has elapsed since last GPS PPS (in raw crystal time)
    int64_t elapsed_raw_us = (int64_t)(raw_us - last_pps_raw_us);

    // Apply frequency compensation to get GPS-accurate elapsed time
    // freq_offset_ppm > 0 means crystal is fast, so we subtract
    // freq_offset_ppm < 0 means crystal is slow, so we add (subtract negative)
    double freq_correction = (double)elapsed_raw_us * freq_offset_ppm / 1000000.0;
    int64_t elapsed_disciplined_us = elapsed_raw_us - (int64_t)freq_correction;

    // Disciplined time = GPS PPS time + frequency-corrected elapsed
    // (GPS PPS time was exactly on a GPS second boundary)
    return last_pps_raw_us + (uint64_t)elapsed_disciplined_us;
}

/**
 * PIO IRQ handler - called when GPS PPS edge detected
 */
static void discipline_pps_irq_handler(void) {
    // Check if PIO0 IRQ 1 triggered (from SM1)
    if (!pio_interrupt_get(DISCIPLINE_PIO, 1)) {
        return;
    }

    // Clear interrupt
    pio_interrupt_clear(DISCIPLINE_PIO, 1);

    // Toggle debug GPIO
    gpio_put(debug_pps_pin, !gpio_get(debug_pps_pin));

    // Read counter value from SM0 FIFO
    // SM0 pushed this value when SM1 signaled via IRQ flag
    // This is the counter value at the EXACT moment of GPS PPS edge
    if (pio_sm_is_rx_fifo_empty(DISCIPLINE_PIO, COUNTER_SM)) {
        // Should never happen, but guard against it
        return;
    }

    uint32_t counter_at_pps = pio_sm_get(DISCIPLINE_PIO, COUNTER_SM);

    // Skip first PPS
    if (first_pps) {
        prev_counter_value = counter_at_pps;
        first_pps = false;
        core1_stats.first_pps_received = true;

        // Enable 100 PPS output and record timestamp
        output_100pps_enabled = true;
        last_pps_raw_us = time_us_64();
        gps_pps_timestamp_us = last_pps_raw_us;  // First PPS, no correction yet
        freq_offset_ppm = 0.0;
        last_pulse_time_us = 0;
        disciplined_error_ns = 0;

        return;
    }

    // Calculate elapsed ticks (handle wraparound)
    uint32_t elapsed_ticks;
    if (counter_at_pps > prev_counter_value) {
        elapsed_ticks = prev_counter_value + (0xFFFFFFFF - counter_at_pps) + 1;
    } else {
        elapsed_ticks = prev_counter_value - counter_at_pps;
    }

    // Calculate phase error
    int64_t phase_error_ticks = (int64_t)elapsed_ticks - (int64_t)EXPECTED_TICKS_PER_SECOND;

    // Calculate frequency offset in PPB for stats
    double freq_offset_ppb = ((double)phase_error_ticks * 1e9) / (double)EXPECTED_TICKS_PER_SECOND;

    // Use RAW crystal measurement (phase_error_ticks) to update frequency estimate
    // This is the crystal's actual drift relative to GPS, measured by PIO counter
    double phase_error_ppm = ((double)phase_error_ticks * 1.0e6) / (double)EXPECTED_TICKS_PER_SECOND;

    // Exponential moving average for frequency estimate
    // Smoothly converges freq_offset_ppm toward measured phase_error_ppm
    freq_offset_ppm = freq_offset_ppm * (1.0 - FREQ_KI) + phase_error_ppm * FREQ_KI;

    // Measure disciplined time accuracy:
    // At GPS PPS, exactly 1 second of GPS time elapsed
    // How much did disciplined time advance from LAST GPS PPS to THIS GPS PPS?
    uint64_t this_gps_pps_raw = time_us_64();
    uint64_t raw_elapsed_us = this_gps_pps_raw - last_pps_raw_us;

    // Apply frequency correction to raw elapsed time
    double freq_corr = (double)raw_elapsed_us * freq_offset_ppm / 1000000.0;
    int64_t disciplined_elapsed_us = raw_elapsed_us - (int64_t)freq_corr;

    // Disciplined time should have advanced by exactly 1,000,000 μs
    int64_t disciplined_error_us = disciplined_elapsed_us - 1000000;
    disciplined_error_ns = disciplined_error_us * 1000;

    // Update raw time at GPS PPS for next cycle
    last_pps_raw_us = time_us_64();

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
    core1_stats.last_update_us = time_us_64();

    // Update previous counter
    prev_counter_value = counter_at_pps;

    // Record GPS PPS timestamp for 100 PPS generation
    // Use last_pps_raw_us which is exactly at GPS second boundary
    gps_pps_timestamp_us = last_pps_raw_us;
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
    irq_set_enabled(pio_irq, true);
    pio_set_irq1_source_enabled(DISCIPLINE_PIO, pis_interrupt1, true);

    printf("  IRQ handler installed (no DMA - GPIO pin only)\n");

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

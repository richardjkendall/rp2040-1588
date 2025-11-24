/**
 * GPS Disciplined Clock - PIO + DMA Architecture
 *
 * Based on frequency counter design - uses FIFO buffering for atomic PPS capture
 * Eliminates Core 1, runs entirely on interrupt + DMA
 *
 * Architecture:
 * - PIO0 SM0: 250 MHz free-running counter (4ns resolution)
 * - DMA: SM0 TX FIFO → SM1 RX FIFO (continuous feed)
 * - PIO0 SM1: GPS PPS edge detector, captures counter on rising edge
 * - CPU IRQ: Reads snapshot, calculates discipline correction
 * - Main loop: Uses time_us_64() + correction for disciplined time
 */

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "gps.h"
#include "shared_state.h"
#include "../common/discipline.h"
#include "gps_discipline.pio.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// PIO configuration - use PIO0 for both SMs
#define DISCIPLINE_PIO pio0
#define COUNTER_SM 0      // SM0: Free counter
#define PPS_CAPTURE_SM 1  // SM1: PPS edge detector (NOTE: GPS init uses SM0 for old PPS capture!)

// GPS PPS pin
#define GPS_PPS_PIN 2

// LED output pin
#define LED_OUTPUT_PIN 3

// Debug GPIOs (set from main)
extern uint debug_lock_pin;
extern uint debug_pps_pin;

// Expected ticks per second at 250 MHz
#define EXPECTED_TICKS_PER_SECOND 250000000ULL

// PI controller gains
#define KP 0.1
#define KI 0.001

// Lock thresholds (relaxed for testing with DMA contention)
#define LOCK_THRESHOLD_TICKS 25000      // 100μs @ 250 MHz (relaxed)
#define LOCK_SAMPLES_REQUIRED 5          // Fewer samples needed
#define UNLOCK_THRESHOLD_TICKS 250000   // 1ms @ 250 MHz

// DMA channel
static int dma_channel = -1;

// Previous counter value for elapsed calculation
static uint32_t prev_counter_value = 0;
static bool first_pps = true;

// Discipline state
static int64_t accumulated_phase_ticks = 0;
static int64_t correction_ticks = 0;
static volatile int64_t correction_us = 0;  // For time_us_64() correction
static uint32_t lock_sample_count = 0;

// Shared state for reporting
core1_stats_t core1_stats = {0};

// 100 PPS state
static uint64_t output_base_time_us = 0;  // Base time for 100 PPS generation (set on first PPS)
static bool output_100pps_enabled = false;
static uint64_t last_pulse_time_us = 0;   // Time of last generated pulse (prevents duplicates)

/**
 * Get disciplined time in microseconds
 */
uint64_t get_disciplined_time_us(void) {
    return time_us_64() + correction_us;
}

/**
 * PIO IRQ handler - called when GPS PPS edge detected
 * Reads counter snapshot from FIFO and calculates discipline
 */
static void discipline_pps_irq_handler(void) {
    // Check if PIO0 SM1 triggered the interrupt
    if (!pio_interrupt_get(DISCIPLINE_PIO, 0)) {
        return;
    }

    // Clear interrupt
    pio_interrupt_clear(DISCIPLINE_PIO, 0);

    // Toggle debug GPIO (for scope measurement)
    gpio_put(debug_pps_pin, !gpio_get(debug_pps_pin));

    // Read counter value from SM1 TX FIFO
    // This is the counter value AT THE EXACT MOMENT of GPS PPS edge
    // (buffered in FIFO, IRQ latency doesn't affect measurement)
    if (pio_sm_is_rx_fifo_empty(DISCIPLINE_PIO, PPS_CAPTURE_SM)) {
        // Should never happen, but guard against it
        return;
    }

    uint32_t counter_at_pps = pio_sm_get(DISCIPLINE_PIO, PPS_CAPTURE_SM);

    // Skip first PPS (no previous value to compare against)
    if (first_pps) {
        prev_counter_value = counter_at_pps;
        first_pps = false;
        core1_stats.first_pps_received = true;

        // Enable 100 PPS output - set base time
        output_100pps_enabled = true;
        output_base_time_us = time_us_64();  // Use raw time as base (no correction yet)
        last_pulse_time_us = 0;  // No pulses generated yet

        return;
    }

    // Calculate elapsed ticks (handle 32-bit wraparound)
    // Counter counts DOWN, so prev > current normally
    uint32_t elapsed_ticks;
    if (counter_at_pps > prev_counter_value) {
        // Wrapped (counting down: was small, now large)
        elapsed_ticks = prev_counter_value + (0xFFFFFFFF - counter_at_pps) + 1;
    } else {
        // Normal case
        elapsed_ticks = prev_counter_value - counter_at_pps;
    }

    // Calculate phase error (expected vs actual)
    int64_t phase_error_ticks = (int64_t)elapsed_ticks - (int64_t)EXPECTED_TICKS_PER_SECOND;

    // Calculate frequency offset (PPB)
    double freq_offset_ppb = ((double)phase_error_ticks * 1e9) / (double)EXPECTED_TICKS_PER_SECOND;

    // PI Controller
    accumulated_phase_ticks += phase_error_ticks;
    correction_ticks = (int64_t)(KP * phase_error_ticks + KI * accumulated_phase_ticks);

    // Convert correction to microseconds for time_us_64() offset
    // 250 ticks = 1μs
    correction_us = correction_ticks / 250;

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
    core1_stats.phase_error_ns = phase_error_ticks * 4;  // 4ns per tick @ 250 MHz
    core1_stats.freq_offset_ppb = (int32_t)freq_offset_ppb;
    core1_stats.last_update_us = time_us_64();
    core1_stats.disciplined_time_ns = (get_disciplined_time_us() % 1000000) * 1000ULL;

    // Update previous counter
    prev_counter_value = counter_at_pps;

    // Update base time every GPS PPS to keep 100 PPS synchronized
    // Advance by exactly 1 second (1,000,000 μs) with discipline correction
    // This smoothly steers the 100 PPS without discontinuities
    output_base_time_us += 1000000 + (correction_us / 1000);  // Add 1s + small correction
}

/**
 * Initialize PIO-based discipline system
 */
bool discipline_init_v2(void) {
    printf("Initializing GPS discipline (PIO + DMA architecture)...\n");

    // Load PIO programs
    uint counter_offset = pio_add_program(DISCIPLINE_PIO, &free_counter_250mhz_program);
    uint pps_offset = pio_add_program(DISCIPLINE_PIO, &pps_edge_capture_program);

    printf("  PIO programs loaded (counter @ %d, pps @ %d)\n", counter_offset, pps_offset);

    // Initialize counter SM (SM0)
    free_counter_250mhz_program_init(DISCIPLINE_PIO, COUNTER_SM, counter_offset);
    printf("  Counter SM initialized (250 MHz)\n");

    // Initialize PPS capture SM (SM1)
    pps_edge_capture_program_init(DISCIPLINE_PIO, PPS_CAPTURE_SM, pps_offset, GPS_PPS_PIN);
    printf("  PPS capture SM initialized (GPIO %d)\n", GPS_PPS_PIN);

    // Configure DMA: SM0 TX FIFO → SM1 RX FIFO
    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_channel);

    channel_config_set_read_increment(&c, false);  // Always read from same FIFO
    channel_config_set_write_increment(&c, false); // Always write to same FIFO
    channel_config_set_dreq(&c, pio_get_dreq(DISCIPLINE_PIO, COUNTER_SM, false));  // TX FIFO DREQ
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

    dma_channel_configure(
        dma_channel, &c,
        (void*)&DISCIPLINE_PIO->rxf[PPS_CAPTURE_SM],  // Write to SM1 RX FIFO
        (void*)&DISCIPLINE_PIO->txf[COUNTER_SM],      // Read from SM0 TX FIFO
        0xFFFFFFFF,                                   // Transfer count (infinite)
        true                                           // Start immediately
    );

    printf("  DMA configured (chan %d, SM0 → SM1)\n", dma_channel);

    // Set up PIO IRQ handler for PPS capture
    uint pio_irq = PIO0_IRQ_0;
    irq_set_exclusive_handler(pio_irq, discipline_pps_irq_handler);
    irq_set_enabled(pio_irq, true);
    pio_set_irq0_source_enabled(DISCIPLINE_PIO, pis_interrupt0, true);

    printf("  IRQ handler installed\n");

    // Mark system as running
    core1_stats.discipline_running = true;

    printf("GPS discipline ready (4ns resolution, software correction)\n");
    return true;
}

/**
 * Generate 100 PPS output (called from main loop)
 */
void discipline_generate_100pps(void) {
    if (!output_100pps_enabled) {
        return;
    }

    uint64_t now_us = time_us_64();

    // Calculate time since last pulse
    uint64_t time_since_last_pulse_us = now_us - last_pulse_time_us;

    // Only generate pulse if at least 9.5ms has passed (safety margin)
    // This ensures we never generate duplicate pulses due to race conditions
    if (time_since_last_pulse_us < 9500) {
        return;  // Too soon for next pulse
    }

    // Calculate elapsed time since base (start of second)
    int64_t elapsed_since_base = (int64_t)(now_us - output_base_time_us);

    // Determine which pulse number we should be at
    // Each pulse is every 10ms (10,000 μs)
    int32_t expected_pulse_num = elapsed_since_base / 10000;

    // Clamp to valid range [0, 99]
    if (expected_pulse_num < 0) expected_pulse_num = 0;
    if (expected_pulse_num > 99) {
        // Past end of second - wait for GPS PPS to reset base time
        return;
    }

    // Calculate exact time for this pulse
    uint64_t pulse_time_us = output_base_time_us + (expected_pulse_num * 10000);

    // Check if we've already passed this pulse time
    int64_t time_error = (int64_t)(now_us - pulse_time_us);

    // Generate pulse if:
    // 1. We're on-time (within ±500μs), OR
    // 2. We're late but not TOO late (< 5ms late)
    // This prevents skipping pulses due to polling delays
    if (time_error >= -500 && time_error <= 5000) {
        // Generate pulse (100μs width - easier for scope to see)
        gpio_put(LED_OUTPUT_PIN, 1);
        busy_wait_us(100);  // 100μs pulse width (was 10μs)
        gpio_put(LED_OUTPUT_PIN, 0);

        // Record this pulse time
        last_pulse_time_us = now_us;
    }
}

/**
 * Update discipline stats (optional - called from main loop for status publishing)
 */
void discipline_update_stats(void) {
    // Update continuous time for PTP timestamping
    uint64_t disciplined_us = get_disciplined_time_us();
    core1_stats.continuous_time_ns = disciplined_us * 1000ULL;
    core1_stats.disciplined_time_ns = (disciplined_us % 1000000) * 1000ULL;
}

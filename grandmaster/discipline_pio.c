/**
 * PIO-based Clock Discipline
 *
 * Uses PIO state machine + DMA for hardware-accelerated time tracking
 * No tight loops, no Core 1 - everything is interrupt/DMA driven
 *
 * Architecture:
 * - PIO SM: 10 MHz counter (100ns resolution) counting down from 0xFFFFFFFF
 * - DMA: Continuously broadcasts counter value to shared memory
 * - GPS PPS IRQ: Captures counter, runs PI controller once per second
 * - Hardware alarms: Generate 100 PPS output pulses
 */

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "gps.h"
#include "shared_state.h"
#include "../common/discipline.h"
#include "discipline_counter.pio.h"

// PIO configuration
#define DISCIPLINE_PIO pio1
#define DISCIPLINE_SM 0

// LED output pin (same as in main.c)
#define LED_OUTPUT_PIN 3

// GPS PPS interval (1 Hz = 1,000,000,000 ns = 10,000,000 ticks @ 10 MHz)
#define GPS_PPS_INTERVAL_TICKS 10000000ULL
#define GPS_PPS_INTERVAL_NS 1000000000ULL

// 100 PPS output interval (10 ms = 10,000,000 ns = 100,000 ticks @ 10 MHz)
#define OUTPUT_PPS_INTERVAL_TICKS 100000ULL

// Shared counter value (continuously updated by DMA)
static volatile uint32_t discipline_counter = 0;

// DMA channel
static int dma_channel = -1;

// Disciplined clock instance
static disciplined_clock_t disciplined_clock;

// Shared state for reporting
core1_stats_t core1_stats = {0};

// Previous counter value for calculating elapsed ticks
static uint32_t prev_counter_value = 0;
static uint64_t accumulated_ticks = 0;

// Hardware alarm for 100 PPS output
static alarm_id_t output_alarm_id = 0;
static uint64_t next_output_time_us = 0;
static bool output_enabled = false;

/**
 * Hardware alarm callback for 100 PPS output
 */
static int64_t output_alarm_callback(alarm_id_t id, void *user_data) {
    // Generate short pulse
    gpio_put(LED_OUTPUT_PIN, 1);
    busy_wait_us(10);  // 10μs pulse width
    gpio_put(LED_OUTPUT_PIN, 0);

    // Schedule next pulse (10ms later)
    next_output_time_us += 10000;  // 10ms = 10,000μs
    return next_output_time_us - time_us_64();  // Return relative delay
}

/**
 * Initialize PIO-based discipline system
 */
bool discipline_pio_init(void) {
    // Initialize disciplined clock with PI controller gains
    discipline_init(&disciplined_clock, 0.1, 0.001);

    // Load PIO program
    uint offset = pio_add_program(DISCIPLINE_PIO, &discipline_counter_program);

    // Initialize PIO state machine
    discipline_counter_program_init(DISCIPLINE_PIO, DISCIPLINE_SM, offset);

    // Claim DMA channel
    dma_channel = dma_claim_unused_channel(true);

    // Configure DMA: PIO FIFO → shared memory (continuously)
    dma_channel_config c = dma_channel_get_default_config(dma_channel);
    channel_config_set_read_increment(&c, false);    // Always read from FIFO
    channel_config_set_write_increment(&c, false);   // Always write to same memory
    channel_config_set_dreq(&c, pio_get_dreq(DISCIPLINE_PIO, DISCIPLINE_SM, false));  // RX DREQ
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

    dma_channel_configure(
        dma_channel, &c,
        (void*)&discipline_counter,                    // Write to shared counter
        &DISCIPLINE_PIO->rxf[DISCIPLINE_SM],          // Read from PIO RX FIFO
        0xFFFFFFFF,                                   // Transfer count (infinite)
        true                                          // Start immediately
    );

    // Initialize counter snapshot
    prev_counter_value = discipline_counter;

    // Mark system as running
    core1_stats.discipline_running = true;

    return true;
}

/**
 * GPS PPS callback - called from gps.c IRQ handler
 * Runs PI controller and updates discipline
 */
void discipline_pio_gps_pps_callback(uint64_t pps_timestamp_us, bool valid) {
    // Capture counter value (atomically read from DMA-updated memory)
    uint32_t counter_snapshot = discipline_counter;

    if (!valid) {
        return;  // Ignore invalid PPS
    }

    // Calculate elapsed ticks since last PPS (handle counter wrap)
    uint32_t elapsed_ticks;
    if (counter_snapshot > prev_counter_value) {
        // Counter wrapped (counting down: was small, now large)
        elapsed_ticks = prev_counter_value + (0xFFFFFFFF - counter_snapshot) + 1;
    } else {
        // Normal case: prev > current (counting down)
        elapsed_ticks = prev_counter_value - counter_snapshot;
    }

    // Update accumulated ticks
    accumulated_ticks += elapsed_ticks;

    // Calculate phase error (expected 10M ticks per second)
    int64_t expected_ticks = GPS_PPS_INTERVAL_TICKS;
    int64_t phase_error_ticks = (int64_t)elapsed_ticks - expected_ticks;
    int64_t phase_error_ns = (phase_error_ticks * 100);  // Convert 100ns ticks to ns

    // Update discipline PI controller
    discipline_update_reference(
        &disciplined_clock,
        pps_timestamp_us,
        GPS_PPS_INTERVAL_NS,
        &core1_stats.lock_event_count,
        &core1_stats.unlock_event_count
    );

    // Update shared statistics
    core1_stats.pps_count++;
    core1_stats.phase_error_ns = phase_error_ns;
    core1_stats.freq_offset_ppb = disciplined_clock.frequency_offset_ppb;
    core1_stats.locked = disciplined_clock.locked;
    core1_stats.last_update_us = time_us_64();

    // Apply frequency correction to PIO clock divider
    float corrected_div = 25.0f * (1.0f + (disciplined_clock.frequency_offset_ppb / 1e9f));
    pio_sm_set_clkdiv(DISCIPLINE_PIO, DISCIPLINE_SM, corrected_div);

    // Update previous counter for next iteration
    prev_counter_value = counter_snapshot;

    // Enable 100 PPS output after first valid PPS
    if (!output_enabled) {
        output_enabled = true;
        next_output_time_us = time_us_64() + 10000;  // Start 10ms from now
        output_alarm_id = add_alarm_at(next_output_time_us, output_alarm_callback, NULL, true);
        core1_stats.first_pps_received = true;
    }
}

/**
 * Update disciplined clock time (called periodically)
 * Much less critical now - just for publishing current time
 */
void discipline_pio_update(void) {
    // Read current counter (DMA keeps it updated)
    uint32_t current_counter = discipline_counter;

    // Calculate elapsed ticks
    uint32_t elapsed_ticks;
    if (current_counter > prev_counter_value) {
        elapsed_ticks = prev_counter_value + (0xFFFFFFFF - current_counter) + 1;
    } else {
        elapsed_ticks = prev_counter_value - current_counter;
    }

    // Update disciplined time (in 100ns ticks)
    uint64_t elapsed_ns = elapsed_ticks * 100ULL;

    // Publish to shared stats (for PTP timestamping)
    core1_stats.disciplined_time_ns = elapsed_ns % GPS_PPS_INTERVAL_NS;
    core1_stats.continuous_time_ns = accumulated_ticks * 100ULL;  // Total time in ns
}

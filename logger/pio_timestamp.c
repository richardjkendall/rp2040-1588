/**
 * PIO Timestamp Capture Implementation
 */

#include "pio_timestamp.h"
#include "shared_state.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"
#include "timestamp_capture.pio.h"
#include <stdio.h>

// PIO instance (use PIO0 to have all 4 state machines)
#define PPS_PIO pio0

// GPIO pin definitions
#define GPS_PPS_PIN      16  // GPS 1 PPS input
#define GM_PPS_PIN       17  // Grandmaster 100 PPS input
#define SLAVE_PPS_PIN    18  // Slave 100 PPS input

// State machine assignments
static uint sm_timer;   // SM0: Timer
static uint sm_gps;     // SM1: GPS PPS capture
static uint sm_gm;      // SM2: GM PPS capture
static uint sm_slave;   // SM3: Slave PPS capture

// PIO clock frequency (typically 125 MHz)
static uint32_t pio_clock_hz;

// Debug counters
static volatile uint32_t irq_call_count = 0;
static volatile uint32_t gps_irq_count = 0;  // Specific count for GPS PPS detections

// Shared memory location for timer value
static volatile uint32_t timer_value = 0;

// DMA reload value (must be persistent, not compound literal)
static const uint32_t dma_reload_value = 1;

// DMA channels (ping-pong pairs for continuous operation)
static int dma_timer_to_mem;         // Timer FIFO → Memory
static int dma_timer_to_mem_ctrl;    // Control channel to reload timer DMA
static int dma_mem_to_gps;           // Memory → GPS input FIFO
static int dma_mem_to_gps_ctrl;      // Control channel to reload GPS DMA
static int dma_mem_to_gm;            // Memory → GM input FIFO
static int dma_mem_to_gm_ctrl;       // Control channel to reload GM DMA
static int dma_mem_to_slave;         // Memory → Slave input FIFO
static int dma_mem_to_slave_ctrl;    // Control channel to reload Slave DMA

/**
 * PIO IRQ handler - reads timestamps from FIFO
 * Marked __time_critical_func for minimal latency
 * Timestamps captured in PIO hardware at exact edge moment (zero latency)
 */
static void __time_critical_func(pps_irq_handler)(void) {
    irq_call_count++;

    // GPS PPS - check FIFO
    if (!pio_sm_is_rx_fifo_empty(PPS_PIO, sm_gps)) {
        uint32_t timestamp = pio_sm_get(PPS_PIO, sm_gps);
        timestamp_data.gps_pps_timestamp = timestamp;
        timestamp_data.gps_pps_count++;
        timestamp_data.gps_pps_valid = true;
        gps_irq_count++;  // Debug: count GPS PPS IRQs specifically
    }

    // GM PPS - check FIFO
    if (!pio_sm_is_rx_fifo_empty(PPS_PIO, sm_gm)) {
        uint32_t timestamp = pio_sm_get(PPS_PIO, sm_gm);
        timestamp_data.gm_pps_timestamp = timestamp;
        timestamp_data.gm_pps_count++;
        timestamp_data.gm_pps_valid = true;
    }

    // Slave PPS - check FIFO
    if (!pio_sm_is_rx_fifo_empty(PPS_PIO, sm_slave)) {
        uint32_t timestamp = pio_sm_get(PPS_PIO, sm_slave);
        timestamp_data.slave_pps_timestamp = timestamp;
        timestamp_data.slave_pps_count++;
        timestamp_data.slave_pps_valid = true;
    }
}

uint32_t pio_timestamp_get_irq_count(void) {
    return irq_call_count;
}

uint32_t pio_timestamp_get_timer_value(void) {
    return timer_value;
}

uint32_t pio_timestamp_get_gps_irq_count(void) {
    return gps_irq_count;
}

void pio_timestamp_init(void) {
    printf("Initializing PIO timestamp capture...\n");

    // Get system clock frequency
    pio_clock_hz = clock_get_hz(clk_sys);
    printf("  System clock: %lu Hz\n", (unsigned long)pio_clock_hz);

    // Claim 4 state machines from PIO0
    sm_timer = pio_claim_unused_sm(PPS_PIO, true);   // Should get SM0
    sm_gps = pio_claim_unused_sm(PPS_PIO, true);     // Should get SM1
    sm_gm = pio_claim_unused_sm(PPS_PIO, true);      // Should get SM2
    sm_slave = pio_claim_unused_sm(PPS_PIO, true);   // Should get SM3

    printf("  State machines claimed: Timer=SM%d, GPS=SM%d, GM=SM%d, Slave=SM%d\n",
           sm_timer, sm_gps, sm_gm, sm_slave);

    // Load PIO programs into instruction memory
    uint offset_timer = pio_add_program(PPS_PIO, &timer_program);
    uint offset_gps = pio_add_program(PPS_PIO, &gps_pps_capture_program);
    uint offset_gm = pio_add_program(PPS_PIO, &gm_pps_capture_program);
    uint offset_slave = pio_add_program(PPS_PIO, &slave_pps_capture_program);

    // --- Configure Timer SM (SM0) ---
    pio_sm_config c_timer = timer_program_get_default_config(offset_timer);
    sm_config_set_clkdiv(&c_timer, 1.0f);  // Full speed (125 MHz)
    sm_config_set_in_shift(&c_timer, false, false, 32);  // ISR config for push
    pio_sm_init(PPS_PIO, sm_timer, offset_timer, &c_timer);
    // Initialize X = 0xFFFFFFFF
    pio_sm_put(PPS_PIO, sm_timer, 0xFFFFFFFF);
    pio_sm_exec(PPS_PIO, sm_timer, pio_encode_pull(false, false));
    pio_sm_exec(PPS_PIO, sm_timer, pio_encode_mov(pio_x, pio_osr));

    // --- Configure GPS Capture SM (SM1) ---
    pio_sm_config c_gps = gps_pps_capture_program_get_default_config(offset_gps);
    sm_config_set_in_pins(&c_gps, GPS_PPS_PIN);
    pio_sm_set_consecutive_pindirs(PPS_PIO, sm_gps, GPS_PPS_PIN, 1, false);  // Input
    pio_gpio_init(PPS_PIO, GPS_PPS_PIN);
    sm_config_set_out_shift(&c_gps, false, false, 32);  // OSR config for pull
    sm_config_set_in_shift(&c_gps, false, false, 32);   // ISR config for push
    pio_sm_init(PPS_PIO, sm_gps, offset_gps, &c_gps);

    // --- Configure GM Capture SM (SM2) ---
    pio_sm_config c_gm = gm_pps_capture_program_get_default_config(offset_gm);
    sm_config_set_in_pins(&c_gm, GM_PPS_PIN);
    pio_sm_set_consecutive_pindirs(PPS_PIO, sm_gm, GM_PPS_PIN, 1, false);  // Input
    pio_gpio_init(PPS_PIO, GM_PPS_PIN);
    sm_config_set_out_shift(&c_gm, false, false, 32);  // OSR config for pull
    sm_config_set_in_shift(&c_gm, false, false, 32);   // ISR config for push
    pio_sm_init(PPS_PIO, sm_gm, offset_gm, &c_gm);

    // --- Configure Slave Capture SM (SM3) ---
    pio_sm_config c_slave = slave_pps_capture_program_get_default_config(offset_slave);
    sm_config_set_in_pins(&c_slave, SLAVE_PPS_PIN);
    pio_sm_set_consecutive_pindirs(PPS_PIO, sm_slave, SLAVE_PPS_PIN, 1, false);  // Input
    pio_gpio_init(PPS_PIO, SLAVE_PPS_PIN);
    sm_config_set_out_shift(&c_slave, false, false, 32);  // OSR config for pull
    sm_config_set_in_shift(&c_slave, false, false, 32);   // ISR config for push
    pio_sm_init(PPS_PIO, sm_slave, offset_slave, &c_slave);

    // --- Setup Ping-Pong DMA Channels ---
    printf("  Setting up ping-pong DMA channels...\n");

    // === Timer FIFO → Memory (ping-pong pair) ===
    dma_timer_to_mem = dma_claim_unused_channel(true);
    dma_timer_to_mem_ctrl = dma_claim_unused_channel(true);

    // Data channel: Timer FIFO → Memory
    dma_channel_config dc_timer = dma_channel_get_default_config(dma_timer_to_mem);
    channel_config_set_transfer_data_size(&dc_timer, DMA_SIZE_32);
    channel_config_set_read_increment(&dc_timer, false);
    channel_config_set_write_increment(&dc_timer, false);
    channel_config_set_dreq(&dc_timer, pio_get_dreq(PPS_PIO, sm_timer, false));
    channel_config_set_chain_to(&dc_timer, dma_timer_to_mem_ctrl);
    dma_channel_configure(dma_timer_to_mem, &dc_timer,
        (void*)&timer_value,
        &PPS_PIO->rxf[sm_timer],
        1,
        false);

    // Control channel: Reload data channel
    dma_channel_config c_timer_ctrl = dma_channel_get_default_config(dma_timer_to_mem_ctrl);
    channel_config_set_transfer_data_size(&c_timer_ctrl, DMA_SIZE_32);
    channel_config_set_read_increment(&c_timer_ctrl, false);
    channel_config_set_write_increment(&c_timer_ctrl, false);
    channel_config_set_chain_to(&c_timer_ctrl, dma_timer_to_mem);
    dma_channel_configure(dma_timer_to_mem_ctrl, &c_timer_ctrl,
        &dma_hw->ch[dma_timer_to_mem].al3_transfer_count,
        &dma_reload_value,
        1,
        false);

    // === Memory → GPS FIFO (ping-pong pair) ===
    dma_mem_to_gps = dma_claim_unused_channel(true);
    dma_mem_to_gps_ctrl = dma_claim_unused_channel(true);

    dma_channel_config dc_gps = dma_channel_get_default_config(dma_mem_to_gps);
    channel_config_set_transfer_data_size(&dc_gps, DMA_SIZE_32);
    channel_config_set_read_increment(&dc_gps, false);
    channel_config_set_write_increment(&dc_gps, false);
    channel_config_set_dreq(&dc_gps, pio_get_dreq(PPS_PIO, sm_gps, true)); // Paced by GPS TX FIFO
    channel_config_set_chain_to(&dc_gps, dma_mem_to_gps_ctrl);
    dma_channel_configure(dma_mem_to_gps, &dc_gps,
        &PPS_PIO->txf[sm_gps],
        (void*)&timer_value,
        1,
        false);

    dma_channel_config c_gps_ctrl = dma_channel_get_default_config(dma_mem_to_gps_ctrl);
    channel_config_set_transfer_data_size(&c_gps_ctrl, DMA_SIZE_32);
    channel_config_set_read_increment(&c_gps_ctrl, false);
    channel_config_set_write_increment(&c_gps_ctrl, false);
    channel_config_set_chain_to(&c_gps_ctrl, dma_mem_to_gps);
    dma_channel_configure(dma_mem_to_gps_ctrl, &c_gps_ctrl,
        &dma_hw->ch[dma_mem_to_gps].al3_transfer_count,
        &dma_reload_value,
        1,
        false);

    // === Memory → GM FIFO (ping-pong pair) ===
    dma_mem_to_gm = dma_claim_unused_channel(true);
    dma_mem_to_gm_ctrl = dma_claim_unused_channel(true);

    dma_channel_config dc_gm = dma_channel_get_default_config(dma_mem_to_gm);
    channel_config_set_transfer_data_size(&dc_gm, DMA_SIZE_32);
    channel_config_set_read_increment(&dc_gm, false);
    channel_config_set_write_increment(&dc_gm, false);
    channel_config_set_dreq(&dc_gm, pio_get_dreq(PPS_PIO, sm_gm, true)); // Paced by GM TX FIFO
    channel_config_set_chain_to(&dc_gm, dma_mem_to_gm_ctrl);
    dma_channel_configure(dma_mem_to_gm, &dc_gm,
        &PPS_PIO->txf[sm_gm],
        (void*)&timer_value,
        1,
        false);

    dma_channel_config c_gm_ctrl = dma_channel_get_default_config(dma_mem_to_gm_ctrl);
    channel_config_set_transfer_data_size(&c_gm_ctrl, DMA_SIZE_32);
    channel_config_set_read_increment(&c_gm_ctrl, false);
    channel_config_set_write_increment(&c_gm_ctrl, false);
    channel_config_set_chain_to(&c_gm_ctrl, dma_mem_to_gm);
    dma_channel_configure(dma_mem_to_gm_ctrl, &c_gm_ctrl,
        &dma_hw->ch[dma_mem_to_gm].al3_transfer_count,
        &dma_reload_value,
        1,
        false);

    // === Memory → Slave FIFO (ping-pong pair) ===
    dma_mem_to_slave = dma_claim_unused_channel(true);
    dma_mem_to_slave_ctrl = dma_claim_unused_channel(true);

    dma_channel_config dc_slave = dma_channel_get_default_config(dma_mem_to_slave);
    channel_config_set_transfer_data_size(&dc_slave, DMA_SIZE_32);
    channel_config_set_read_increment(&dc_slave, false);
    channel_config_set_write_increment(&dc_slave, false);
    channel_config_set_dreq(&dc_slave, pio_get_dreq(PPS_PIO, sm_slave, true)); // Paced by Slave TX FIFO
    channel_config_set_chain_to(&dc_slave, dma_mem_to_slave_ctrl);
    dma_channel_configure(dma_mem_to_slave, &dc_slave,
        &PPS_PIO->txf[sm_slave],
        (void*)&timer_value,
        1,
        false);

    dma_channel_config c_slave_ctrl = dma_channel_get_default_config(dma_mem_to_slave_ctrl);
    channel_config_set_transfer_data_size(&c_slave_ctrl, DMA_SIZE_32);
    channel_config_set_read_increment(&c_slave_ctrl, false);
    channel_config_set_write_increment(&c_slave_ctrl, false);
    channel_config_set_chain_to(&c_slave_ctrl, dma_mem_to_slave);
    dma_channel_configure(dma_mem_to_slave_ctrl, &c_slave_ctrl,
        &dma_hw->ch[dma_mem_to_slave].al3_transfer_count,
        &dma_reload_value,
        1,
        false);

    printf("  Ping-pong DMA pairs configured\n");

    // --- Setup Interrupts ---
    // Clear any pending interrupts
    pio_interrupt_clear(PPS_PIO, 0);
    pio_interrupt_clear(PPS_PIO, 1);

    // Enable FIFO not empty interrupts for edge detection SMs (output)
    pio_set_irq0_source_enabled(PPS_PIO, pis_sm1_rx_fifo_not_empty, true);  // GPS
    pio_set_irq0_source_enabled(PPS_PIO, pis_sm2_rx_fifo_not_empty, true);  // GM
    pio_set_irq0_source_enabled(PPS_PIO, pis_sm3_rx_fifo_not_empty, true);  // Slave

    // Set up CPU IRQ handler
    irq_set_exclusive_handler(PIO0_IRQ_0, pps_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);

    // --- Enable all 4 SMs simultaneously FIRST ---
    // (Must start SMs before DMAs so Timer SM produces data for Timer DMA to read)
    printf("  Starting all 4 PIO state machines...\n");
    pio_enable_sm_mask_in_sync(PPS_PIO,
        (1u << sm_timer) | (1u << sm_gps) | (1u << sm_gm) | (1u << sm_slave));

    // Small delay to let timer SM produce first value
    sleep_us(10);

    // --- Start DMA Channels ---
    printf("  Starting DMA channels...\n");
    dma_channel_start(dma_timer_to_mem);
    dma_channel_start(dma_mem_to_gps);
    dma_channel_start(dma_mem_to_gm);
    dma_channel_start(dma_mem_to_slave);

    printf("PIO timestamp capture ready:\n");
    printf("  Resolution: 8 ns per tick @ %lu Hz\n", (unsigned long)pio_clock_hz);
    printf("  GPS PPS input: GPIO%d\n", GPS_PPS_PIN);
    printf("  GM PPS input: GPIO%d\n", GM_PPS_PIN);
    printf("  Slave PPS input: GPIO%d\n", SLAVE_PPS_PIN);
}

uint32_t pio_timestamp_get_freq_hz(void) {
    return pio_clock_hz;  // 125 MHz PIO clock
}

uint32_t pio_timestamp_get_ns_per_tick(void) {
    // PIO clock is 125 MHz (8 nanoseconds per tick)
    return (uint32_t)(1000000000ULL / pio_clock_hz);
}

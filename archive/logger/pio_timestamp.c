/**
 * PIO Timestamp Capture Implementation (1us resolution)
 *
 * Configures PIO state machines to detect PPS edges, and uses DMA
 * with CPU IRQ handlers to timestamp these events with 1us resolution.
 */

#include "pio_timestamp.h"
#include "shared_state.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"
#include "pico/time.h" // For absolute_time_t and get_absolute_time()
#include "timestamp_capture.pio.h" // The simplified PIO program
#include <stdio.h>

// --- Peripheral and Pin Configuration ---

#define PPS_PIO pio0

// GPIO pin definitions
#define GPS_PPS_PIN      16
#define GM_PPS_PIN       17
#define SLAVE_PPS_PIN    18

// --- State Machine Assignments ---
static uint sm_gps;
static uint sm_gm;
static uint sm_slave;

// --- DMA Channel Assignments ---
static int dma_chan_gps;
static int dma_chan_gm;
static int dma_chan_slave;

// --- DMA Buffers (for continuous, single-word transfers) ---
// Each DMA channel drains a PIO RX FIFO. For a 1us timestamp, we only care that
// the DMA triggers, not the value itself. A small buffer per channel suffices.
#define DMA_BUFFER_DEPTH 1
static volatile uint32_t gps_dma_buffer[DMA_BUFFER_DEPTH];
static volatile uint32_t gm_dma_buffer[DMA_BUFFER_DEPTH];
static volatile uint32_t slave_dma_buffer[DMA_BUFFER_DEPTH];

// --- Module State ---
static uint32_t pio_clock_hz;
static uint pio_program_offset;

// --- Global Shared State Instances (defined in timestamp_processor.c) ---
extern timestamp_data_t timestamp_data;

// --- DMA IRQ Handlers ---

/**
 * @brief Common IRQ handler for all DMA channels.
 * It identifies the channel and processes the timestamp.
 */
static void __time_critical_func(dma_common_irq_handler)(void) {
    absolute_time_t now = get_absolute_time(); // Timestamp immediately

    if (dma_irqn_get_channel_status(DMA_IRQ_0, dma_chan_gps)) {
        dma_hw->ints0 = 1u << dma_chan_gps; // Clear the interrupt request for this channel
        timestamp_data.gps_abs_time = now;
        timestamp_data.gps_raw_pio_val = gps_dma_buffer[0];
        timestamp_data.gps_pps_valid = true;
        timestamp_data.gps_pps_count++;
        dma_channel_set_read_addr(dma_chan_gps, &PPS_PIO->rxf[sm_gps], true);
    }
    if (dma_irqn_get_channel_status(DMA_IRQ_0, dma_chan_gm)) {
        dma_hw->ints0 = 1u << dma_chan_gm; // Clear the interrupt request for this channel
        timestamp_data.gm_abs_time = now;
        timestamp_data.gm_raw_pio_val = gm_dma_buffer[0];
        timestamp_data.gm_pps_valid = true;
        timestamp_data.gm_pps_count++;
        dma_channel_set_read_addr(dma_chan_gm, &PPS_PIO->rxf[sm_gm], true);
    }
    if (dma_irqn_get_channel_status(DMA_IRQ_0, dma_chan_slave)) {
        dma_hw->ints0 = 1u << dma_chan_slave; // Clear the interrupt request for this channel
        timestamp_data.slave_abs_time = now;
        timestamp_data.slave_raw_pio_val = slave_dma_buffer[0];
        timestamp_data.slave_pps_valid = true;
        timestamp_data.slave_pps_count++;
        dma_channel_set_read_addr(dma_chan_slave, &PPS_PIO->rxf[sm_slave], true);
    }
}


// --- Initialization Helpers ---

/**
 * @brief Configures a single PIO state machine and its associated DMA channel.
 */
static void pio_sm_dma_configure(PIO pio, uint sm_index, uint pps_pin,
                                 int dma_channel, volatile uint32_t* dma_buffer) {
    pio_sm_config c = pps_capture_event_program_get_default_config(pio_program_offset);

    // Set the input pin
    pio_gpio_init(pio, pps_pin);
    sm_config_set_in_pins(&c, pps_pin);
    pio_sm_set_consecutive_pindirs(pio, sm_index, pps_pin, 1, false);

    // Set clock divider to 1 for full speed (8ns resolution)
    sm_config_set_clkdiv(&c, 1.0f);

    // Load the configuration into the state machine
    pio_sm_init(pio, sm_index, pio_program_offset, &c);
    pio_sm_set_enabled(pio, sm_index, false); // Keep disabled until all setup is done

    // Configure DMA channel
    dma_channel_config dma_cfg = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_cfg, false); // Read from PIO FIFO
    channel_config_set_write_increment(&dma_cfg, false); // Write to a single location

    // DREQ from PIO RX FIFO to pace transfers, specifically for this SM
    channel_config_set_dreq(&dma_cfg, pio_get_dreq(pio, sm_index, false));

    // Transfer one word to the buffer, then trigger interrupt
    dma_channel_configure(
        dma_channel,
        &dma_cfg,
        dma_buffer,               // Write address (single word buffer)
        &pio->rxf[sm_index],      // Read address (PIO RX FIFO)
        1,                        // Transfer count (one word)
        false                     // Don't start immediately
    );

    // Enable DMA IRQ for this channel
    dma_channel_set_irq0_enabled(dma_channel, true);
}


// --- Public Functions ---

void pio_timestamp_init(void) {
    printf("Initializing PIO Timestamp Capture (1us resolution)...\n");

    pio_clock_hz = clock_get_hz(clk_sys);
    printf("  System clock: %lu Hz\n", (unsigned long)pio_clock_hz);

    // 1. Load the PIO program into instruction memory
    pio_program_offset = pio_add_program(PPS_PIO, &pps_capture_event_program);

    // 2. Claim state machines
    sm_gps = pio_claim_unused_sm(PPS_PIO, true);
    sm_gm = pio_claim_unused_sm(PPS_PIO, true);
    sm_slave = pio_claim_unused_sm(PPS_PIO, true);
    printf("  State machines claimed: GPS=SM%d, GM=SM%d, Slave=SM%d\n", sm_gps, sm_gm, sm_slave);

    // 3. Claim DMA channels
    dma_chan_gps = dma_claim_unused_channel(true);
    dma_chan_gm = dma_claim_unused_channel(true);
    dma_chan_slave = dma_claim_unused_channel(true);
    printf("  DMA channels claimed: GPS=%d, GM=%d, Slave=%d\n", dma_chan_gps, dma_chan_gm, dma_chan_slave);

    // 4. Configure SMs and their DMAs
    pio_sm_dma_configure(PPS_PIO, sm_gps, GPS_PPS_PIN, dma_chan_gps, gps_dma_buffer);
    pio_sm_dma_configure(PPS_PIO, sm_gm, GM_PPS_PIN, dma_chan_gm, gm_dma_buffer);
    pio_sm_dma_configure(PPS_PIO, sm_slave, SLAVE_PPS_PIN, dma_chan_slave, slave_dma_buffer);
    printf("  SMs and DMAs configured\n");

    // 5. Set up a single IRQ handler for all DMA channels
    irq_set_exclusive_handler(DMA_IRQ_0, dma_common_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    printf("  DMA IRQ handler configured\n");

    // 6. Start DMA channels
    dma_channel_start(dma_chan_gps);
    dma_channel_start(dma_chan_gm);
    dma_channel_start(dma_chan_slave);
    printf("  DMA channels started\n");

    // 7. Start PIO state machines
    pio_sm_set_enabled(PPS_PIO, sm_gps, true);
    pio_sm_set_enabled(PPS_PIO, sm_gm, true);
    pio_sm_set_enabled(PPS_PIO, sm_slave, true);
    printf("  PIO State Machines started.\n");

    printf("PIO timestamp capture ready.\n");
}

uint32_t pio_timestamp_get_freq_hz(void) {
    return pio_clock_hz;
}

uint32_t pio_timestamp_get_ns_per_tick(void) {
    return (uint32_t)(1000000000ULL / pio_clock_hz); // Still 8ns for PIO clock
}
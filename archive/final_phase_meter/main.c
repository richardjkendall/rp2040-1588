/**
 * Final Phase Meter (v9 - 10Hz Summarized Output)
 *
 * This version changes the output to a 10Hz summary, providing an
 * aggregated view (avg, min, max, p-p) of the 100Hz measurements.
 */

#include <stdio.h>
#include <string.h>
#include <limits.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "frequency_counter.pio.h"
#include "pulse_generator.pio.h"
#include "phase_diff.pio.h"

// --- Pin Configuration ---
#define GPS_1PPS_PIN                    16
#define GM_100PPS_IN_PIN                17
#define DISCIPLINED_100PPS_OUT_PIN      20
#define DISCIPLINED_100PPS_IN_PIN       21 // Physically wire GPIO20 to GPIO21

// --- PIO Configuration ---
#define PIO_INSTANCE pio0
#define MEASUREMENT_IRQ PIO0_IRQ_0

// --- PLL/Measurement Configuration ---
#define IDEAL_SYS_CLOCKS_PER_SECOND 125000000
#define PULSE_HIGH_CYCLES           1248 // approx 10us pulse width
#define MAX_PHASE_TICKS             (IDEAL_SYS_CLOCKS_PER_SECOND / 200) // 5ms
#define SUMMARY_BUFFER_SIZE         10

// --- Shared State & Globals ---
volatile uint32_t g_measured_clocks_per_gps_second = IDEAL_SYS_CLOCKS_PER_SECOND;
critical_section_t g_correction_crit_sec;

// --- Globals for GM Channel ---
volatile int64_t g_gm_phase_buffer[SUMMARY_BUFFER_SIZE];
volatile uint32_t g_gm_buffer_index = 0;
volatile bool g_gm_summary_ready = false;

uint sm_ref_to_gm;
uint sm_gm_to_ref;
uint sm_pulse_generator;
uint sm_freq_counter;

// --- Forward Declarations ---
void core1_synthesis_engine(void);

// =============================================================================
//  Core 1 - The Synthesis Engine (Slave Role)
// =============================================================================

void core1_synthesis_engine(void) {
    // This code is unchanged. Core 1 simply generates pulses on command.
    uint offset = pio_add_program(PIO_INSTANCE, &pulse_generator_program);
    sm_pulse_generator = pio_claim_unused_sm(PIO_INSTANCE, true);
    pio_sm_config c = pulse_generator_program_get_default_config(offset);
    pio_gpio_init(PIO_INSTANCE, DISCIPLINED_100PPS_OUT_PIN);
    pio_sm_set_consecutive_pindirs(PIO_INSTANCE, sm_pulse_generator, DISCIPLINED_100PPS_OUT_PIN, 1, true);
    sm_config_set_set_pins(&c, DISCIPLINED_100PPS_OUT_PIN, 1);
    pio_sm_init(PIO_INSTANCE, sm_pulse_generator, offset, &c);
    pio_sm_set_enabled(PIO_INSTANCE, sm_pulse_generator, true);

    while (true) {
        multicore_fifo_pop_blocking();
        uint32_t local_measured_clocks;
        critical_section_enter_blocking(&g_correction_crit_sec);
        local_measured_clocks = g_measured_clocks_per_gps_second;
        critical_section_exit(&g_correction_crit_sec);
        uint32_t total_period_cycles = local_measured_clocks / 100;
        uint32_t low_cycles = total_period_cycles > PULSE_HIGH_CYCLES ? total_period_cycles - PULSE_HIGH_CYCLES : 1;
        pio_sm_put_blocking(PIO_INSTANCE, sm_pulse_generator, PULSE_HIGH_CYCLES);
        pio_sm_put_blocking(PIO_INSTANCE, sm_pulse_generator, low_cycles);
    }
}

// --- Helper function to configure a phase difference measurement SM ---
static void setup_phase_meter_sm(uint sm_index, uint offset, uint start_pin, uint stop_pin) {
    pio_sm_config c = phase_diff_program_get_default_config(offset);
    sm_config_set_in_pins(&c, start_pin);
    sm_config_set_jmp_pin(&c, stop_pin);
    pio_gpio_init(PIO_INSTANCE, start_pin);
    pio_gpio_init(PIO_INSTANCE, stop_pin);
    pio_sm_set_consecutive_pindirs(PIO_INSTANCE, sm_index, start_pin, 1, false);
    pio_sm_set_consecutive_pindirs(PIO_INSTANCE, sm_index, stop_pin, 1, false);
    pio_sm_init(PIO_INSTANCE, sm_index, offset, &c);
}

// =============================================================================
//  Core 0 - The Master Controller
// =============================================================================

bool __not_in_flash_func(master_pacer_isr)(struct repeating_timer *t) {
    multicore_fifo_push_blocking(1);
    return true;
}

static void __not_in_flash_func(pio_measurement_irq_handler)() {
    bool processed = false;
    int64_t phase_ns = 0;
    
    // Check sm_ref_to_gm FIFO
    if (!pio_sm_is_rx_fifo_empty(PIO_INSTANCE, sm_ref_to_gm)) {
        uint32_t val = pio_sm_get(PIO_INSTANCE, sm_ref_to_gm);
        uint32_t ticks = 0xFFFFFFFF - val;
        if (ticks < MAX_PHASE_TICKS) {
            phase_ns = (int64_t)ticks * 8;
            processed = true;
        }
    }
    
    // Check sm_gm_to_ref FIFO
    if (!pio_sm_is_rx_fifo_empty(PIO_INSTANCE, sm_gm_to_ref)) {
        uint32_t val = pio_sm_get(PIO_INSTANCE, sm_gm_to_ref);
        uint32_t ticks = 0xFFFFFFFF - val;
        if (ticks < MAX_PHASE_TICKS && !processed) {
            phase_ns = -((int64_t)ticks * 8);
            processed = true;
        }
    }

    // If we got a valid measurement, add it to the buffer
    if (processed) {
        if (g_gm_buffer_index < SUMMARY_BUFFER_SIZE) {
            g_gm_phase_buffer[g_gm_buffer_index++] = phase_ns;
        }
        if (g_gm_buffer_index >= SUMMARY_BUFFER_SIZE) {
            g_gm_summary_ready = true;
            g_gm_buffer_index = 0;
        }
    }
    pio_interrupt_clear(PIO_INSTANCE, 0);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    critical_section_init(&g_correction_crit_sec);

    printf("\n=== Final Phase Meter (v9 - 10Hz Summary) ===\n");
    printf("NOTE: Wire GPIO %d (output) to GPIO %d (input).\n", DISCIPLINED_100PPS_OUT_PIN, DISCIPLINED_100PPS_IN_PIN);

    // --- Claim all hardware resources on Core 0 ---
    sm_freq_counter = pio_claim_unused_sm(PIO_INSTANCE, true);
    sm_pulse_generator = pio_claim_unused_sm(PIO_INSTANCE, true);
    sm_ref_to_gm = pio_claim_unused_sm(PIO_INSTANCE, true);
    sm_gm_to_ref = pio_claim_unused_sm(PIO_INSTANCE, true);
    
    uint pulse_gen_offset = pio_add_program(PIO_INSTANCE, &pulse_generator_program);
    uint freq_offset = pio_add_program(PIO_INSTANCE, &frequency_counter_program);
    uint phase_offset = pio_add_program(PIO_INSTANCE, &phase_diff_program);
    
    // Setup and enable all SMs on Core 0
    pio_sm_config c_freq = frequency_counter_program_get_default_config(freq_offset);
    pio_gpio_init(PIO_INSTANCE, GPS_1PPS_PIN);
    sm_config_set_in_pins(&c_freq, GPS_1PPS_PIN);
    sm_config_set_jmp_pin(&c_freq, GPS_1PPS_PIN);
    pio_sm_init(PIO_INSTANCE, sm_freq_counter, freq_offset, &c_freq);

    pio_sm_config c_pulse = pulse_generator_program_get_default_config(pulse_gen_offset);
    pio_gpio_init(PIO_INSTANCE, DISCIPLINED_100PPS_OUT_PIN);
    pio_sm_set_consecutive_pindirs(PIO_INSTANCE, sm_pulse_generator, DISCIPLINED_100PPS_OUT_PIN, 1, true);
    sm_config_set_set_pins(&c_pulse, DISCIPLINED_100PPS_OUT_PIN, 1);
    pio_sm_init(PIO_INSTANCE, sm_pulse_generator, pulse_gen_offset, &c_pulse);

    setup_phase_meter_sm(sm_ref_to_gm, phase_offset, DISCIPLINED_100PPS_IN_PIN, GM_100PPS_IN_PIN);
    setup_phase_meter_sm(sm_gm_to_ref, phase_offset, GM_100PPS_IN_PIN, DISCIPLINED_100PPS_IN_PIN);
    
    multicore_launch_core1(core1_synthesis_engine);
    printf("Launched Synthesis Engine on Core 1.\n");

    irq_set_exclusive_handler(MEASUREMENT_IRQ, pio_measurement_irq_handler);
    irq_set_enabled(MEASUREMENT_IRQ, true);
    pio_set_irq0_source_enabled(PIO_INSTANCE, pis_sm0_rx_fifo_not_empty + sm_ref_to_gm, true);
    pio_set_irq0_source_enabled(PIO_INSTANCE, pis_sm0_rx_fifo_not_empty + sm_gm_to_ref, true);

    pio_set_sm_mask_enabled(PIO_INSTANCE, (1u << sm_freq_counter) | (1u << sm_pulse_generator) | (1u << sm_ref_to_gm) | (1u << sm_gm_to_ref), true);
    
    printf("System Initialized. Starting master pacer...\n\n");
    printf("sample,avg_ns,min_ns,max_ns,peak_to_peak_ns\n");

    struct repeating_timer master_timer;
    add_repeating_timer_us(-10000, master_pacer_isr, NULL, &master_timer);

    uint32_t sample_count = 0;
    int64_t local_phase_buffer[SUMMARY_BUFFER_SIZE];

    while (true) {
        if (g_gm_summary_ready) {
            critical_section_enter_blocking(&g_correction_crit_sec);
            memcpy(local_phase_buffer, (const void*)g_gm_phase_buffer, sizeof(g_gm_phase_buffer));
            g_gm_summary_ready = false; // Consume the flag
            critical_section_exit(&g_correction_crit_sec);

            int64_t sum = 0;
            int64_t min_phase = LLONG_MAX;
            int64_t max_phase = LLONG_MIN;
            for (int i = 0; i < SUMMARY_BUFFER_SIZE; ++i) {
                sum += local_phase_buffer[i];
                if (local_phase_buffer[i] < min_phase) min_phase = local_phase_buffer[i];
                if (local_phase_buffer[i] > max_phase) max_phase = local_phase_buffer[i];
            }
            int64_t avg = sum / SUMMARY_BUFFER_SIZE;
            int64_t peak_to_peak = max_phase - min_phase;

            printf("%lu,%lld,%lld,%lld,%lld\n", sample_count++, avg, min_phase, max_phase, peak_to_peak);
        }

        if (!pio_sm_is_rx_fifo_empty(PIO_INSTANCE, sm_freq_counter)) {
            uint32_t raw_value = pio_sm_get(PIO_INSTANCE, sm_freq_counter);
            uint32_t measured_clocks = (0xFFFFFFFF - raw_value) * 2;
            if (measured_clocks > (IDEAL_SYS_CLOCKS_PER_SECOND * 0.9) && measured_clocks < (IDEAL_SYS_CLOCKS_PER_SECOND * 1.1)) {
                critical_section_enter_blocking(&g_correction_crit_sec);
                g_measured_clocks_per_gps_second = measured_clocks;
                critical_section_exit(&g_correction_crit_sec);
            }
        }
        __wfi();
    }
    return 0;
}

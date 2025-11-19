/**
 * PIO Timestamp Capture Initialization
 *
 * Configures 4 PIO state machines for hardware timestamping:
 *   SM0: Timer - Free-running 32-bit counter
 *   SM1: GPS PPS capture (1 Hz)
 *   SM2: Grandmaster PPS capture (100 Hz)
 *   SM3: Slave PPS capture (100 Hz)
 */

#ifndef PIO_TIMESTAMP_H
#define PIO_TIMESTAMP_H

#include <stdint.h>

/**
 * Initialize PIO timestamp capture system
 *
 * Sets up 4 state machines on PIO0:
 * - SM0 runs a free-running timer at 125 MHz (8ns resolution)
 * - SM1-3 capture timer value on rising edges of their input pins
 *
 * All 4 SMs are started simultaneously for timing coherence.
 */
void pio_timestamp_init(void);

/**
 * Get PIO timer clock frequency
 *
 * @return Frequency in Hz (typically 125000000 for 125 MHz)
 */
uint32_t pio_timestamp_get_freq_hz(void);

/**
 * Get nanoseconds per PIO timer tick
 *
 * @return Nanoseconds per tick (typically 8 for 125 MHz)
 */
uint32_t pio_timestamp_get_ns_per_tick(void);

/**
 * Get IRQ call count (for debugging)
 *
 * @return Number of times the IRQ handler has been called
 */
uint32_t pio_timestamp_get_irq_count(void);

/**
 * Get current timer value from DMA memory (for debugging)
 *
 * @return Current timer value being broadcast by DMA
 */
uint32_t pio_timestamp_get_timer_value(void);

/**
 * Get GPS PPS IRQ count (for debugging)
 *
 * @return Number of times GPS PPS was detected in IRQ handler
 */
uint32_t pio_timestamp_get_gps_irq_count(void);

#endif // PIO_TIMESTAMP_H

/**
 * PIO Timestamp Capture Initialization
 *
 * Configures PIO state machines for hardware timestamping.
 */

#ifndef PIO_TIMESTAMP_H
#define PIO_TIMESTAMP_H

#include <stdint.h>

/**
 * @brief Initializes the PIO timestamp capture system.
 *
 * Sets up 3 state machines on PIO0 to capture raw PIO counter values
 * on rising edges of input pins (GPS, GM, Slave).
 * Configures DMA to continuously drain the PIO RX FIFOs, and sets up
 * an IRQ handler to process these captured values and correlate them
 * with system absolute time.
 */
void pio_timestamp_init(void);

/**
 * @brief Get the PIO clock frequency.
 *
 * @return Frequency in Hz (typically 125,000,000).
 */
uint32_t pio_timestamp_get_freq_hz(void);

/**
 * @brief Get the nanoseconds per PIO clock tick.
 *
 * @return Nanoseconds per tick (typically 8ns for a 125MHz clock).
 */
uint32_t pio_timestamp_get_ns_per_tick(void);

#endif // PIO_TIMESTAMP_H
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

// Ring buffer size - holds ~16 minutes of measurements at 1 Hz
#define MEASUREMENT_BUFFER_SIZE 1000

/**
 * Single measurement data point
 * All timing information for one PPS cycle
 */
typedef struct {
    uint64_t sequence;           // Measurement number
    double phase_offset_ns;      // True phase offset (minimum of both directions)
    double gm_to_slave_ns;       // GM→Slave measurement
    double slave_to_gm_ns;       // Slave→GM measurement
    double scale_factor;         // GPS-calibrated crystal scale
    bool gm_first;               // Which edge came first
    uint64_t timestamp_us;       // System time when measured
    int32_t crystal_error_ns;    // GPS crystal error this second
} measurement_t;

/**
 * Lock-free ring buffer for Core 0 → Core 1 communication
 *
 * Core 0 (writer): Only modifies write_idx and dropped_count
 * Core 1 (reader): Only modifies read_idx
 *
 * No mutexes needed - single producer, single consumer
 */
typedef struct {
    measurement_t buffer[MEASUREMENT_BUFFER_SIZE];
    volatile uint32_t write_idx;      // Next slot to write (Core 0 only)
    volatile uint32_t read_idx;       // Next slot to read (Core 1 only)
    volatile uint32_t dropped_count;  // Incremented when buffer full (Core 0 only)
} measurement_ring_buffer_t;

/**
 * Initialize ring buffer
 * Call once from Core 0 before launching Core 1
 */
void ring_buffer_init(measurement_ring_buffer_t *rb);

/**
 * Non-blocking write (Core 0 only)
 *
 * @param rb Ring buffer
 * @param m Measurement to write
 * @return true if written, false if buffer full
 *
 * If buffer is full, increments dropped_count and returns false
 */
bool ring_buffer_try_write(measurement_ring_buffer_t *rb, const measurement_t *m);

/**
 * Blocking read with timeout (Core 1 only)
 *
 * @param rb Ring buffer
 * @param m Output measurement
 * @param timeout_ms Maximum time to wait (0 = no wait)
 * @return true if read, false on timeout
 */
bool ring_buffer_read(measurement_ring_buffer_t *rb, measurement_t *m, uint32_t timeout_ms);

/**
 * Get buffer statistics (callable from either core)
 *
 * @param rb Ring buffer
 * @param available Output: number of measurements available to read
 * @param dropped Output: total dropped count since init
 */
void ring_buffer_get_stats(const measurement_ring_buffer_t *rb,
                           uint32_t *available, uint32_t *dropped);

#endif // RING_BUFFER_H

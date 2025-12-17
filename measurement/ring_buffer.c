#include "ring_buffer.h"
#include "pico/stdlib.h"
#include <string.h>

void ring_buffer_init(measurement_ring_buffer_t *rb) {
    rb->write_idx = 0;
    rb->read_idx = 0;
    rb->dropped_count = 0;

    // Clear buffer (optional, for cleanliness)
    memset(rb->buffer, 0, sizeof(rb->buffer));
}

bool ring_buffer_try_write(measurement_ring_buffer_t *rb, const measurement_t *m) {
    // Calculate next write position
    uint32_t next_write = (rb->write_idx + 1) % MEASUREMENT_BUFFER_SIZE;

    // Check if buffer is full (write would catch up to read)
    if (next_write == rb->read_idx) {
        // Buffer full - drop measurement
        rb->dropped_count++;
        return false;
    }

    // Write measurement to current slot
    rb->buffer[rb->write_idx] = *m;

    // Advance write index (atomic on single core)
    rb->write_idx = next_write;

    return true;
}

bool ring_buffer_read(measurement_ring_buffer_t *rb, measurement_t *m, uint32_t timeout_ms) {
    uint64_t start_time = time_us_64();
    uint64_t timeout_us = (uint64_t)timeout_ms * 1000;

    while (rb->read_idx == rb->write_idx) {
        // Buffer empty
        if (timeout_ms == 0) {
            return false;  // Non-blocking mode
        }

        // Check timeout
        if ((time_us_64() - start_time) >= timeout_us) {
            return false;
        }

        // Yield to other core (small sleep to avoid busy-wait)
        sleep_us(100);
    }

    // Read measurement from current slot
    *m = rb->buffer[rb->read_idx];

    // Advance read index
    rb->read_idx = (rb->read_idx + 1) % MEASUREMENT_BUFFER_SIZE;

    return true;
}

void ring_buffer_get_stats(const measurement_ring_buffer_t *rb,
                           uint32_t *available, uint32_t *dropped) {
    // Calculate available measurements
    uint32_t write = rb->write_idx;
    uint32_t read = rb->read_idx;

    if (write >= read) {
        *available = write - read;
    } else {
        *available = MEASUREMENT_BUFFER_SIZE - read + write;
    }

    *dropped = rb->dropped_count;
}

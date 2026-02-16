#ifndef LTSP_PIO_TIMESTAMP_H
#define LTSP_PIO_TIMESTAMP_H

#include <stdint.h>
#include <stdbool.h>

#define LTSP_PIO_TICK_NS 12

/*
 * 64-bit extended PIO timestamp.
 * The PIO X register is 32 bits, counting DOWN at 83.33MHz (12ns/tick).
 * Wraps every ~51.5 seconds. This struct tracks the upper 32 bits in software.
 */
typedef struct {
    uint32_t upper;     /* Software-maintained upper half (increments on wrap) */
    uint32_t lower;     /* Raw 32-bit PIO counter value (counts DOWN) */
    bool     initialized;
} ltsp_pio_ts_t;

/* Initialize (not yet valid — call extend() with first sample) */
void ltsp_pio_ts_init(ltsp_pio_ts_t *ts);

/*
 * Extend a new 32-bit counter reading to 64 bits.
 * Detects wrap: counter counts DOWN, so wrap occurs when new > previous.
 * Updates ts in place.
 */
void ltsp_pio_ts_extend(ltsp_pio_ts_t *ts, uint32_t raw);

/* Combine upper and lower into a single 64-bit value.
 * The value counts DOWN — larger values are earlier in time. */
uint64_t ltsp_pio_ts_to_u64(const ltsp_pio_ts_t *ts);

/*
 * Compute the signed difference between two 64-bit PIO timestamps
 * in PIO ticks. Result = a - b (positive if a is earlier in time,
 * since counter counts DOWN: earlier = higher value).
 *
 * For elapsed time: elapsed_ticks = ltsp_pio_ts_diff(earlier, later)
 * which will be positive.
 */
int64_t ltsp_pio_ts_diff(const ltsp_pio_ts_t *a, const ltsp_pio_ts_t *b);

/*
 * Convert to a monotonically INCREASING tick count.
 * The raw (upper, lower) representation has discontinuities at wrap
 * because upper increments while lower jumps to 0xFFFFFFFF.
 * This function returns: upper * 2^32 + (0xFFFFFFFF - lower),
 * which increases by 1 for every PIO tick regardless of wraps.
 */
static inline uint64_t ltsp_pio_ts_to_monotonic(const ltsp_pio_ts_t *ts) {
    return ((uint64_t)ts->upper << 32) + (0xFFFFFFFFU - ts->lower);
}

/* Convert a tick difference to nanoseconds */
static inline int64_t ltsp_pio_ticks_to_ns(int64_t ticks) {
    return ticks * LTSP_PIO_TICK_NS;
}

/* Convert nanoseconds to PIO ticks (truncating) */
static inline int64_t ltsp_ns_to_pio_ticks(int64_t ns) {
    return ns / LTSP_PIO_TICK_NS;
}

#endif /* LTSP_PIO_TIMESTAMP_H */

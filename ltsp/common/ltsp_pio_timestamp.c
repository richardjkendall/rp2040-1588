#include "ltsp_pio_timestamp.h"

void ltsp_pio_ts_init(ltsp_pio_ts_t *ts) {
    ts->upper       = 0;
    ts->lower       = 0;
    ts->initialized = false;
}

void ltsp_pio_ts_extend(ltsp_pio_ts_t *ts, uint32_t raw) {
    if (!ts->initialized) {
        ts->lower       = raw;
        ts->upper       = 0;
        ts->initialized = true;
        return;
    }

    /* Counter counts DOWN. Wrap occurs when new > previous
     * (counter went from near 0 back to near 0xFFFFFFFF). */
    if (raw > ts->lower) {
        ts->upper++;
    }
    ts->lower = raw;
}

uint64_t ltsp_pio_ts_to_u64(const ltsp_pio_ts_t *ts) {
    return ((uint64_t)ts->upper << 32) | ts->lower;
}

int64_t ltsp_pio_ts_diff(const ltsp_pio_ts_t *a, const ltsp_pio_ts_t *b) {
    uint64_t va = ltsp_pio_ts_to_u64(a);
    uint64_t vb = ltsp_pio_ts_to_u64(b);
    /* Counter counts DOWN: earlier time = higher value.
     * diff = a - b: positive when a is earlier (higher). */
    return (int64_t)(va - vb);
}

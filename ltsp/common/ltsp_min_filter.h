#ifndef LTSP_MIN_FILTER_H
#define LTSP_MIN_FILTER_H

#include <stdint.h>
#include <stdbool.h>

#define LTSP_MIN_FILTER_DEFAULT_WINDOW 120
#define LTSP_PATH_JUMP_THRESHOLD_NS    10000  /* 10 us */

typedef struct {
    int64_t *buffer;        /* Circular buffer of delay samples (ns) */
    uint16_t window;        /* Buffer capacity */
    uint16_t write_idx;     /* Next write position */
    uint16_t count;         /* Samples currently in buffer */
    int64_t  current_min;   /* Current minimum value */
    uint16_t min_age;       /* Samples since min was last set */
    bool     valid;         /* Has at least one sample */
} ltsp_min_filter_t;

/*
 * Initialize the minimum filter.
 * buf must point to an array of at least window int64_t elements.
 */
void ltsp_min_filter_init(ltsp_min_filter_t *f, int64_t *buf, uint16_t window);

/*
 * Add a new delay sample (nanoseconds).
 * Returns true if a path jump was detected (filter was reset).
 */
bool ltsp_min_filter_update(ltsp_min_filter_t *f, int64_t delay_ns);

/* Reset the filter (e.g., after protocol restart or path jump) */
void ltsp_min_filter_reset(ltsp_min_filter_t *f);

/* Get current minimum estimate. Only valid if f->valid is true. */
int64_t ltsp_min_filter_get_min(const ltsp_min_filter_t *f);

#endif /* LTSP_MIN_FILTER_H */

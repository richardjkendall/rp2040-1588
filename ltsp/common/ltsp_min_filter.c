#include "ltsp_min_filter.h"
#include <limits.h>

void ltsp_min_filter_init(ltsp_min_filter_t *f, int64_t *buf, uint16_t window) {
    f->buffer      = buf;
    f->window      = window;
    f->write_idx   = 0;
    f->count       = 0;
    f->current_min = INT64_MAX;
    f->min_age     = 0;
    f->valid       = false;
}

void ltsp_min_filter_reset(ltsp_min_filter_t *f) {
    f->write_idx   = 0;
    f->count       = 0;
    f->current_min = INT64_MAX;
    f->min_age     = 0;
    f->valid       = false;
}

static int64_t scan_min(const ltsp_min_filter_t *f) {
    int64_t min = INT64_MAX;
    uint16_t n = (f->count < f->window) ? f->count : f->window;
    for (uint16_t i = 0; i < n; i++) {
        if (f->buffer[i] < min) {
            min = f->buffer[i];
        }
    }
    return min;
}

bool ltsp_min_filter_update(ltsp_min_filter_t *f, int64_t delay_ns) {
    bool path_jump = false;

    /* Store sample */
    f->buffer[f->write_idx] = delay_ns;
    f->write_idx = (f->write_idx + 1) % f->window;
    if (f->count < f->window) {
        f->count++;
    }

    if (!f->valid) {
        /* First sample */
        f->current_min = delay_ns;
        f->min_age     = 0;
        f->valid       = true;
        return false;
    }

    if (delay_ns <= f->current_min) {
        /* New minimum or equal — path jump detection */
        int64_t prev_min = f->current_min;
        f->current_min = delay_ns;
        f->min_age     = 0;

        /* Check for path jump: new min significantly larger than old min
         * This detects a route change to a longer path */
        if (delay_ns > prev_min + LTSP_PATH_JUMP_THRESHOLD_NS) {
            /* This shouldn't happen if delay_ns <= current_min, but guard
             * against the case where we just did a full scan reset */
        }
    } else {
        f->min_age++;

        /* Check if the minimum has aged out of the window */
        if (f->min_age >= f->window) {
            int64_t old_min = f->current_min;
            f->current_min = scan_min(f);
            f->min_age     = 0;

            /* Path jump detection: new minimum significantly higher */
            if (f->current_min > old_min + LTSP_PATH_JUMP_THRESHOLD_NS) {
                path_jump = true;
                ltsp_min_filter_reset(f);
                /* Re-add current sample as first in fresh filter */
                f->buffer[0] = delay_ns;
                f->write_idx = 1;
                f->count     = 1;
                f->current_min = delay_ns;
                f->valid     = true;
            }
        }
    }

    return path_jump;
}

int64_t ltsp_min_filter_get_min(const ltsp_min_filter_t *f) {
    return f->current_min;
}

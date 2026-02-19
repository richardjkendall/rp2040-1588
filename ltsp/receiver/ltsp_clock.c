/**
 * LTSP Clock — GPS-domain, regression-only frequency
 *
 * Maintains a disciplined GPS time estimate on the receiver by:
 * 1. Initializing clock to GPS TX time (offset by one-way delay)
 * 2. Interpolating between packets using crystal scale_factor
 * 3. Setting frequency directly from drift regression (no servo)
 * 4. Managing sync state (INIT → ACQUIRING → LOCKED → HOLDOVER)
 */

#include "ltsp_clock.h"
#include "pico/stdlib.h"
#include "pico/platform.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Global state (accessible from both cores via get_ltsp_time_ns) */
static volatile int64_t  g_clock_ns = 0;
static volatile uint64_t g_clock_update_us = 0;
static volatile double   g_scale_factor = 1.0;
static volatile bool     g_clock_valid = false;

void ltsp_clock_init(ltsp_clock_state_t *st) {
    st->clock_ns = 0;
    st->clock_update_us = 0;
    st->scale_factor = 1.0;
    st->base_drift_ns_per_s = 0.0;
    st->state = LTSP_SYNC_INIT;
    st->valid_packet_count = 0;
    st->locked_count = 0;
    st->last_packet_us = 0;
    st->last_clock_error_ns = 0;
    st->clock_valid = false;

    g_clock_ns = 0;
    g_clock_update_us = 0;
    g_scale_factor = 1.0;
    g_clock_valid = false;
}

/**
 * Get current disciplined time.
 * Placed in SRAM for deterministic execution (no flash cache misses).
 */
uint64_t __time_critical_func(get_ltsp_time_ns)(void) {
    if (!g_clock_valid) return 0;

    uint64_t now_us = time_us_64();
    uint64_t elapsed_us = now_us - g_clock_update_us;
    int64_t elapsed_ns = (int64_t)(elapsed_us * 1000.0 * g_scale_factor);
    return (uint64_t)(g_clock_ns + elapsed_ns);
}

double ltsp_clock_get_scale_factor(void) {
    return g_scale_factor;
}

void ltsp_clock_set_initial(ltsp_clock_state_t *st,
                            int64_t gps_time_ns,
                            double drift_ns_per_s) {
    uint64_t now_us = time_us_64();

    /* Set initial frequency from drift regression */
    st->base_drift_ns_per_s = drift_ns_per_s;
    st->scale_factor = 1.0 - (drift_ns_per_s / 1e9);
    st->clock_ns = gps_time_ns;
    st->clock_update_us = now_us;
    st->clock_valid = true;

    /* Publish to global (atomic on ARM for aligned writes) */
    g_scale_factor = st->scale_factor;
    g_clock_ns = st->clock_ns;
    g_clock_update_us = st->clock_update_us;
    g_clock_valid = true;

    printf("# CLOCK: Initial set — GPS time %lld ns, drift %.1f ns/s, sf=%.9f\n",
           (long long)gps_time_ns, drift_ns_per_s, st->scale_factor);
}

void ltsp_clock_advance(ltsp_clock_state_t *st) {
    if (!st->clock_valid) return;

    uint64_t now_us = time_us_64();
    uint64_t elapsed_us = now_us - st->clock_update_us;
    if (elapsed_us == 0) return;

    int64_t elapsed_ns = (int64_t)(elapsed_us * 1000.0 * st->scale_factor);
    st->clock_ns += elapsed_ns;
    st->clock_update_us = now_us;

    /* Publish */
    g_clock_ns = st->clock_ns;
    g_clock_update_us = st->clock_update_us;
}

void ltsp_clock_reanchor(ltsp_clock_state_t *st, int64_t gps_time_ns) {
    if (!st->clock_valid) return;

    uint64_t now_us = time_us_64();
    st->clock_ns = gps_time_ns;
    st->clock_update_us = now_us;

    /* Publish */
    g_clock_ns = st->clock_ns;
    g_clock_update_us = st->clock_update_us;
}

void ltsp_clock_update_frequency(ltsp_clock_state_t *st,
                                 double drift_ns_per_s) {
    st->base_drift_ns_per_s = drift_ns_per_s;
    st->scale_factor = 1.0 - (drift_ns_per_s / 1e9);
    g_scale_factor = st->scale_factor;
}

void ltsp_clock_update_state(ltsp_clock_state_t *st,
                             bool has_packet,
                             bool gm_holdover,
                             int64_t clock_error_ns) {
    uint64_t now_us = time_us_64();

    if (has_packet) {
        st->last_packet_us = now_us;
    }

    switch (st->state) {
    case LTSP_SYNC_INIT:
        if (has_packet) {
            st->valid_packet_count++;
            if (st->valid_packet_count >= LTSP_INIT_PACKETS_REQUIRED) {
                st->state = LTSP_SYNC_ACQUIRING;
                printf("# STATE: INIT → ACQUIRING (%lu packets)\n",
                       st->valid_packet_count);
            }
        } else {
            st->valid_packet_count = 0;
        }
        break;

    case LTSP_SYNC_ACQUIRING:
        /* Transition to LOCKED handled externally after clock is set
         * and clock_error is available */
        if (st->clock_valid && llabs(clock_error_ns) < LTSP_LOCK_THRESHOLD_NS) {
            st->locked_count++;
            if (st->locked_count >= LTSP_LOCK_SAMPLES_REQUIRED) {
                st->state = LTSP_SYNC_LOCKED;
                printf("# STATE: ACQUIRING → LOCKED (error %+lld ns)\n",
                       (long long)clock_error_ns);
            }
        } else {
            st->locked_count = 0;
        }
        /* Fall to HOLDOVER check */
        if (!has_packet && st->last_packet_us > 0 &&
            (now_us - st->last_packet_us) > LTSP_HOLDOVER_TIMEOUT_US) {
            st->state = LTSP_SYNC_HOLDOVER;
            printf("# STATE: ACQUIRING → HOLDOVER (timeout)\n");
        }
        break;

    case LTSP_SYNC_LOCKED:
        if (gm_holdover) {
            st->state = LTSP_SYNC_HOLDOVER;
            printf("# STATE: LOCKED → HOLDOVER (GM holdover)\n");
        } else if (!has_packet && st->last_packet_us > 0 &&
                   (now_us - st->last_packet_us) > LTSP_HOLDOVER_TIMEOUT_US) {
            st->state = LTSP_SYNC_HOLDOVER;
            printf("# STATE: LOCKED → HOLDOVER (timeout)\n");
        }
        break;

    case LTSP_SYNC_HOLDOVER:
        if (has_packet && !gm_holdover) {
            /* Re-acquire — go back to ACQUIRING, not directly to LOCKED */
            st->state = LTSP_SYNC_ACQUIRING;
            st->locked_count = 0;
            printf("# STATE: HOLDOVER → ACQUIRING (packets resumed)\n");
        } else if (!has_packet && st->last_packet_us > 0 &&
                   (now_us - st->last_packet_us) > 2 * LTSP_HOLDOVER_TIMEOUT_US) {
            /* Extended holdover — reset to INIT */
            st->state = LTSP_SYNC_INIT;
            st->valid_packet_count = 0;
            st->clock_valid = false;
            g_clock_valid = false;
            printf("# STATE: HOLDOVER → INIT (extended timeout)\n");
        }
        break;
    }
}

const char *ltsp_sync_state_name(ltsp_sync_state_t state) {
    switch (state) {
    case LTSP_SYNC_INIT:      return "INIT";
    case LTSP_SYNC_ACQUIRING: return "ACQUIRING";
    case LTSP_SYNC_LOCKED:    return "LOCKED";
    case LTSP_SYNC_HOLDOVER:  return "HOLDOVER";
    default:                  return "UNKNOWN";
    }
}

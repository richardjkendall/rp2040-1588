#ifndef LTSP_CLOCK_H
#define LTSP_CLOCK_H

#include <stdint.h>
#include <stdbool.h>

/* Synchronization state machine (RFC Section 9) */
typedef enum {
    LTSP_SYNC_INIT,         /* Collecting initial samples, no clock output */
    LTSP_SYNC_ACQUIRING,    /* Drift regression converging, clock low quality */
    LTSP_SYNC_LOCKED,       /* Steady state, clock valid, 1PPS enabled */
    LTSP_SYNC_HOLDOVER      /* No packets, extrapolating with last model */
} ltsp_sync_state_t;

/* Clock discipline state */
typedef struct {
    /* Disciplined clock */
    int64_t  clock_ns;            /* Current disciplined GPS time estimate */
    uint64_t clock_update_us;     /* System timer at last update */
    double   scale_factor;        /* Crystal correction (1.0 = nominal) */

    /* PI servo */
    double   base_drift_ns_per_s; /* Initial drift from regression */
    double   pi_integral;         /* Accumulated residual frequency error (ns/s) */
    double   freq_offset_ppb;     /* Total frequency correction */

    /* State machine */
    ltsp_sync_state_t state;
    uint32_t valid_packet_count;  /* Consecutive valid packets (INIT→ACQUIRING) */
    uint32_t locked_count;        /* Consecutive low-error samples */
    uint64_t last_packet_us;      /* System timer at last packet (holdover timeout) */

    /* Statistics */
    int64_t  last_clock_error_ns;
    bool     clock_valid;         /* True when clock output is meaningful */
} ltsp_clock_state_t;

/* State transition thresholds */
#define LTSP_INIT_PACKETS_REQUIRED     3       /* Valid packets before ACQUIRING */
#define LTSP_LOCK_THRESHOLD_NS         500000  /* 500 µs — clock error for lock */
#define LTSP_LOCK_SAMPLES_REQUIRED     5       /* Consecutive low-error for LOCKED */
#define LTSP_HOLDOVER_TIMEOUT_US       30000000ULL  /* 30 seconds */
#define LTSP_PHASE_STEP_THRESHOLD_NS   1000000 /* 1 ms — jump rather than slew */

/**
 * Initialize clock discipline state.
 */
void ltsp_clock_init(ltsp_clock_state_t *st);

/**
 * Get current disciplined time in nanoseconds.
 * Interpolates between updates using system timer and scale_factor.
 * Placed in SRAM for deterministic, low-jitter execution.
 * Safe to call from any core.
 */
uint64_t get_ltsp_time_ns(void);

/**
 * Get current crystal scale factor (for 1PPS scheduler).
 */
double ltsp_clock_get_scale_factor(void);

/**
 * Set initial clock value (phase step) and frequency.
 * Called once when transitioning from INIT to ACQUIRING.
 *
 * @param gps_time_ns  Estimated GPS time at this moment
 * @param drift_ns_per_s  Measured crystal drift (from regression a1)
 */
void ltsp_clock_set_initial(ltsp_clock_state_t *st,
                            int64_t gps_time_ns,
                            double drift_ns_per_s);

/**
 * Advance clock_ns to current time using scale_factor interpolation.
 * Must be called before ltsp_clock_discipline() to bring clock up to date.
 */
void ltsp_clock_advance(ltsp_clock_state_t *st);

/**
 * Apply PI servo discipline step.
 * Returns the correction applied (ns). Positive = clock was ahead, stepped back.
 *
 * @param clock_error_ns  Our clock minus GM estimate (positive = ahead)
 * @param dt_sec          Time since last discipline step (seconds)
 */
int64_t ltsp_clock_discipline(ltsp_clock_state_t *st,
                              int64_t clock_error_ns,
                              double dt_sec);

/**
 * Update scale_factor directly from drift regression.
 * Called each packet with the current regression a1 value.
 */
void ltsp_clock_update_frequency(ltsp_clock_state_t *st,
                                  double drift_ns_per_s);

/**
 * Update synchronization state machine.
 *
 * @param has_packet    True if a valid packet was received this cycle
 * @param gm_holdover   True if GM's HOLDOVER flag is set
 * @param clock_error_ns  Current clock error (for lock detection)
 */
void ltsp_clock_update_state(ltsp_clock_state_t *st,
                             bool has_packet,
                             bool gm_holdover,
                             int64_t clock_error_ns);

/**
 * Get state name as string (for CSV/logging).
 */
const char *ltsp_sync_state_name(ltsp_sync_state_t state);

#endif /* LTSP_CLOCK_H */

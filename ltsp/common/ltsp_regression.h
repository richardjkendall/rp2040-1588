#ifndef LTSP_REGRESSION_H
#define LTSP_REGRESSION_H

#include <stdint.h>
#include <stdbool.h>

#define LTSP_REGRESSION_DEFAULT_WINDOW 60

/* Regression output */
typedef struct {
    double  a0;         /* Phase bias at t_0 (PIO ticks) */
    double  a1;         /* Frequency drift (PIO ticks per PIO tick, dimensionless) */
    float   a1_ppb;     /* Frequency drift (ppb, for PDU) */
    float   sigma;      /* Residual standard deviation (PIO ticks) */
    float   sigma_ns;   /* Residual standard deviation (nanoseconds) */
    int64_t t_ref;      /* Reference time t_0 (PIO ticks of first sample in buffer) */
    uint16_t n;         /* Number of samples used */
    bool    valid;      /* True if n >= 2 */
} ltsp_regression_result_t;

/* Circular buffer sample */
typedef struct {
    int64_t t;          /* PIO counter at 1PPS edge (64-bit extended) */
    int64_t e;          /* Phase error in PIO ticks (expected - actual interval) */
} ltsp_regression_sample_t;

typedef struct {
    ltsp_regression_sample_t *buffer;   /* Circular buffer */
    uint16_t window;                    /* Buffer capacity */
    uint16_t write_idx;                 /* Next write position */
    uint16_t count;                     /* Samples in buffer */
    uint32_t total_samples;             /* Total samples ever added */

    /* Running sums for incremental computation (double for precision) */
    double S_tau;       /* sum of tau_k (relative times) */
    double S_e;         /* sum of e_k */
    double S_tau_tau;   /* sum of tau_k^2 */
    double S_tau_e;     /* sum of tau_k * e_k */
    double S_e_e;       /* sum of e_k^2 */

    /* Last computed result */
    ltsp_regression_result_t result;
} ltsp_regression_t;

/*
 * Initialize the regression.
 * buf must point to an array of at least window samples.
 */
void ltsp_regression_init(ltsp_regression_t *r,
                          ltsp_regression_sample_t *buf,
                          uint16_t window);

/*
 * Add a new 1PPS sample.
 * t: PIO counter value at this 1PPS edge (64-bit extended)
 * e: phase error = expected_ticks - actual_interval (PIO ticks)
 */
void ltsp_regression_add_sample(ltsp_regression_t *r, int64_t t, int64_t e);

/*
 * Compute the regression.
 * Must have at least 2 samples. Updates r->result.
 */
void ltsp_regression_compute(ltsp_regression_t *r);

/*
 * Re-reference a0 to a new epoch.
 * Returns a0 at t_epoch: a0_new = a0 + a1 * (t_epoch - t_ref)
 * t_epoch is in the same domain as the sample timestamps (PIO ticks).
 */
double ltsp_regression_a0_at(const ltsp_regression_t *r, int64_t t_epoch);

#endif /* LTSP_REGRESSION_H */

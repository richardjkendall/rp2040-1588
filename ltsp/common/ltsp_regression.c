#include "ltsp_regression.h"
#include <math.h>

#define PIO_TICK_NS 12

void ltsp_regression_init(ltsp_regression_t *r,
                          ltsp_regression_sample_t *buf,
                          uint16_t window) {
    r->buffer        = buf;
    r->window        = window;
    r->write_idx     = 0;
    r->count         = 0;
    r->total_samples = 0;
    r->S_tau     = 0.0;
    r->S_e       = 0.0;
    r->S_tau_tau = 0.0;
    r->S_tau_e   = 0.0;
    r->S_e_e     = 0.0;
    r->result.valid = false;
    r->result.n     = 0;
}

void ltsp_regression_add_sample(ltsp_regression_t *r, int64_t t, int64_t e) {
    /* If buffer full, evict oldest (will recompute from scratch) */
    if (r->count >= r->window) {
        /* Just overwrite — we recompute from scratch every time */
    }

    r->buffer[r->write_idx].t = t;
    r->buffer[r->write_idx].e = e;
    r->write_idx = (r->write_idx + 1) % r->window;
    if (r->count < r->window) {
        r->count++;
    }
    r->total_samples++;
}

/*
 * Full recomputation from buffer contents.
 * Uses relative times (tau_k = t_k - t_0) for numerical stability.
 */
void ltsp_regression_compute(ltsp_regression_t *r) {
    uint16_t n = r->count;
    if (n < 2) {
        r->result.valid = false;
        r->result.n     = n;
        if (n == 1) {
            /* Single sample: can report bias but not drift */
            uint16_t idx = (r->write_idx + r->window - 1) % r->window;
            r->result.a0    = (double)r->buffer[idx].e;
            r->result.a1    = 0.0;
            r->result.a1_ppb = 0.0f;
            r->result.sigma  = 0.0f;
            r->result.sigma_ns = 0.0f;
            r->result.t_ref = r->buffer[idx].t;
        }
        return;
    }

    /* Find first sample in buffer (oldest) */
    uint16_t start;
    if (r->count < r->window) {
        start = 0;
    } else {
        start = r->write_idx; /* oldest is at write_idx when full */
    }

    int64_t t_0 = r->buffer[start].t;
    r->result.t_ref = t_0;

    /* Recompute running sums from scratch */
    double S_tau     = 0.0;
    double S_e       = 0.0;
    double S_tau_tau = 0.0;
    double S_tau_e   = 0.0;
    double S_e_e     = 0.0;

    for (uint16_t i = 0; i < n; i++) {
        uint16_t idx = (start + i) % r->window;
        double tau_k = (double)(r->buffer[idx].t - t_0);
        double e_k   = (double)r->buffer[idx].e;

        S_tau     += tau_k;
        S_e       += e_k;
        S_tau_tau += tau_k * tau_k;
        S_tau_e   += tau_k * e_k;
        S_e_e     += e_k * e_k;
    }

    /* Store running sums (for potential incremental use later) */
    r->S_tau     = S_tau;
    r->S_e       = S_e;
    r->S_tau_tau = S_tau_tau;
    r->S_tau_e   = S_tau_e;
    r->S_e_e     = S_e_e;

    /* OLS: a1 = (N*S_tau_e - S_tau*S_e) / (N*S_tau_tau - S_tau^2) */
    double denom = (double)n * S_tau_tau - S_tau * S_tau;

    if (fabs(denom) < 1e-30) {
        /* Degenerate — all samples at same time */
        r->result.a0     = S_e / (double)n;
        r->result.a1     = 0.0;
        r->result.a1_ppb = 0.0f;
        r->result.sigma  = 0.0f;
        r->result.sigma_ns = 0.0f;
        r->result.n      = n;
        r->result.valid  = true;
        return;
    }

    double a1 = ((double)n * S_tau_e - S_tau * S_e) / denom;
    double a0 = (S_e - a1 * S_tau) / (double)n;

    /* Residuals for Sigma */
    double ss_res = 0.0;
    for (uint16_t i = 0; i < n; i++) {
        uint16_t idx = (start + i) % r->window;
        double tau_k = (double)(r->buffer[idx].t - t_0);
        double e_k   = (double)r->buffer[idx].e;
        double residual = e_k - a0 - a1 * tau_k;
        ss_res += residual * residual;
    }
    double sigma = sqrt(ss_res / (double)(n - 2));

    r->result.a0       = a0;
    r->result.a1       = a1;
    /* Convert a1 from ticks/tick to ppb:
     * a1 is dimensionless (error_ticks per elapsed_tick)
     * ppb = a1 * 1e9 */
    r->result.a1_ppb   = (float)(a1 * 1e9);
    r->result.sigma    = (float)sigma;
    r->result.sigma_ns = (float)(sigma * PIO_TICK_NS);
    r->result.n        = n;
    r->result.valid    = true;
}

double ltsp_regression_a0_at(const ltsp_regression_t *r, int64_t t_epoch) {
    if (!r->result.valid) return 0.0;
    double dt = (double)(t_epoch - r->result.t_ref);
    return r->result.a0 + r->result.a1 * dt;
}

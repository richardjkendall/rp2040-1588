#include "../common/ltsp_regression.h"
#include <stdio.h>
#include <math.h>

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) do { tests_run++; printf("  %-50s ", name); } while(0)
#define PASS() do { tests_passed++; printf("PASS\n"); } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); } while(0)

/* Expected ticks per second at 83.33MHz */
#define EXPECTED_TICKS 83333333LL

static void test_zero_drift(void) {
    TEST("Zero drift crystal");
    ltsp_regression_sample_t buf[60];
    ltsp_regression_t r;
    ltsp_regression_init(&r, buf, 60);

    /* Perfect crystal: phase error = 0 each second */
    for (int i = 0; i < 10; i++) {
        int64_t t = (int64_t)i * EXPECTED_TICKS;
        ltsp_regression_add_sample(&r, t, 0);
    }
    ltsp_regression_compute(&r);

    if (!r.result.valid) { FAIL("not valid"); return; }
    if (fabs(r.result.a1) > 1e-15) { FAIL("a1 not zero"); return; }
    if (fabs(r.result.a0) > 1e-10) { FAIL("a0 not zero"); return; }
    if (r.result.sigma > 0.001f) { FAIL("sigma not zero"); return; }
    PASS();
}

static void test_constant_drift(void) {
    TEST("Constant drift (+3 ticks/sec)");
    ltsp_regression_sample_t buf[60];
    ltsp_regression_t r;
    ltsp_regression_init(&r, buf, 60);

    /* Crystal consistently +3 ticks fast per second
     * phase error = expected - actual = -3 (crystal ran 3 extra ticks) */
    for (int i = 0; i < 20; i++) {
        int64_t t = (int64_t)i * EXPECTED_TICKS;
        int64_t e = -3;  /* 3 ticks fast */
        ltsp_regression_add_sample(&r, t, e);
    }
    ltsp_regression_compute(&r);

    if (!r.result.valid) { FAIL("not valid"); return; }
    /* a0 should be ~-3 (constant bias) */
    if (fabs(r.result.a0 - (-3.0)) > 0.01) { FAIL("a0 wrong"); return; }
    /* a1 should be ~0 (no drift in the drift — constant error) */
    if (fabs(r.result.a1) > 1e-12) { FAIL("a1 not zero"); return; }
    PASS();
}

static void test_linear_drift(void) {
    TEST("Linearly increasing drift");
    ltsp_regression_sample_t buf[60];
    ltsp_regression_t r;
    ltsp_regression_init(&r, buf, 60);

    /* Phase error grows by 2 ticks per second:
     * e(i) = 2*i */
    for (int i = 0; i < 30; i++) {
        int64_t t = (int64_t)i * EXPECTED_TICKS;
        int64_t e = 2 * i;
        ltsp_regression_add_sample(&r, t, e);
    }
    ltsp_regression_compute(&r);

    if (!r.result.valid) { FAIL("not valid"); return; }
    /* a1 = slope of e vs t. e increases by 2 per EXPECTED_TICKS ticks.
     * a1 = 2 / EXPECTED_TICKS ≈ 2.4e-8 */
    double expected_a1 = 2.0 / (double)EXPECTED_TICKS;
    if (fabs(r.result.a1 - expected_a1) > 1e-15) {
        printf("(a1=%.15e expected=%.15e) ", r.result.a1, expected_a1);
        FAIL("a1 wrong");
        return;
    }
    /* sigma should be ~0 (perfect linear data) */
    if (r.result.sigma > 0.01f) { FAIL("sigma too large"); return; }
    PASS();
}

static void test_a1_ppb(void) {
    TEST("a1_ppb conversion");
    ltsp_regression_sample_t buf[60];
    ltsp_regression_t r;
    ltsp_regression_init(&r, buf, 60);

    /* Simulate ~36 ppb drift: phase error grows by 3 ticks/sec
     * a1 = 3/EXPECTED_TICKS ticks/tick
     * ppb = a1 * 1e9 = 3e9/83333333 ≈ 36 ppb */
    for (int i = 0; i < 20; i++) {
        int64_t t = (int64_t)i * EXPECTED_TICKS;
        int64_t e = 3 * i;
        ltsp_regression_add_sample(&r, t, e);
    }
    ltsp_regression_compute(&r);

    float expected_ppb = (float)(3.0e9 / (double)EXPECTED_TICKS);
    if (fabsf(r.result.a1_ppb - expected_ppb) > 0.1f) {
        printf("(a1_ppb=%.2f expected=%.2f) ", r.result.a1_ppb, expected_ppb);
        FAIL("a1_ppb wrong");
        return;
    }
    PASS();
}

static void test_sigma_with_noise(void) {
    TEST("Sigma reflects noise level");
    ltsp_regression_sample_t buf[60];
    ltsp_regression_t r;
    ltsp_regression_init(&r, buf, 60);

    /* Linear drift + noise pattern: e(i) = 5*i + noise
     * Use a simple deterministic noise pattern */
    int noise[] = {1, -2, 3, -1, 0, 2, -3, 1, -1, 2,
                   -2, 3, 0, -1, 1, -3, 2, 0, 1, -2};
    for (int i = 0; i < 20; i++) {
        int64_t t = (int64_t)i * EXPECTED_TICKS;
        int64_t e = 5 * i + noise[i];
        ltsp_regression_add_sample(&r, t, e);
    }
    ltsp_regression_compute(&r);

    if (!r.result.valid) { FAIL("not valid"); return; }
    /* Sigma should be > 0 (there's noise) but reasonable */
    if (r.result.sigma < 0.5f) { FAIL("sigma too small"); return; }
    if (r.result.sigma > 10.0f) { FAIL("sigma too large"); return; }
    PASS();
}

static void test_re_reference(void) {
    TEST("Re-reference a0 to new epoch");
    ltsp_regression_sample_t buf[60];
    ltsp_regression_t r;
    ltsp_regression_init(&r, buf, 60);

    /* e(i) = 100 + 2*i */
    for (int i = 0; i < 10; i++) {
        int64_t t = (int64_t)i * EXPECTED_TICKS;
        int64_t e = 100 + 2 * i;
        ltsp_regression_add_sample(&r, t, e);
    }
    ltsp_regression_compute(&r);

    /* Re-reference to t=5*EXPECTED_TICKS (middle of data) */
    int64_t t_epoch = 5 * EXPECTED_TICKS;
    double a0_new = ltsp_regression_a0_at(&r, t_epoch);

    /* At i=5: e = 100 + 10 = 110. a0_new should be ~110 */
    if (fabs(a0_new - 110.0) > 0.1) {
        printf("(a0_new=%.2f expected=110) ", a0_new);
        FAIL("a0 re-reference wrong");
        return;
    }
    PASS();
}

static void test_buffer_wrap(void) {
    TEST("Buffer wraps correctly at window size");
    ltsp_regression_sample_t buf[10];
    ltsp_regression_t r;
    ltsp_regression_init(&r, buf, 10);

    /* Add 15 samples (wraps past 10) */
    for (int i = 0; i < 15; i++) {
        int64_t t = (int64_t)i * EXPECTED_TICKS;
        int64_t e = i * 2;
        ltsp_regression_add_sample(&r, t, e);
    }
    ltsp_regression_compute(&r);

    if (!r.result.valid) { FAIL("not valid"); return; }
    if (r.result.n != 10) { FAIL("wrong count"); return; }
    /* Should use samples 5-14, with e = 10..28 */
    PASS();
}

static void test_single_sample(void) {
    TEST("Single sample (insufficient for regression)");
    ltsp_regression_sample_t buf[10];
    ltsp_regression_t r;
    ltsp_regression_init(&r, buf, 10);

    ltsp_regression_add_sample(&r, 0, 42);
    ltsp_regression_compute(&r);

    if (r.result.valid) { FAIL("should not be valid with 1 sample"); return; }
    if (r.result.n != 1) { FAIL("wrong count"); return; }
    PASS();
}

void test_regression_run(void) {
    printf("\n=== Regression Tests ===\n");
    test_zero_drift();
    test_constant_drift();
    test_linear_drift();
    test_a1_ppb();
    test_sigma_with_noise();
    test_re_reference();
    test_buffer_wrap();
    test_single_sample();
    printf("Regression: %d/%d passed\n", tests_passed, tests_run);
}

int test_regression_results(int *run, int *passed) {
    *run = tests_run; *passed = tests_passed;
    return tests_run - tests_passed;
}

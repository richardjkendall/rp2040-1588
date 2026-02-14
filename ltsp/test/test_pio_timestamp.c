#include "../common/ltsp_pio_timestamp.h"
#include <stdio.h>

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) do { tests_run++; printf("  %-50s ", name); } while(0)
#define PASS() do { tests_passed++; printf("PASS\n"); } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); } while(0)

static void test_init_and_first_sample(void) {
    TEST("Init and first sample");
    ltsp_pio_ts_t ts;
    ltsp_pio_ts_init(&ts);
    if (ts.initialized) { FAIL("should not be initialized"); return; }

    ltsp_pio_ts_extend(&ts, 0xFFFFFFF0);
    if (!ts.initialized) { FAIL("should be initialized"); return; }
    if (ts.upper != 0) { FAIL("upper not 0"); return; }
    if (ts.lower != 0xFFFFFFF0) { FAIL("lower wrong"); return; }
    PASS();
}

static void test_normal_countdown(void) {
    TEST("Normal countdown (no wrap)");
    ltsp_pio_ts_t ts;
    ltsp_pio_ts_init(&ts);

    ltsp_pio_ts_extend(&ts, 0x80000000);
    ltsp_pio_ts_extend(&ts, 0x70000000); /* counting down, no wrap */

    if (ts.upper != 0) { FAIL("upper should be 0"); return; }
    if (ts.lower != 0x70000000) { FAIL("lower wrong"); return; }

    uint64_t v = ltsp_pio_ts_to_u64(&ts);
    if (v != 0x0000000070000000ULL) { FAIL("u64 wrong"); return; }
    PASS();
}

static void test_single_wrap(void) {
    TEST("Single wrap detection");
    ltsp_pio_ts_t ts;
    ltsp_pio_ts_init(&ts);

    /* Counter near zero, then wraps to near 0xFFFFFFFF */
    ltsp_pio_ts_extend(&ts, 0x00000010);
    ltsp_pio_ts_extend(&ts, 0xFFFFFFF0); /* new > prev → wrap */

    if (ts.upper != 1) { FAIL("upper not 1"); return; }
    if (ts.lower != 0xFFFFFFF0) { FAIL("lower wrong"); return; }
    PASS();
}

static void test_multiple_wraps(void) {
    TEST("Multiple wraps");
    ltsp_pio_ts_t ts;
    ltsp_pio_ts_init(&ts);

    ltsp_pio_ts_extend(&ts, 0x80000000);
    /* First wrap */
    ltsp_pio_ts_extend(&ts, 0x00000010);  /* counting down past 0 */
    ltsp_pio_ts_extend(&ts, 0xFFFFFFF0);  /* wrap detected */
    if (ts.upper != 1) { FAIL("wrap 1"); return; }

    /* Second wrap */
    ltsp_pio_ts_extend(&ts, 0x00000005);
    ltsp_pio_ts_extend(&ts, 0xFFFFFFFF);  /* wrap */
    if (ts.upper != 2) { FAIL("wrap 2"); return; }
    PASS();
}

static void test_diff_no_wrap(void) {
    TEST("Diff without wrap");
    ltsp_pio_ts_t a, b;
    ltsp_pio_ts_init(&a);
    ltsp_pio_ts_init(&b);

    /* a = earlier (higher value, counter counts down) */
    ltsp_pio_ts_extend(&a, 0x80000000);
    /* b = later (lower value) */
    ltsp_pio_ts_extend(&b, 0x70000000);

    int64_t diff = ltsp_pio_ts_diff(&a, &b);
    /* a - b should be positive (a is earlier = higher) */
    int64_t expected = (int64_t)0x80000000 - (int64_t)0x70000000;
    if (diff != expected) {
        printf("(diff=%lld expected=%lld) ", (long long)diff, (long long)expected);
        FAIL("diff wrong");
        return;
    }
    PASS();
}

static void test_diff_across_wrap(void) {
    TEST("Diff across wrap boundary");
    ltsp_pio_ts_t a, b;
    ltsp_pio_ts_init(&a);
    ltsp_pio_ts_init(&b);

    /* a: before wrap, upper=0, lower=0x00000100 */
    ltsp_pio_ts_extend(&a, 0x00000100);

    /* b: after wrap, upper=1, lower=0xFFFFFF00 */
    ltsp_pio_ts_extend(&b, 0x00000100);
    ltsp_pio_ts_extend(&b, 0xFFFFFF00); /* wrap */

    /* a(u=0, l=0x00000100) = 0x0000000000000100
     * b(u=1, l=0xFFFFFF00) = 0x00000001FFFFFF00
     * diff = a - b = negative (b is later in the extended domain,
     * but represents higher value due to wrap tracking) */
    int64_t diff = ltsp_pio_ts_diff(&a, &b);
    /* a_u64 = 0x100, b_u64 = 0x1FFFFFF00, diff = 0x100 - 0x1FFFFFF00 */
    int64_t expected = (int64_t)0x100 - (int64_t)0x1FFFFFF00LL;
    if (diff != expected) {
        printf("(diff=%lld expected=%lld) ", (long long)diff, (long long)expected);
        FAIL("diff wrong");
        return;
    }
    PASS();
}

static void test_ticks_to_ns(void) {
    TEST("Ticks to nanoseconds conversion");
    int64_t ns = ltsp_pio_ticks_to_ns(83333333);
    /* 83333333 * 12 = 999999996 ns ≈ 1 second */
    if (ns != 999999996LL) { FAIL("conversion wrong"); return; }
    PASS();
}

static void test_ns_to_ticks(void) {
    TEST("Nanoseconds to ticks conversion");
    int64_t ticks = ltsp_ns_to_pio_ticks(1000000000LL);
    /* 1e9 / 12 = 83333333 */
    if (ticks != 83333333LL) { FAIL("conversion wrong"); return; }
    PASS();
}

void test_pio_timestamp_run(void) {
    printf("\n=== PIO Timestamp Tests ===\n");
    test_init_and_first_sample();
    test_normal_countdown();
    test_single_wrap();
    test_multiple_wraps();
    test_diff_no_wrap();
    test_diff_across_wrap();
    test_ticks_to_ns();
    test_ns_to_ticks();
    printf("PIO Timestamp: %d/%d passed\n", tests_passed, tests_run);
}

int test_pio_timestamp_results(int *run, int *passed) {
    *run = tests_run; *passed = tests_passed;
    return tests_run - tests_passed;
}

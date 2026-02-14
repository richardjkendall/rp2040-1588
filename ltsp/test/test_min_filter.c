#include "../common/ltsp_min_filter.h"
#include <stdio.h>
#include <limits.h>

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) do { tests_run++; printf("  %-50s ", name); } while(0)
#define PASS() do { tests_passed++; printf("PASS\n"); } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); } while(0)

static void test_single_sample(void) {
    TEST("Single sample becomes minimum");
    int64_t buf[10];
    ltsp_min_filter_t f;
    ltsp_min_filter_init(&f, buf, 10);

    ltsp_min_filter_update(&f, 5000);
    if (!f.valid) { FAIL("not valid"); return; }
    if (ltsp_min_filter_get_min(&f) != 5000) { FAIL("wrong min"); return; }
    PASS();
}

static void test_decreasing(void) {
    TEST("Decreasing samples update minimum");
    int64_t buf[10];
    ltsp_min_filter_t f;
    ltsp_min_filter_init(&f, buf, 10);

    ltsp_min_filter_update(&f, 5000);
    ltsp_min_filter_update(&f, 3000);
    ltsp_min_filter_update(&f, 1000);
    if (ltsp_min_filter_get_min(&f) != 1000) { FAIL("wrong min"); return; }
    PASS();
}

static void test_increasing_keeps_min(void) {
    TEST("Increasing samples keep original minimum");
    int64_t buf[10];
    ltsp_min_filter_t f;
    ltsp_min_filter_init(&f, buf, 10);

    ltsp_min_filter_update(&f, 1000);
    ltsp_min_filter_update(&f, 2000);
    ltsp_min_filter_update(&f, 3000);
    if (ltsp_min_filter_get_min(&f) != 1000) { FAIL("min changed"); return; }
    PASS();
}

static void test_min_expiry(void) {
    TEST("Minimum expires after window samples");
    int64_t buf[5];
    ltsp_min_filter_t f;
    ltsp_min_filter_init(&f, buf, 5);

    /* Add minimum, then fill window with larger values */
    ltsp_min_filter_update(&f, 100);  /* This will be the min */
    ltsp_min_filter_update(&f, 500);
    ltsp_min_filter_update(&f, 500);
    ltsp_min_filter_update(&f, 500);
    ltsp_min_filter_update(&f, 500);
    /* min_age = 4, not yet expired (need >= 5) */
    if (ltsp_min_filter_get_min(&f) != 100) { FAIL("min expired too early"); return; }

    /* One more — now min_age reaches window, triggers full scan.
     * Buffer contains: [500, 500, 500, 500, 500] (100 was overwritten) */
    ltsp_min_filter_update(&f, 500);
    if (ltsp_min_filter_get_min(&f) != 500) { FAIL("min didn't update after expiry"); return; }
    PASS();
}

static void test_path_jump_detection(void) {
    TEST("Path jump detected on large min increase");
    int64_t buf[5];
    ltsp_min_filter_t f;
    ltsp_min_filter_init(&f, buf, 5);

    /* Establish a low minimum */
    ltsp_min_filter_update(&f, 100);
    /* Fill and expire it */
    for (int i = 0; i < 5; i++) {
        ltsp_min_filter_update(&f, 100000); /* 100us, well above threshold */
    }
    /* The min should have expired and a full scan found 100000.
     * 100000 - 100 = 99900 > 10000 threshold → path jump.
     * After reset, the filter re-adds the last sample. */
    if (ltsp_min_filter_get_min(&f) != 100000) { FAIL("wrong min after jump"); return; }
    PASS();
}

static void test_reset(void) {
    TEST("Reset clears state");
    int64_t buf[10];
    ltsp_min_filter_t f;
    ltsp_min_filter_init(&f, buf, 10);

    ltsp_min_filter_update(&f, 1000);
    ltsp_min_filter_reset(&f);
    if (f.valid) { FAIL("still valid after reset"); return; }
    if (f.count != 0) { FAIL("count not zero"); return; }
    PASS();
}

static void test_steady_state(void) {
    TEST("Steady state with occasional low packet");
    int64_t buf[LTSP_MIN_FILTER_DEFAULT_WINDOW];
    ltsp_min_filter_t f;
    ltsp_min_filter_init(&f, buf, LTSP_MIN_FILTER_DEFAULT_WINDOW);

    /* Simulate: mostly 5000ns with occasional 500ns "lucky packet" */
    for (int i = 0; i < 200; i++) {
        int64_t val = (i % 30 == 0) ? 500 : 5000;
        ltsp_min_filter_update(&f, val);
    }
    if (ltsp_min_filter_get_min(&f) != 500) { FAIL("missed lucky packet"); return; }
    PASS();
}

void test_min_filter_run(void) {
    printf("\n=== Minimum Filter Tests ===\n");
    test_single_sample();
    test_decreasing();
    test_increasing_keeps_min();
    test_min_expiry();
    test_path_jump_detection();
    test_reset();
    test_steady_state();
    printf("Min Filter: %d/%d passed\n", tests_passed, tests_run);
}

int test_min_filter_results(int *run, int *passed) {
    *run = tests_run; *passed = tests_passed;
    return tests_run - tests_passed;
}

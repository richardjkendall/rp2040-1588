#include "../common/ltsp_sequence.h"
#include <stdio.h>

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) do { tests_run++; printf("  %-50s ", name); } while(0)
#define PASS() do { tests_passed++; printf("PASS\n"); } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); } while(0)

static void test_first_packet(void) {
    TEST("First packet always accepted");
    ltsp_seq_state_t s;
    ltsp_seq_init(&s);
    uint16_t gap;
    ltsp_seq_result_t r = ltsp_seq_validate(&s, 100, &gap);
    if (r != LTSP_SEQ_ACCEPT) { FAIL("not accepted"); return; }
    if (s.accept_count != 1) { FAIL("count"); return; }
    PASS();
}

static void test_sequential(void) {
    TEST("Sequential packets accepted");
    ltsp_seq_state_t s;
    ltsp_seq_init(&s);
    uint16_t gap;
    ltsp_seq_validate(&s, 0, &gap);
    ltsp_seq_result_t r = ltsp_seq_validate(&s, 1, &gap);
    if (r != LTSP_SEQ_ACCEPT) { FAIL("seq 1"); return; }
    r = ltsp_seq_validate(&s, 2, &gap);
    if (r != LTSP_SEQ_ACCEPT) { FAIL("seq 2"); return; }
    PASS();
}

static void test_gap_1(void) {
    TEST("Gap of 1 (one lost packet)");
    ltsp_seq_state_t s;
    ltsp_seq_init(&s);
    uint16_t gap;
    ltsp_seq_validate(&s, 10, &gap);
    ltsp_seq_result_t r = ltsp_seq_validate(&s, 12, &gap);
    if (r != LTSP_SEQ_GAP) { FAIL("not GAP"); return; }
    if (gap != 1) { FAIL("gap size"); return; }
    if (s.gap_count != 1) { FAIL("gap_count"); return; }
    PASS();
}

static void test_gap_max(void) {
    TEST("Gap of SEQUENCE_GAP_MAX (3)");
    ltsp_seq_state_t s;
    ltsp_seq_init(&s);
    uint16_t gap;
    ltsp_seq_validate(&s, 10, &gap);
    ltsp_seq_result_t r = ltsp_seq_validate(&s, 14, &gap); /* 11,12,13 lost */
    if (r != LTSP_SEQ_GAP) { FAIL("not GAP"); return; }
    if (gap != 3) { FAIL("gap size"); return; }
    PASS();
}

static void test_restart(void) {
    TEST("Large gap triggers restart");
    ltsp_seq_state_t s;
    ltsp_seq_init(&s);
    uint16_t gap;
    ltsp_seq_validate(&s, 10, &gap);
    ltsp_seq_result_t r = ltsp_seq_validate(&s, 15, &gap); /* gap=4 > max=3 */
    if (r != LTSP_SEQ_RESTART) { FAIL("not RESTART"); return; }
    if (s.restart_count != 1) { FAIL("restart_count"); return; }
    PASS();
}

static void test_duplicate(void) {
    TEST("Backward sequence is duplicate");
    ltsp_seq_state_t s;
    ltsp_seq_init(&s);
    uint16_t gap;
    ltsp_seq_validate(&s, 10, &gap);
    ltsp_seq_validate(&s, 11, &gap);
    ltsp_seq_result_t r = ltsp_seq_validate(&s, 10, &gap);
    if (r != LTSP_SEQ_DUPLICATE) { FAIL("not DUPLICATE"); return; }
    PASS();
}

static void test_same_seq_is_duplicate(void) {
    TEST("Same sequence is duplicate");
    ltsp_seq_state_t s;
    ltsp_seq_init(&s);
    uint16_t gap;
    ltsp_seq_validate(&s, 10, &gap);
    ltsp_seq_result_t r = ltsp_seq_validate(&s, 10, &gap);
    if (r != LTSP_SEQ_DUPLICATE) { FAIL("not DUPLICATE"); return; }
    PASS();
}

static void test_wrap_normal(void) {
    TEST("Sequence wrap 65535 -> 0");
    ltsp_seq_state_t s;
    ltsp_seq_init(&s);
    uint16_t gap;
    ltsp_seq_validate(&s, 65535, &gap);
    ltsp_seq_result_t r = ltsp_seq_validate(&s, 0, &gap);
    if (r != LTSP_SEQ_ACCEPT) { FAIL("not ACCEPT at wrap"); return; }
    PASS();
}

static void test_wrap_gap(void) {
    TEST("Gap across wrap boundary");
    ltsp_seq_state_t s;
    ltsp_seq_init(&s);
    uint16_t gap;
    ltsp_seq_validate(&s, 65534, &gap);
    ltsp_seq_result_t r = ltsp_seq_validate(&s, 0, &gap); /* 65535 lost */
    if (r != LTSP_SEQ_GAP) { FAIL("not GAP"); return; }
    if (gap != 1) { FAIL("gap size"); return; }
    PASS();
}

static void test_wrap_restart(void) {
    TEST("Large gap across wrap boundary");
    ltsp_seq_state_t s;
    ltsp_seq_init(&s);
    uint16_t gap;
    ltsp_seq_validate(&s, 65530, &gap);
    ltsp_seq_result_t r = ltsp_seq_validate(&s, 0, &gap); /* gap=5 > max=3 */
    if (r != LTSP_SEQ_RESTART) { FAIL("not RESTART"); return; }
    PASS();
}

void test_sequence_run(void) {
    printf("\n=== Sequence Tests ===\n");
    test_first_packet();
    test_sequential();
    test_gap_1();
    test_gap_max();
    test_restart();
    test_duplicate();
    test_same_seq_is_duplicate();
    test_wrap_normal();
    test_wrap_gap();
    test_wrap_restart();
    printf("Sequence: %d/%d passed\n", tests_passed, tests_run);
}

int test_sequence_results(int *run, int *passed) {
    *run = tests_run; *passed = tests_passed;
    return tests_run - tests_passed;
}

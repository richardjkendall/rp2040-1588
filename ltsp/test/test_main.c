#include <stdio.h>

/* Test suite entry points */
extern void test_pdu_run(void);
extern int  test_pdu_results(int *run, int *passed);

extern void test_sequence_run(void);
extern int  test_sequence_results(int *run, int *passed);

extern void test_min_filter_run(void);
extern int  test_min_filter_results(int *run, int *passed);

extern void test_regression_run(void);
extern int  test_regression_results(int *run, int *passed);

extern void test_pio_timestamp_run(void);
extern int  test_pio_timestamp_results(int *run, int *passed);

int main(void) {
    printf("LTSP v0.1 Host-Side Test Suite\n");
    printf("==============================\n");

    test_pdu_run();
    test_sequence_run();
    test_min_filter_run();
    test_regression_run();
    test_pio_timestamp_run();

    /* Aggregate results */
    int total_run = 0, total_passed = 0;
    int run, passed;

    test_pdu_results(&run, &passed);
    total_run += run; total_passed += passed;

    test_sequence_results(&run, &passed);
    total_run += run; total_passed += passed;

    test_min_filter_results(&run, &passed);
    total_run += run; total_passed += passed;

    test_regression_results(&run, &passed);
    total_run += run; total_passed += passed;

    test_pio_timestamp_results(&run, &passed);
    total_run += run; total_passed += passed;

    printf("\n==============================\n");
    printf("TOTAL: %d/%d passed\n", total_passed, total_run);

    if (total_passed == total_run) {
        printf("ALL TESTS PASSED\n");
        return 0;
    } else {
        printf("FAILURES: %d\n", total_run - total_passed);
        return 1;
    }
}

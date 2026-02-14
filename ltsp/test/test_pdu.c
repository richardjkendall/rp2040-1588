#include "../common/ltsp_pdu.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) do { \
    tests_run++; \
    printf("  %-50s ", name); \
} while(0)

#define PASS() do { tests_passed++; printf("PASS\n"); } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); } while(0)

static void test_pack_unpack_roundtrip(void) {
    TEST("PDU pack/unpack round-trip");

    ltsp_pdu_t orig = {
        .version                 = LTSP_VERSION,
        .flags                   = LTSP_FLAG_CRYSTAL_CLASS,
        .sequence                = 12345,
        .prev_tx_timestamp       = 1234567890123456789LL,
        .model_epoch             = 987654321098765432LL,
        .source_phase_bias       = -42000000LL,
        .source_freq_drift       = 34.567f,
        .model_uncertainty       = 123.456f,
        .last_1pps_count         = 5555555555555555555LL,
        .last_1pps_interval      = 83333400,
        .gm_local_processing_mean = 150000,
    };

    uint8_t wire[LTSP_PDU_SIZE];
    ltsp_pdu_pack(&orig, wire);

    ltsp_pdu_t decoded;
    bool ok = ltsp_pdu_unpack(wire, &decoded);

    if (!ok) { FAIL("unpack returned false"); return; }
    if (decoded.version != orig.version) { FAIL("version"); return; }
    if (decoded.flags != orig.flags) { FAIL("flags"); return; }
    if (decoded.sequence != orig.sequence) { FAIL("sequence"); return; }
    if (decoded.prev_tx_timestamp != orig.prev_tx_timestamp) { FAIL("prev_tx_timestamp"); return; }
    if (decoded.model_epoch != orig.model_epoch) { FAIL("model_epoch"); return; }
    if (decoded.source_phase_bias != orig.source_phase_bias) { FAIL("source_phase_bias"); return; }
    if (fabsf(decoded.source_freq_drift - orig.source_freq_drift) > 0.001f) { FAIL("source_freq_drift"); return; }
    if (fabsf(decoded.model_uncertainty - orig.model_uncertainty) > 0.001f) { FAIL("model_uncertainty"); return; }
    if (decoded.last_1pps_count != orig.last_1pps_count) { FAIL("last_1pps_count"); return; }
    if (decoded.last_1pps_interval != orig.last_1pps_interval) { FAIL("last_1pps_interval"); return; }
    if (decoded.gm_local_processing_mean != orig.gm_local_processing_mean) { FAIL("gm_local_processing_mean"); return; }

    PASS();
}

static void test_byte_order(void) {
    TEST("PDU big-endian byte order");

    ltsp_pdu_t pdu = {0};
    pdu.version  = 1;
    pdu.sequence = 0x1234;

    uint8_t wire[LTSP_PDU_SIZE];
    ltsp_pdu_pack(&pdu, wire);

    /* Sequence 0x1234 should be at offset 2-3, big-endian */
    if (wire[2] != 0x12 || wire[3] != 0x34) {
        FAIL("sequence byte order");
        return;
    }

    PASS();
}

static void test_negative_a0(void) {
    TEST("PDU negative a0 round-trip");

    ltsp_pdu_t orig = { .version = 1 };
    orig.source_phase_bias = -999999999999LL;

    uint8_t wire[LTSP_PDU_SIZE];
    ltsp_pdu_pack(&orig, wire);

    ltsp_pdu_t decoded;
    ltsp_pdu_unpack(wire, &decoded);

    if (decoded.source_phase_bias != orig.source_phase_bias) {
        FAIL("negative a0 lost");
        return;
    }
    PASS();
}

static void test_version_reject(void) {
    TEST("PDU rejects unknown version");

    uint8_t wire[LTSP_PDU_SIZE] = {0};
    wire[0] = 99; /* bogus version */

    ltsp_pdu_t pdu;
    bool ok = ltsp_pdu_unpack(wire, &pdu);
    if (ok) { FAIL("should reject version 99"); return; }

    PASS();
}

static void test_zero_prev_tx(void) {
    TEST("PDU zero prev_tx_timestamp round-trip");

    ltsp_pdu_t orig = { .version = 1, .prev_tx_timestamp = 0 };
    uint8_t wire[LTSP_PDU_SIZE];
    ltsp_pdu_pack(&orig, wire);

    ltsp_pdu_t decoded;
    ltsp_pdu_unpack(wire, &decoded);
    if (decoded.prev_tx_timestamp != 0) { FAIL("should be zero"); return; }

    PASS();
}

static void test_frame_build_and_check(void) {
    TEST("Frame build + check round-trip");

    uint8_t src_mac[6] = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x01};
    ltsp_pdu_t pdu = {
        .version = 1,
        .sequence = 42,
        .prev_tx_timestamp = 1000000000LL,
    };

    uint8_t frame[LTSP_FRAME_SIZE];
    uint16_t len = ltsp_frame_build(frame, src_mac, &pdu);

    if (len != LTSP_FRAME_SIZE) { FAIL("wrong frame length"); return; }

    /* Check broadcast destination */
    for (int i = 0; i < 6; i++) {
        if (frame[i] != 0xFF) { FAIL("dst not broadcast"); return; }
    }

    /* Check source MAC */
    if (memcmp(frame + 6, src_mac, 6) != 0) { FAIL("src MAC"); return; }

    /* Check EtherType */
    if (frame[12] != 0x88 || frame[13] != 0xB5) { FAIL("ethertype"); return; }

    /* Parse back */
    const uint8_t *payload = ltsp_frame_check(frame, len);
    if (!payload) { FAIL("frame_check returned NULL"); return; }

    ltsp_pdu_t decoded;
    if (!ltsp_pdu_unpack(payload, &decoded)) { FAIL("unpack failed"); return; }
    if (decoded.sequence != 42) { FAIL("sequence mismatch"); return; }

    PASS();
}

static void test_frame_check_rejects_wrong_ethertype(void) {
    TEST("Frame check rejects wrong EtherType");

    uint8_t frame[LTSP_FRAME_SIZE] = {0};
    frame[12] = 0x08; frame[13] = 0x00; /* IPv4, not LTSP */

    const uint8_t *payload = ltsp_frame_check(frame, LTSP_FRAME_SIZE);
    if (payload != NULL) { FAIL("should reject IPv4"); return; }

    PASS();
}

static void test_frame_check_rejects_short(void) {
    TEST("Frame check rejects short frame");

    uint8_t frame[20] = {0};
    frame[12] = 0x88; frame[13] = 0xB5;

    const uint8_t *payload = ltsp_frame_check(frame, 20);
    if (payload != NULL) { FAIL("should reject short frame"); return; }

    PASS();
}

void test_pdu_run(void) {
    printf("\n=== PDU Tests ===\n");
    test_pack_unpack_roundtrip();
    test_byte_order();
    test_negative_a0();
    test_version_reject();
    test_zero_prev_tx();
    test_frame_build_and_check();
    test_frame_check_rejects_wrong_ethertype();
    test_frame_check_rejects_short();
    printf("PDU: %d/%d passed\n", tests_passed, tests_run);
}

int test_pdu_results(int *run, int *passed) {
    *run = tests_run;
    *passed = tests_passed;
    return tests_run - tests_passed;
}

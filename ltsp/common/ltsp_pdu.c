#include "ltsp_pdu.h"

/* --- Byte-order helpers (explicit, no dependency on arpa/inet.h) --- */

static inline void put_u16_be(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v);
}

static inline void put_u32_be(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v);
}

static inline void put_i64_be(uint8_t *p, int64_t v) {
    uint64_t u = (uint64_t)v;
    p[0] = (uint8_t)(u >> 56);
    p[1] = (uint8_t)(u >> 48);
    p[2] = (uint8_t)(u >> 40);
    p[3] = (uint8_t)(u >> 32);
    p[4] = (uint8_t)(u >> 24);
    p[5] = (uint8_t)(u >> 16);
    p[6] = (uint8_t)(u >> 8);
    p[7] = (uint8_t)(u);
}

static inline void put_float_be(uint8_t *p, float v) {
    uint32_t u;
    memcpy(&u, &v, 4);
    put_u32_be(p, u);
}

static inline uint16_t get_u16_be(const uint8_t *p) {
    return ((uint16_t)p[0] << 8) | p[1];
}

static inline uint32_t get_u32_be(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

static inline int64_t get_i64_be(const uint8_t *p) {
    uint64_t u = ((uint64_t)p[0] << 56) | ((uint64_t)p[1] << 48) |
                 ((uint64_t)p[2] << 40) | ((uint64_t)p[3] << 32) |
                 ((uint64_t)p[4] << 24) | ((uint64_t)p[5] << 16) |
                 ((uint64_t)p[6] << 8) | p[7];
    return (int64_t)u;
}

static inline float get_float_be(const uint8_t *p) {
    uint32_t u = get_u32_be(p);
    float v;
    memcpy(&v, &u, 4);
    return v;
}

/* --- PDU wire layout offsets ---
 *
 * Offset  Size  Field
 * 0       1     version
 * 1       1     flags
 * 2       2     sequence
 * 4       8     prev_tx_timestamp
 * 12      8     model_epoch
 * 20      8     source_phase_bias (a0)
 * 28      4     source_freq_drift (a1)
 * 32      4     model_uncertainty (sigma)
 * 36      8     last_1pps_count
 * 44      4     last_1pps_interval
 * 48      4     gm_local_processing_mean
 * 52      4     (padding to 56 bytes — reserved, zero)
 * Total: 56
 */

#define OFF_VERSION       0
#define OFF_FLAGS         1
#define OFF_SEQUENCE      2
#define OFF_PREV_TX       4
#define OFF_EPOCH         12
#define OFF_A0            20
#define OFF_A1            28
#define OFF_SIGMA         32
#define OFF_1PPS_COUNT    36
#define OFF_1PPS_INTERVAL 44
#define OFF_GM_LOCAL_MEAN 48
#define OFF_RESERVED      52

void ltsp_pdu_pack(const ltsp_pdu_t *pdu, uint8_t *buf) {
    buf[OFF_VERSION] = pdu->version;
    buf[OFF_FLAGS]   = pdu->flags;
    put_u16_be(buf + OFF_SEQUENCE, pdu->sequence);
    put_i64_be(buf + OFF_PREV_TX, pdu->prev_tx_timestamp);
    put_i64_be(buf + OFF_EPOCH, pdu->model_epoch);
    put_i64_be(buf + OFF_A0, pdu->source_phase_bias);
    put_float_be(buf + OFF_A1, pdu->source_freq_drift);
    put_float_be(buf + OFF_SIGMA, pdu->model_uncertainty);
    put_i64_be(buf + OFF_1PPS_COUNT, pdu->last_1pps_count);
    put_u32_be(buf + OFF_1PPS_INTERVAL, pdu->last_1pps_interval);
    put_u32_be(buf + OFF_GM_LOCAL_MEAN, pdu->gm_local_processing_mean);
    /* Reserved bytes — zero */
    buf[OFF_RESERVED]     = 0;
    buf[OFF_RESERVED + 1] = 0;
    buf[OFF_RESERVED + 2] = 0;
    buf[OFF_RESERVED + 3] = 0;
}

bool ltsp_pdu_unpack(const uint8_t *buf, ltsp_pdu_t *pdu) {
    pdu->version = buf[OFF_VERSION];
    if (pdu->version != LTSP_VERSION) {
        return false;
    }
    pdu->flags                    = buf[OFF_FLAGS];
    pdu->sequence                 = get_u16_be(buf + OFF_SEQUENCE);
    pdu->prev_tx_timestamp        = get_i64_be(buf + OFF_PREV_TX);
    pdu->model_epoch              = get_i64_be(buf + OFF_EPOCH);
    pdu->source_phase_bias        = get_i64_be(buf + OFF_A0);
    pdu->source_freq_drift        = get_float_be(buf + OFF_A1);
    pdu->model_uncertainty        = get_float_be(buf + OFF_SIGMA);
    pdu->last_1pps_count          = get_i64_be(buf + OFF_1PPS_COUNT);
    pdu->last_1pps_interval       = get_u32_be(buf + OFF_1PPS_INTERVAL);
    pdu->gm_local_processing_mean = get_u32_be(buf + OFF_GM_LOCAL_MEAN);
    return true;
}

uint16_t ltsp_frame_build(uint8_t *frame, const uint8_t *src_mac,
                          const ltsp_pdu_t *pdu) {
    /* Destination: broadcast */
    memset(frame, 0xFF, 6);
    /* Source MAC */
    memcpy(frame + 6, src_mac, 6);
    /* EtherType */
    put_u16_be(frame + 12, LTSP_ETHERTYPE);
    /* PDU payload */
    ltsp_pdu_pack(pdu, frame + LTSP_ETH_HEADER_SIZE);
    return LTSP_FRAME_SIZE;
}

const uint8_t *ltsp_frame_check(const uint8_t *frame, uint16_t frame_len) {
    if (frame_len < LTSP_FRAME_SIZE) {
        return NULL;
    }
    uint16_t ethertype = get_u16_be(frame + 12);
    if (ethertype != LTSP_ETHERTYPE) {
        return NULL;
    }
    return frame + LTSP_ETH_HEADER_SIZE;
}

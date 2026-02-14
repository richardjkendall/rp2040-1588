#ifndef LTSP_PDU_H
#define LTSP_PDU_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* LTSP Protocol Data Unit - draft-user-ltsp-rp2040-04 Section 5 */

#define LTSP_VERSION        1
#define LTSP_PDU_SIZE       56      /* bytes, without auth */
#define LTSP_PDU_SIZE_AUTH  72      /* bytes, with auth */
#define LTSP_ETHERTYPE      0x88B5  /* IEEE Local Experimental */
#define LTSP_ETH_HEADER_SIZE 14     /* dst(6) + src(6) + ethertype(2) */
#define LTSP_FRAME_SIZE     (LTSP_ETH_HEADER_SIZE + LTSP_PDU_SIZE)

/* Flags (Section 5) */
#define LTSP_FLAG_AUTH          (1 << 0)
#define LTSP_FLAG_HOLDOVER      (1 << 1)
#define LTSP_FLAG_LEAP_PENDING  (1 << 2)
#define LTSP_FLAG_CRYSTAL_CLASS (1 << 3)

/* PDU structure (host byte order) */
typedef struct {
    uint8_t  version;                   /* Protocol version (must be 1) */
    uint8_t  flags;                     /* LTSP_FLAG_* bits */
    uint16_t sequence;                  /* Packet sequence number */
    int64_t  prev_tx_timestamp;         /* GPS ns: TX time of packet N-1 (deferred) */
    int64_t  model_epoch;               /* GPS ns: reference point for a0, a1 */
    int64_t  source_phase_bias;         /* ns: GM phase offset from GPS at epoch (a0) */
    float    source_freq_drift;         /* ppb: GM frequency error at epoch (a1) */
    float    model_uncertainty;         /* ns: regression residual stddev (Sigma) */
    int64_t  last_1pps_count;           /* GPS ns: most recent 1PPS edge */
    uint32_t last_1pps_interval;        /* PIO ticks: ticks between last two 1PPS edges */
    uint32_t gm_local_processing_mean;  /* ns: mean GM SPI-to-INTn delay */
} ltsp_pdu_t;

/*
 * Pack an ltsp_pdu_t into wire format (big-endian).
 * buf must be at least LTSP_PDU_SIZE bytes.
 */
void ltsp_pdu_pack(const ltsp_pdu_t *pdu, uint8_t *buf);

/*
 * Unpack wire bytes (big-endian) into an ltsp_pdu_t (host byte order).
 * buf must be at least LTSP_PDU_SIZE bytes.
 * Returns false if version is not recognized.
 */
bool ltsp_pdu_unpack(const uint8_t *buf, ltsp_pdu_t *pdu);

/*
 * Build a complete LTSP Ethernet frame.
 * frame must be at least LTSP_FRAME_SIZE bytes.
 * Returns total frame length.
 */
uint16_t ltsp_frame_build(uint8_t *frame, const uint8_t *src_mac,
                          const ltsp_pdu_t *pdu);

/*
 * Check if a received Ethernet frame is an LTSP frame.
 * Returns pointer to PDU payload if EtherType matches, NULL otherwise.
 * frame_len must be >= LTSP_FRAME_SIZE.
 */
const uint8_t *ltsp_frame_check(const uint8_t *frame, uint16_t frame_len);

#endif /* LTSP_PDU_H */

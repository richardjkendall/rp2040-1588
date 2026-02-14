#include "ltsp_sequence.h"

void ltsp_seq_init(ltsp_seq_state_t *s) {
    s->last_received  = 0;
    s->initialized    = false;
    s->accept_count   = 0;
    s->gap_count      = 0;
    s->restart_count  = 0;
    s->duplicate_count = 0;
}

ltsp_seq_result_t ltsp_seq_validate(ltsp_seq_state_t *s, uint16_t seq,
                                    uint16_t *gap_size) {
    if (gap_size) *gap_size = 0;

    /* First packet after init — always accept */
    if (!s->initialized) {
        s->last_received = seq;
        s->initialized   = true;
        s->accept_count++;
        return LTSP_SEQ_ACCEPT;
    }

    uint16_t expected = s->last_received + 1; /* wraps naturally at 16 bits */

    if (seq == expected) {
        s->last_received = seq;
        s->accept_count++;
        return LTSP_SEQ_ACCEPT;
    }

    /* Forward gap: seq - expected, with 16-bit wraparound */
    uint16_t forward = seq - expected; /* unsigned subtraction wraps correctly */

    if (forward >= 1 && forward <= LTSP_SEQUENCE_GAP_MAX) {
        if (gap_size) *gap_size = forward;
        s->gap_count += forward;
        s->last_received = seq;
        s->accept_count++;
        return LTSP_SEQ_GAP;
    }

    /* Check for backward / duplicate: if forward > 32767, it's effectively
     * a backward step (two's complement of 16-bit unsigned) */
    if (forward > 32767) {
        s->duplicate_count++;
        return LTSP_SEQ_DUPLICATE;
    }

    /* Large forward gap — protocol restart */
    s->restart_count++;
    s->last_received = seq;
    s->accept_count++;
    if (gap_size) *gap_size = forward;
    return LTSP_SEQ_RESTART;
}

#ifndef LTSP_SEQUENCE_H
#define LTSP_SEQUENCE_H

#include <stdint.h>
#include <stdbool.h>

/* Sequence validation results */
typedef enum {
    LTSP_SEQ_ACCEPT,    /* Expected sequence, process normally */
    LTSP_SEQ_GAP,       /* 1-SEQUENCE_GAP_MAX packets lost, accept */
    LTSP_SEQ_RESTART,   /* Gap too large, protocol restart */
    LTSP_SEQ_DUPLICATE, /* Sequence went backward, discard */
} ltsp_seq_result_t;

#define LTSP_SEQUENCE_GAP_MAX 3

typedef struct {
    uint16_t last_received;     /* Last accepted sequence number */
    bool     initialized;       /* Have we received any packet? */
    uint32_t accept_count;      /* Total accepted packets */
    uint32_t gap_count;         /* Packets lost (sum of all gaps) */
    uint32_t restart_count;     /* Protocol restarts */
    uint32_t duplicate_count;   /* Duplicates discarded */
} ltsp_seq_state_t;

void ltsp_seq_init(ltsp_seq_state_t *s);

/*
 * Validate incoming sequence number.
 * On GAP, gap_size is set to the number of missing packets.
 */
ltsp_seq_result_t ltsp_seq_validate(ltsp_seq_state_t *s, uint16_t seq,
                                    uint16_t *gap_size);

#endif /* LTSP_SEQUENCE_H */

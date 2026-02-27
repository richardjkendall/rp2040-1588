# LTSP Holdover Fixes — v1.0

Test 3 (90-second cable disconnect) exposed three bugs that cause catastrophic
behavior during extended outages. This document describes each bug, the root
cause, and the fix.

## Test 3 Results (reference: `20260227_100012`)

- Cable pulled at ~t=1796s, reconnected at ~t=1886s (90s outage)
- PPS stopped at t=1846s (50s into outage) — PIO anchor too stale
- After reconnect: regression produced drift = -393M ppm, scale_factor = 2.29
- PPS resumed 383s after reconnect; full recovery at 669s
- Post-recovery performance: sigma 4.5 us (no permanent degradation)

---

## Fix 1: State Machine — Holdover Timer on Core 0

### Bug

`ltsp_clock_update_state()` is only called from `on_ltsp_packet()` in the W5500
IRQ callback (Core 1). It always passes `has_packet=true` (main.c:254):

```c
ltsp_clock_update_state(&clock_state, true, gm_holdover, ...);
```

The HOLDOVER transition in `ltsp_clock.c:185-189` requires `!has_packet`:

```c
} else if (!has_packet && st->last_packet_us > 0 &&
           (now_us - st->last_packet_us) > LTSP_HOLDOVER_TIMEOUT_US) {
```

Since the function is never called without a packet, HOLDOVER is never
triggered by timeout — only by the GM holdover flag.

### Fix

Add a periodic `ltsp_clock_update_state()` call in the Core 0 loop (which runs
every 1ms regardless of packet arrival). Core 0 already reads `clock_state`
fields; calling the state update is safe because Core 1 only writes to
`clock_state` inside `on_ltsp_packet`, and the state machine updates are not
timing-critical.

**In Core 0 loop (main.c, after drain_hw_timestamp_fifo):**

```c
// Check holdover timeout independently of packet arrival
// This runs every 1ms; the state machine internally checks elapsed time
ltsp_clock_update_state(&clock_state, false, false, clock_state.last_clock_error_ns);
```

Also disable PPS when entering HOLDOVER. Add to the Core 0 enable/disable
block (main.c:569-582):

```c
// Disable 1PPS on HOLDOVER
if (pps_enabled && clock_state.state == LTSP_SYNC_HOLDOVER) {
    pps_enabled = false;
    pps_anchor_valid = false;
    printf("# 1PPS: Disabled (HOLDOVER)\n");
}
```

**Effect:** After 30s without packets, state transitions LOCKED -> HOLDOVER.
PPS stops cleanly. After 60s, HOLDOVER -> INIT (clock_valid = false).

---

## Fix 2: Reset Regression and Min Filter After Gap

### Bug

After a gap/restart, the gap packet correctly skips the measurement pipeline
(main.c:263-265). However:

1. `drift_reg` retains 60 stale samples from before the outage
2. `drift_sample_count` continues from its old value
3. `d_total_ref_ns` / `d_total_ref_set` reference the pre-outage baseline
4. `min_filter` retains stale minimum values

When normal packets resume, new `d_total_rel` values are added to a regression
window full of stale data. With `drift_sample_count` continuing from ~1800,
the regression fits a line through data at indices 1800 and 1860+ but with a
90s gap — producing catastrophically wrong drift (-393M ppm, scale_factor 2.29).

The corrupted `scale_factor` (2.29) doubles the PIO-to-GPS conversion, making
`gps_now` jump forward by millions of ns. Even with PPS reseed, the filter
converges on garbage for ~130s until the regression window fully flushes.

### Fix

Reset the regression, min filter, sample counter, and reference baseline when
a gap/restart is detected. Add to the gap handler (main.c:233-243):

```c
if (seq_result == LTSP_SEQ_GAP || seq_result == LTSP_SEQ_RESTART) {
    seq_gap_count += gap_size;
    // Reset adaptive alpha for fast reconvergence after outage
    if (pps_filter_initialized) {
        pps_filter_count = 0;
        pps_filter_reseed = true;
        printf("# ALPHA RESET: seq %s=%u, restarting adaptive alpha, reseed PPS\n",
               seq_result == LTSP_SEQ_GAP ? "gap" : "restart", gap_size);
    }

    // NEW: Reset measurement pipelines — stale regression/min filter
    // data from before the outage would corrupt drift for ~130s
    ltsp_regression_init(&drift_reg, drift_reg_buf, LTSP_REGRESSION_DEFAULT_WINDOW);
    ltsp_min_filter_reset(&min_filter);
    drift_sample_count = 0;
    d_total_ref_set = false;
    printf("# PIPELINE RESET: regression, min filter, sample counter\n");
}
```

**Effect:** After reconnect, regression starts fresh. First valid drift estimate
at 2 samples (~2s), full window at 60 samples (~60s). No corrupted scale_factor.

**Interaction with PPS reseed:** The reseed fires on the first post-gap packet
that reaches the detrend_valid block (after 60 samples). During the 60-sample
ramp-up, `detrend_valid` is false so the PPS anchor is NOT updated. This means
Core 0 coasts on the pre-holdover anchor for ~60s. This is acceptable because:
- The state will be HOLDOVER (Fix 1), so PPS is already disabled
- When regression stabilizes at 60 samples, the first PPS anchor update
  triggers the reseed with a clean `gps_now`

Wait — actually the reseed flag would be consumed by the first packet that
enters the `clock_initialized` else branch (line 324+), but that branch is
inside the `detrend_valid` block. So the reseed will fire at sample 60, which
is correct. But we should verify: does `clock_initialized` stay true across
the gap? Yes — it's a one-shot flag (line 101), never reset. Good.

**State machine interaction:** When state goes HOLDOVER -> ACQUIRING -> LOCKED,
the PPS enable check (Core 0) will re-enable PPS at ACQUIRING (with valid clock),
which is correct.

---

## Fix 3: PPS Coast / Freeze During Holdover

### Bug

During an outage, Core 0 continues scheduling PPS from an increasingly stale
anchor. After ~50s, the accumulated crystal drift (~2 us/s * 50s = 100 us)
pushes the PPS prediction far enough that scheduling fails (the computed tick
count wraps or falls outside the scheduling window). PPS stops abruptly and
doesn't resume until a new anchor arrives.

### Fix

This is **already handled by Fix 1**. When the state transitions to HOLDOVER
at 30s, the Core 0 PPS disable block (added in Fix 1) sets `pps_enabled = false`
and `pps_anchor_valid = false`. PPS stops cleanly at 30s instead of
crashing at ~50s.

For belt-and-suspenders, we could also add an anchor staleness check:

```c
// Optional: disable PPS if anchor is too old (backup for Fix 1)
if (pps_enabled && pps_anchor_valid) {
    uint32_t elapsed_ticks = pps_anchor_counter - read_counter();
    double elapsed_s = (double)elapsed_ticks * 12.0 / 1e9;
    if (elapsed_s > 45.0) {  // Before the ~50s crash point
        pps_anchor_valid = false;
        printf("# 1PPS: Anchor stale (%.1f s)\n", elapsed_s);
    }
}
```

**Recommendation:** Implement Fix 1's HOLDOVER-based PPS disable. Skip the
staleness check unless we find a case where Fix 1 alone isn't sufficient.

---

## Implementation Order

1. **Fix 2** (regression reset) — Most impactful, prevents the 130s of garbage
2. **Fix 1** (holdover timer + PPS disable) — Clean state transitions
3. **Fix 3** — Already covered by Fix 1

## Expected Behavior After Fixes

### 90-second cable pull:
1. t=0: Cable pulled, no packets arrive
2. t=30: State LOCKED -> HOLDOVER, PPS disabled cleanly
3. t=60: State HOLDOVER -> INIT, clock_valid = false
4. t=90: Cable reconnected, first packet arrives
5. t=90: Sequence gap detected -> regression/min filter/counter reset
6. t=90: State INIT -> INIT (has_packet, counting to 3)
7. t=93: State INIT -> ACQUIRING (3 packets)
8. t=150: Regression has 60 samples, detrend_valid = true
9. t=150: Clock initialized (or re-initialized?), PPS reseed fires
10. t=155: State ACQUIRING -> LOCKED (5 consecutive good errors)
11. t=155: PPS enabled, first pulse within ~1s

**Total PPS outage: ~125s** (vs 423s in Test 3)
**No corrupted scale_factor** (vs 2.29 in Test 3)

### One concern — clock re-initialization

After HOLDOVER -> INIT, `clock_valid` is set to false (ltsp_clock.c:203).
But `clock_initialized` (main.c:101) is a one-shot — never reset. This means:
- After returning from INIT -> ACQUIRING, the code takes the `else` branch
  (line 324), not the init branch (line 312)
- The PPS filter will be in reseed mode, which is correct
- But `ltsp_clock_advance()` will be called with an invalid `clock_state`

**Additional fix needed:** Reset `clock_initialized = false` in the gap handler
so the clock gets re-initialized from the first valid `gps_now` after recovery:

```c
// In gap handler, after pipeline reset:
clock_initialized = false;
```

This ensures `ltsp_clock_set_initial()` is called again, which sets
`clock_valid = true` and seeds the PPS filter fresh.

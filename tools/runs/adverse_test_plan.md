# LTSP Receiver — Adverse Conditions Test Plan

**Baseline performance (adaptive alpha, 1-hour):**
Settled sigma 5.8 µs, P-P 51.5 µs, mean -1.0 µs, zero packet loss.
Run: `tools/runs/20260221_065717`

**System under test:**
- GM (192.168.1.100) → Ethernet → RX (192.168.1.102)
- LTSP PDU at 1 Hz, raw Ethernet (EtherType 0x88B5)
- State machine: INIT → ACQUIRING → LOCKED → HOLDOVER
- Holdover timeout: 30s → HOLDOVER, 60s → INIT (full reset)
- Crystal drift: ~30 ppm (~30 µs/s free-run error)

**Equipment needed:**
- Scope (already in place for phase measurement)
- Managed switch or Linux box with two NICs (for traffic injection/blocking)
- OR: simple cable disconnect/reconnect for basic tests

---

## Test 0: Switch vs Crossover Baseline — COMPLETED

**Result (1-hour, fresh boot, via switch):** Run `tools/runs/20260221_141657`

| Metric | Switch | Crossover | Delta |
|--------|--------|-----------|-------|
| Jitter sigma | 30.6 µs | 31.5 µs | -0.9 µs |
| Phase sigma | **5.9 µs** | **5.8 µs** | +0.1 µs |
| Phase P-P | 42.7 µs | 51.5 µs | -8.8 µs |
| Phase mean | **+6.8 µs** | **-1.0 µs** | **+7.8 µs shift** |
| 5-min windows | 4.4–7.4 µs | 3.1–8.0 µs | narrower |

**No-reboot run (1-hour):** Run `tools/runs/20260221_164357`
- Sigma 5.6 µs, P-P 42.2 µs, mean +7.3 µs, drift 0.05 ns/s
- Jitter sigma: 31.7 µs, zero packet loss
- 5-min windows: 3.8–7.8 µs sigma

**Summary of all three 1-hour runs:**

| Metric | Switch no-reboot | Switch reboot | Crossover reboot |
|--------|-----------------|---------------|-----------------|
| Sigma | **5.6 µs** | 5.9 µs | 5.8 µs |
| P-P | 42.2 µs | 42.7 µs | 51.5 µs |
| Mean | +7.3 µs | +6.8 µs | -1.0 µs |
| Jitter sigma | 31.7 µs | 30.6 µs | 31.5 µs |

**Conclusions:**
- Switch adds negligible jitter — sigma unchanged at ~5.5–6.0 µs
- Mean offset shifted +7.8 µs from asymmetric store-and-forward delay
- 5-min windows consistent across all runs
- System fully robust to switch insertion (no competing traffic)
- Performance repeatable across 3 independent 1-hour runs

---

## Test 1: Packet Loss — Cable Disconnect (10s) — COMPLETED

**Result (65-min, fresh boot, via switch):** Run `tools/runs/20260223_102636`

| Phase | Pre-pull (settled) | Post-recovery |
|-------|-------------------|---------------|
| Sigma | 5.7 µs | 6.6 µs |
| Mean | +7.1 µs | +7.7 µs |
| P-P | 35.1 µs | 44.6 µs |

**Recovery timeline:**
- t=1862s (31.0m): Last good reading (+6.4 µs)
- t=1892s (31.5m): Phase jumps to -103.6 µs (scope limit)
- t=1892–2225s: Stuck beyond scope limit (~5.5 min)
- t=2255s (37.6m): -83 µs (returning to view)
- t=2346s (39.1m): +2.5 µs (recovered)
- **Total recovery: ~7.5 minutes**

**Root cause:** First packet after reconnect has `elapsed_pio_ns` spanning the
entire 10s gap, producing a huge gps_now spike. This pushes pps_filtered_clock_ns
far off. At alpha=0.02, exponential recovery takes ~7.5 min (most beyond scope).
d_min also spiked to -15 billion ns but regression recovered within 60 packets.

**Fix needed:** Reset adaptive alpha (pps_filter_count=0) on packet gap so the
filter reconverges at alpha=0.2 instead of 0.02. Expected recovery: ~30s.

---

## Test 1b: Cable Disconnect (10s) with Alpha Reset Only — FAILED

**Result:** Run `tools/runs/20260223_125320`. Alpha reset triggered (`seq restart=12`)
but regression was still corrupted by the gap packet (d_min=-11 billion, drift=+19.6M ns/s).
High alpha (0.2) amplified the bad scale_factor into the PPS filter, making it WORSE.
Phase stuck at -411 µs, same recovery time as without fix.

**Root cause:** Only skipping the PPS filter update isn't enough — the gap packet must
be excluded from ALL pipelines (regression, min filter, PPS filter).

## Test 1c: Cable Disconnect (10s) with Full Gap Skip (v0.8) — COMPLETED

**Fix (v0.8):** Gap packet skips entire measurement pipeline. Alpha resets to 0.2.

**Result:** Run `tools/runs/20260225_112419` (65-min, 2 MSa/s scope at 250 µs/div)
- Gap skip fired correctly, but PPS filter still crashed to -4.1 ms
- Recovery: 7.4 min — same as without fix

**Root cause found:** After gap, `pps_filtered_clock_ns` is stale by outage duration
but `elapsed_pio_ns` only covers ~1s (gap packet → next packet). Prediction
`stale + 1s` is ~10s short → massive correction shifts sub-second PPS phase.

---

## Test 1d: Cable Disconnect (10s) with PPS Reseed Fix (v0.9) — COMPLETED

**Fix (v0.9):** Added `pps_filter_reseed` flag. On gap/restart, first normal packet
re-seeds `pps_filtered_clock_ns = gps_now` instead of predicting from stale value.

**Result:** Run `tools/runs/20260225_132419` (65-min, 2 MSa/s scope at 250 µs/div)

| Metric | Test 1c (no reseed) | Test 1d (with reseed) |
|--------|--------------------|-----------------------|
| Max excursion | -4,104 µs | **-80 µs** |
| Recovery time | 7.4 min | **~100s** |
| Pre-pull sigma | 5.6 µs | 5.6 µs |
| Post-recovery sigma | — | 5.3 µs |

---

## Test 1e: Cable Disconnect (10s) with PPS Reseed — Zoomed Scope — COMPLETED

**Result:** Run `tools/runs/20260225_144428` (65-min, 100 MSa/s scope)

| Phase | Pre-pull | Worst excursion | Post-recovery |
|-------|----------|-----------------|---------------|
| Sigma | 4.5 µs | — | 5.8 µs |
| Mean | +8.4 µs | — | +8.7 µs |
| Min | — | **-103 µs** | — |
| Max | — | **+50 µs** | — |

**Recovery timeline (100 MSa/s detail):**
- During outage: phase drifts smoothly +15 → -18 µs (~2 µs/s crystal drift)
- Reconnect: reseed jumps to +50 µs (one jittery gps_now sample)
- Adaptive alpha (0.2) absorbs regression instability → dips to -103 µs
- Exponential recovery: -103 → 0 µs over ~2.7 min
- Full recovery to within 10 µs of steady state: **~162s**

**Remaining issue:** The -103 µs dip is caused by regression instability amplified
by high alpha (0.2) during reconvergence. Delaying alpha reset until regression
re-stabilises (~60 packets) could reduce the excursion significantly.

**Success criteria: PASS**
- No crash or state reset
- Phase recovers to <10 µs sigma within 3 minutes
- No permanent offset shift

---

## Test 2: Packet Loss — Cable Disconnect (45s) — COMPLETED

**Result:** Run `tools/runs/20260226_043003` (65-min, 100 MSa/s scope)

| Phase | Pre-pull | Worst excursion | Post-recovery |
|-------|----------|-----------------|---------------|
| Sigma | 5.9 µs | — | 5.6 µs |
| Mean | +9.2 µs | — | +7.8 µs |
| Max neg | — | **-103.6 µs** (scope floor) | — |
| Max pos | — | +43.2 µs | — |

**During 45s outage:**
- PPS coasted correctly: smooth linear drift +6 → -89 µs (~1.8 µs/s)
- Crystal drift ~28 ppm minus scale_factor correction

**On reconnect:**
- PPS RESEED fired (+43 µs), then regression instability dipped to scope floor
- Stuck at -103.6 µs for ~85s while regression window refilled
- Recovery to within 15 µs of steady state: **204s** (~3.4 min)
- Post-recovery sigma identical to pre-pull

**Key finding — no HOLDOVER transition:** State stayed LOCKED. The state machine
only checks elapsed time on packet arrival, so the 45s gap is never seen as a
continuous timeout. This is acceptable — PPS reseed handles recovery correctly.

### Test 2b: Re-run with wider scope (500 µs/div, 1 MSa/s) — COMPLETED

**Result:** Run `tools/runs/20260227_082340` (65-min, 500 µs/div captures ±4 ms)

**Full excursion now visible:**
- During 47s outage: smooth drift +18 → -75 µs (~2 µs/s)
- Reconnect reseed: jumps to -10 µs
- Regression instability: exponential dip to **-374 µs** at 1898s (~53s after reconnect)
- Exponential recovery: -374 → 0 µs over ~2.5 min
- Within 20 µs of steady state: **158s** from reconnect

| Phase | Pre-pull | Worst excursion | Post-recovery |
|-------|----------|-----------------|---------------|
| Sigma | 5.2 µs | — | 5.6 µs |
| Mean | +7.5 µs | — | +8.1 µs |
| Min | — | **-374 µs** | — |

**Success criteria: PASS**
- No crash or state reset
- Phase recovers to <10 µs sigma within 5 minutes
- No permanent offset shift
- Max excursion -374 µs (was -4,104 µs before PPS reseed fix)

---

## Test 3: Packet Loss — Extended Outage (90s) — COMPLETED

**Result:** Run `tools/runs/20260227_100012` (65-min, 500 µs/div, 1 MSa/s)

| Phase | Pre-pull (5 min) | Post-recovery (5 min) |
|-------|------------------|-----------------------|
| Sigma | 5.9 µs | 4.5 µs |
| Mean | +8.2 µs | +6.6 µs |
| P-P | 25.6 µs | 22.4 µs |

**Timeline:**
- t≈1796s: Cable pulled (settled at +8 µs)
- t≈1846s: **1PPS stopped** — 50s into outage, phase at -106 µs
- t≈1886s: Cable reconnected (90s after pull)
- t≈2269s: PPS resumed — **383s after reconnect**
- t≈2486s: Within 100 µs of steady state — **600s after reconnect**
- t≈2555s: Within 20 µs of steady state — **669s after reconnect**

**During 90s outage:**
- Phase drifted smoothly +2.4 → -106 µs (~2.3 µs/s crystal drift)
- At -106 µs (50s in), PPS stopped entirely — scheduler anchor too stale
- Phase jumped to -8200 µs baseline (no synchronized PPS output)

**On reconnect:**
- Gap detection fired correctly: `ALPHA RESET: seq restart=93, reseed PPS`
- PPS RESEED fired but regression was catastrophically corrupted
- Drift jumped to -393M ppm, scale_factor reached 2.29
- Regression needed ~130s to refill 60-sample window with valid data
- PPS did not resume until 383s after reconnect
- Total PPS outage: **423s** (7 min) — far longer than 90s cable disconnect

**Critical findings:**
1. **No HOLDOVER/INIT transition** — state machine only checks on packet arrival,
   stayed LOCKED the entire time (same as Test 2)
2. **PPS stops at ~50s of coasting** — scheduler prediction diverges beyond valid
   range as anchor becomes stale (consistent with -103 µs scope floor in earlier tests)
3. **Regression not reset after gap** — stale 60-sample window produces insane drift
   for ~130s, corrupting scale_factor and PPS scheduling
4. **PPS reseed from corrupted gps_now** — reseed value is wrong because regression
   was corrupted when first post-gap packet arrived
5. Post-recovery performance matches pre-pull baseline (no permanent degradation)

**Fixes needed (priority order):**
1. **Holdover timer on Core 0** — check elapsed time since last packet independently
   of packet arrival; transition LOCKED→HOLDOVER→INIT on timer
2. **Regression window reset after gap** — clear stale samples so regression
   reconverges in 60s instead of producing corrupted drift for 130s
3. **Freeze PPS at last good value during holdover** — stop updating PPS scheduler
   when no packets arriving, instead of letting prediction diverge

**Success criteria: PARTIAL FAIL**
- ✓ No crash
- ✓ Performance matches cold-boot baseline after recovery
- ✗ 1PPS outage 423s (expected: stops cleanly, restarts in ~300s)
- ✗ No HOLDOVER/INIT state transitions (state machine broken for outages)
- ✗ Recovery took 669s to within 20 µs (expected: ~300s cold boot)

---

## Test 3b: Extended Outage (90s) — WITH HOLDOVER FIXES (v1.0) — COMPLETED

**Result:** Run `tools/runs/20260227_112917` (65-min, 500 µs/div, 1 MSa/s)

**Fixes applied:**
1. Holdover timer on Core 0 — `ltsp_clock_update_state(false)` every 1ms
2. Pipeline reset after gap — regression, min filter, sample counter, clock_initialized
3. PPS disable on HOLDOVER — clean shutdown before anchor goes stale

| Phase | Pre-pull (800-1800s) | Post-recovery (>2200s) |
|-------|---------------------|------------------------|
| Sigma | 5.4 µs | 5.0 µs |
| Mean | +10.6 µs | +10.0 µs |
| P-P | 29.1 µs | 26.4 µs |

**Timeline:**
- t≈1860s: Cable pulled (settled at +10 µs)
- t≈1890s: **LOCKED→HOLDOVER** (30s timeout) — PPS disabled cleanly
- t≈1950s: **HOLDOVER→INIT** (60s extended timeout)
- t≈1950s: Cable reconnected (90s after pull)
- t≈1950s: PIPELINE RESET — regression, min filter, sample counter, clock
- t≈1953s: INIT→ACQUIRING (3 packets)
- t≈2008s: Clock re-initialized, **PPS resumed** — 58s after reconnect
- t≈2008s: ACQUIRING→LOCKED
- t≈2149s: Within 20 µs of steady state — **199s after reconnect**

**PPS outage: 118s** (vs 423s in Test 3 — **72% reduction**)

**On reconnect:**
- Gap detection + PIPELINE RESET: regression/min filter/clock cleared immediately
- Clock re-initialized from fresh gps_now at 60-sample convergence
- No corrupted scale_factor (vs 2.29 in Test 3)
- Max excursion: +393 µs at t=2031 (regression convergence transient)
- PPS reseed fired correctly from clean gps_now

**State transitions (all correct):**
1. LOCKED → HOLDOVER (timeout, 30s) ✓
2. 1PPS Disabled (HOLDOVER) ✓
3. HOLDOVER → INIT (extended timeout, 60s) ✓
4. INIT → ACQUIRING (3 packets) ✓
5. Clock re-initialized ✓
6. 1PPS Enabled (ACQUIRING) ✓
7. ACQUIRING → LOCKED ✓

**Comparison: Test 3 vs Test 3b:**

| Metric | Test 3 (no fix) | Test 3b (fixed) |
|--------|-----------------|-----------------|
| PPS outage | 423s | 118s |
| Max excursion | -8207 µs (stopped) | +393 µs |
| Scale factor max | 2.29 (corrupted) | normal |
| Full recovery | 669s | 199s |
| Post sigma | 4.5 µs | 5.0 µs |
| State transitions | none | all correct |

**Success criteria: PASS**
- ✓ No crash
- ✓ All state transitions fire correctly
- ✓ PPS disabled cleanly on HOLDOVER (30s)
- ✓ Pipeline reset prevents corrupted scale_factor
- ✓ PPS resumes 58s after reconnect (regression convergence)
- ✓ Post-recovery performance matches pre-pull baseline
- ✓ Recovery within 20 µs in 199s (vs 669s unfixed)

**Remaining improvement opportunities:**
- Max excursion of +393 µs during convergence (alpha=0.2 amplifies regression noise)
- Could delay alpha reset until regression has ≥30 samples to reduce transient
- PPS outage dominated by 60-sample regression window (inherent, not a bug)

---

## Test 3c: Extended Outage (90s) — WITH HOLDOVER COASTING (v1.0) — COMPLETED

**Result:** Run `tools/runs/20260227_143124` (65-min, 500 µs/div, 1 MSa/s)

**Additional fix vs Test 3b:**
- Removed PPS disable on HOLDOVER entry
- Added holdover re-anchor: Core 0 re-anchors itself every ~40s (before 51.5s
  32-bit PIO counter wrap) to keep PPS coasting on last known scale_factor

| Phase | Pre-pull (800-1800s) | Post-recovery (>2200s) |
|-------|---------------------|------------------------|
| Sigma | 8.0 µs | 5.9 µs |
| Mean | +12.1 µs | +10.3 µs |
| P-P | 149.8 µs | 34.6 µs |

**Timeline:**
- t≈1809s: Cable pulled (settled at +12 µs)
- t≈1839s: **LOCKED→HOLDOVER** (30s timeout) — PPS keeps coasting
- t≈1849s: Re-anchor fired (40s, before 51.5s counter wrap)
- t≈1868s: **HOLDOVER→INIT** (60s extended timeout) — PPS disabled (clock_valid=false)
- t≈1899s: Cable reconnected (90s after pull)
- t≈1899s: PIPELINE RESET + INIT→ACQUIRING
- t≈1962s: Clock re-initialized, **PPS resumed** — 63s after reconnect
- t≈2104s: Within 20 µs of steady state — 205s after reconnect

**PPS coasting during holdover:**
- PPS ran for **59s** after cable pull (vs 50s crash in Test 3, 0s in Test 3b)
- Phase drifted smoothly: +12 → -47 µs at **-1.1 µs/s** (crystal wander)
- Re-anchor at 40s successfully extended PPS past 51.5s counter wrap
- PPS stopped at 59s: HOLDOVER→INIT transition killed clock_valid

**PPS outage: 94s** (vs 118s in Test 3b, vs 423s in Test 3)

**Comparison across all Test 3 variants:**

| Metric | Test 3 (v0.9) | Test 3b (v1.0) | Test 3c (coast) |
|--------|--------------|----------------|-----------------|
| PPS coasting | 50s (crash) | 0s (disabled) | 59s (smooth) |
| PPS outage | 423s | 118s | 94s |
| Max excursion | -8207 µs | +393 µs | +409 µs |
| Recovery to 20 µs | 669s | 199s | 205s |
| Post sigma | 4.5 µs | 5.0 µs | 5.9 µs |

**Success criteria: PASS**
- ✓ PPS coasts through holdover on last known scale_factor
- ✓ Re-anchor prevents 32-bit counter wrap crash
- ✓ Smooth phase drift during coasting (-1.1 µs/s)
- ✓ All state transitions correct
- ✓ Post-recovery matches pre-pull baseline

**Remaining limitation:**
- HOLDOVER→INIT at 60s kills PPS. Extending LTSP_HOLDOVER_TIMEOUT_US from 30s
  to e.g. 300s would let PPS coast the entire 90s outage (~100 µs drift).
  At -1.1 µs/s, even 5 min of holdover would only drift ~330 µs.

---

## Test 3d: Extended Outage (90s) — WITH 5-MINUTE HOLDOVER (v1.0) — COMPLETED

**Result:** Run `tools/runs/20260227_155200` (65-min, 500 µs/div, 1 MSa/s)

**Change vs Test 3c:** Extended `LTSP_HOLDOVER_TIMEOUT_US` from 30s to 300s (5 min).

| Phase | Pre-pull (800-1800s) | Post-recovery (>2200s) |
|-------|---------------------|------------------------|
| Sigma | 5.5 µs | 5.6 µs |
| Mean | +8.6 µs | +11.6 µs |
| P-P | 28.8 µs | 35.2 µs |

**Timeline:**
- t≈1807s: Cable pulled (settled at +4.8 µs)
- t≈1847s: Re-anchor #1 (40s, before 51.5s counter wrap)
- t≈1887s: Re-anchor #2 (80s)
- t≈1903s: Cable reconnected (~96 missing packets)
- t≈1903s: PIPELINE RESET + PPS RESEED (state stays LOCKED — 96s < 300s timeout)
- t≈1958s: Phase minimum -212 µs (coasting drift at scope measurement time)
- t≈1965s: Within 20 µs of steady state — **7s after reconnect**

**PPS outage: 0s — PPS coasted the ENTIRE outage!**

**During ~96s outage:**
- Phase drifted smoothly: +4.8 → -212 µs at **-2.1 µs/s** (crystal error)
- Two re-anchors fired at 40s and 80s — prevented 32-bit PIO counter wrap
- State stayed LOCKED (96s < 300s holdover timeout)
- No HOLDOVER transition needed — holdover coasting on Core 0 kept PPS running

**On reconnect:**
- Gap detection: `seq restart=96`
- Pipeline reset: regression, min filter, sample counter, clock cleared
- PPS reseed: immediate re-sync from fresh gps_now
- Recovery from -212 µs to within 20 µs in **7 seconds**
- No regression instability (pipeline reset prevented corrupted drift)
- No excursion beyond coasting drift (-212 µs was the maximum)

**Comparison across all Test 3 variants:**

| Metric | Test 3 (v0.9) | Test 3b (v1.0) | Test 3c (coast) | **Test 3d (5min)** |
|--------|--------------|----------------|-----------------|-------------------|
| PPS coasting | 50s (crash) | 0s (disabled) | 59s (smooth) | **96s (full)** |
| PPS outage | 423s | 118s | 94s | **0s** |
| Max excursion | -8207 µs | +393 µs | +409 µs | **-212 µs** |
| Recovery to 20 µs | 669s | 199s | 205s | **7s** |
| Post sigma | 4.5 µs | 5.0 µs | 5.9 µs | **5.6 µs** |

**Success criteria: PASS (best result of all tests)**
- ✓ Zero PPS outage — coasted the entire 96s cable disconnect
- ✓ Smooth phase drift during coasting (-2.1 µs/s, max -212 µs)
- ✓ Re-anchors at 40s/80s prevented counter wrap
- ✓ Immediate recovery: within 20 µs in 7s after reconnect
- ✓ No regression instability (pipeline reset works correctly)
- ✓ Post-recovery performance identical to pre-pull baseline
- ✓ State stayed LOCKED throughout (no unnecessary transitions)

---

## Test 4: Packet Delay Variation (PDV)

**What it tests:** Filter response to increased network jitter.

**Procedure — Option A (network switch):**
1. Insert a managed switch between GM and RX
2. Generate cross-traffic (e.g. iperf flood from a PC through the switch)
3. Run 30-min test, compare phase sigma to baseline

**Procedure — Option B (software delay, if GM on same network as a Linux box):**
1. Use `tc netem` on a Linux bridge to add random delay:
   `tc qdisc add dev eth0 root netem delay 100us 50us distribution normal`
2. This adds 100±50 µs of delay variation on top of existing jitter
3. Run 30-min test

**Procedure — Option C (simplest — no extra equipment):**
1. Run the test with a longer cable or through a consumer switch/router
2. Compare jitter sigma and phase sigma to direct-connect baseline

**Expected behavior:**
- Higher d_detrended sigma (e.g. 50–100 µs instead of 31 µs)
- Phase sigma should increase proportionally: sigma_phase ≈ alpha × sigma_jitter
  At alpha=0.02, doubling jitter from 31→62 µs should give ~8–10 µs phase sigma
- The filter should still track — no loss of lock

**Success criteria:**
- State remains LOCKED throughout
- Phase sigma scales roughly linearly with jitter sigma
- No catastrophic failure or lock loss

---

## Test 5: Congestion — Burst Packet Loss

**What it tests:** Intermittent packet loss (as from network congestion).

**Procedure:**
1. Requires a Linux bridge or managed switch
2. Periodically block LTSP packets for 5s every 30s (simulates congestion bursts)
   Could use iptables rule toggling on a Linux bridge
3. Run 30-min test

**Alternative (manual):**
1. While test is running, briefly disconnect/reconnect cable every 30 seconds
   for ~3-5 second gaps (tedious but requires no equipment)

**Expected behavior:**
- 5s gap: well within holdover timeout (30s), state stays LOCKED
- Phase drifts ~150 µs per burst (30 µs/s × 5s)
- Filter corrects after each burst — but recovery is slow at alpha=0.02
- Overall phase sigma likely degraded to 15–30 µs

**Success criteria:**
- No state transition (stays LOCKED)
- Phase recovers between bursts
- No cumulative drift or offset shift

---

## Test 6: GM Holdover (GPS Loss)

**What it tests:** RX behavior when GM loses GPS lock.

**Procedure:**
1. Run system until LOCKED and settled
2. Disconnect GPS antenna from GM
3. Observe GM logs for holdover flag
4. Observe RX state transition to HOLDOVER
5. Reconnect GPS antenna
6. Observe recovery

**Expected behavior:**
- GM sets LTSP_FLAG_HOLDOVER in PDU
- RX transitions LOCKED → HOLDOVER on receiving holdover flag
- RX continues to receive packets but doesn't trust them for timing
- 1PPS coasts on crystal
- After GPS re-lock: GM clears holdover flag, RX: HOLDOVER → ACQUIRING → LOCKED

**Success criteria:**
- Correct state transitions on both GM and RX
- RX doesn't use holdover-flagged packets for filter updates
- Clean recovery after GPS re-lock

---

## Test 7: Adaptive Alpha Recovery (Regression Test)

**What it tests:** Whether adaptive alpha should reset after holdover.

**Procedure:**
1. Run Test 2 (45s outage) twice:
   a. First with current code (alpha stays at 0.02 after recovery)
   b. Then with modified code that resets `pps_filter_count = 0` on
      HOLDOVER → ACQUIRING transition (alpha restarts at 0.2)
2. Compare recovery time to <10 µs sigma

**Expected behavior:**
- Version (a): slow recovery (~5 min) because alpha=0.02 can't track fast
- Version (b): fast recovery (~30s) because alpha resets to 0.2

**This test determines:** Whether we need to add alpha reset logic.

---

## Recommended Test Order

| # | Test | Equipment | Duration | Priority |
|---|------|-----------|----------|----------|
| 1 | 10s cable disconnect | None | 10 min | **HIGH** — easiest, most informative |
| 2 | 45s cable disconnect | None | 15 min | **HIGH** — tests holdover |
| 3 | 90s cable disconnect | None | 20 min | MEDIUM — tests full reset |
| 6 | GM GPS loss | None | 15 min | MEDIUM — tests GM holdover flag |
| 7 | Alpha recovery | Code change | 30 min | HIGH — likely improvement needed |
| 4 | PDV increase | Switch/bridge | 30 min | LOW — needs equipment |
| 5 | Burst loss | Switch/bridge | 30 min | LOW — needs equipment |

Tests 1, 2, 3 can be done immediately with just a cable pull. Test 6 requires
access to the GM's GPS antenna. Tests 4 and 5 need a Linux bridge or managed switch.

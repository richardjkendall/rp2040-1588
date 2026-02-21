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

**Conclusions:**
- Switch adds negligible jitter — sigma unchanged at ~6 µs
- Mean offset shifted +7.8 µs from asymmetric store-and-forward delay
- 5-min windows actually more consistent with switch (fewer extremes)
- System fully robust to switch insertion (no competing traffic)

---

## Test 1: Packet Loss — Cable Disconnect (10s)

**What it tests:** Short holdover — does the 1PPS coast correctly on crystal?

**Procedure:**
1. Run system until LOCKED and settled (>5 min)
2. Disconnect Ethernet cable from RX for 10 seconds
3. Reconnect
4. Observe: state transitions, 1PPS phase during/after

**Expected behavior:**
- State stays LOCKED (holdover timeout is 30s, outage is only 10s)
- 1PPS coasts on last-known scale_factor
- Phase drifts ~30 µs/s × 10s = ~300 µs during outage (crystal drift)
- After reconnect: filter reconverges over ~50 packets (tau=30 at alpha=0.02)
- Should return to <10 µs sigma within ~2 minutes

**Success criteria:**
- No crash or state reset
- Phase recovers to <10 µs sigma within 3 minutes of reconnect
- No permanent offset shift

---

## Test 2: Packet Loss — Cable Disconnect (45s)

**What it tests:** HOLDOVER state entry and recovery.

**Procedure:**
1. Run system until LOCKED and settled (>5 min)
2. Disconnect Ethernet cable from RX for 45 seconds
3. Reconnect
4. Observe: state transitions, 1PPS phase, recovery time

**Expected behavior:**
- At 30s: LOCKED → HOLDOVER
- Phase drifts ~30 µs/s × 45s = ~1.35 ms during outage
- 1PPS may go off-scope during outage
- On reconnect: HOLDOVER → ACQUIRING → LOCKED
- Filter re-converges — but adaptive alpha has already decayed to 0.02!
  This means recovery will be slow (~150 packets / 2.5 min at alpha=0.02)

**Known issue to watch:** The adaptive alpha doesn't reset on HOLDOVER recovery.
After a long outage, the filter is stuck at alpha=0.02 and must re-converge from
a ~1.35 ms phase error. This could take 5+ minutes. Consider resetting
`pps_filter_count` to 0 on HOLDOVER → ACQUIRING transition.

**Success criteria:**
- Correct state transitions logged
- Phase recovers to <10 µs sigma within 5 minutes of reconnect
- No crash, no permanent offset

---

## Test 3: Packet Loss — Extended Outage (90s)

**What it tests:** Full state reset (HOLDOVER → INIT) and cold restart.

**Procedure:**
1. Run system until LOCKED and settled (>5 min)
2. Disconnect Ethernet cable for 90 seconds
3. Reconnect
4. Observe: full state machine reset and re-initialization

**Expected behavior:**
- At 30s: LOCKED → HOLDOVER
- At 60s: HOLDOVER → INIT (full reset, clock_valid = false)
- 1PPS should stop during INIT (pps_enabled requires ACQUIRING/LOCKED)
- On reconnect: behaves like cold boot — INIT → ACQUIRING → LOCKED
- Adaptive alpha resets naturally (new clock init → pps_filter_count = 0)
- Full convergence: ~300s (same as cold boot)

**Success criteria:**
- 1PPS stops during extended holdover
- Clean re-initialization after reconnect
- Performance matches cold-boot baseline within 10 minutes

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

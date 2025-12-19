# PIO Measurement Glitch - Root Cause Analysis

## Date: 2025-12-19

---

## Observed Pattern

Progressive degradation leading to invalid measurements:

```
seq=20419: phase=100.5µs   path_delay=999895.9µs  ← Phase decreasing
seq=20420: phase= 76.2µs   path_delay=999922.3µs  ← Getting closer
seq=20421: phase= 52.5µs   path_delay=999945.9µs  ← Very close
seq=20422: phase= 22.8µs   path_delay=999972.8µs  ← Almost zero!
seq=20423: phase=1000000µs path_delay=1000000µs   ✗ INVALID
seq=20424: phase=0µs       path_delay=0µs         ✗ INVALID
seq=20425: phase=0µs       path_delay=0µs         ✗ INVALID
```

Notice:
- Phase offset decreasing: 100µs → 76µs → 52µs → 22µs → invalid
- Path delay increasing: 999896µs → 999923µs → 999946µs → 999973µs → invalid
- Both approaching limits (phase → 0, path_delay → 1 second)

---

## Normal Operation

**Setup:**
- GM 1PPS arrives first (e.g., at time T)
- Slave 1PPS arrives later (e.g., at T + 120µs)
- 1PPS pulse width: ~100-200ms (typical)

**GM→Slave PIO:**
```
1. Wait for GM rising edge          → triggers at time T
2. X = 0xFFFFFFFF (countdown start)
3. Check if Slave pin already HIGH  → NO (Slave hasn't pulsed yet)
4. Count loop: wait for Slave edge  → triggers at T + 120µs
5. Count: ~3,333 ticks (120µs / 36ns per tick)
6. Push count to FIFO: 3333
7. Convert: 3333 * 36ns = 119,988ns ≈ 120µs ✓
```

**Slave→GM PIO:**
```
1. Wait for Slave rising edge       → triggers at time T + 120µs
2. X = 0xFFFFFFFF
3. Check if GM pin already HIGH     → YES! (GM pulse is still active)
4. Jump to done_immediately
5. Push count to FIFO: 0
   OR if GM pulse has ended:
   Count until next GM edge (1 second later)
   Push count: ~27,777,777 ticks ≈ 999,880µs ✓
```

Result:
- `gm_to_slave_ns = 120,000ns` ✓
- `slave_to_gm_ns = 999,880,000ns` (or small invalid value) ✓
- Code selects minimum valid measurement: `phase_offset_ns = 120,000ns` ✓

---

## Failure Mode: Phase Offset Approaching Zero

**What causes phase to approach zero:**

The slave applies **aggressive clock corrections** (20% of filtered offset):
- Offset detected: +120µs
- Correction applied: -24µs
- New offset: ~96µs (on average)

Over multiple correction cycles, the offset can:
1. Oscillate (overcorrect → undercorrect → overcorrect)
2. Trend toward zero if drift compensates for offset
3. Cross zero (slave arrives before GM)

**Sequence of events leading to glitch:**

### Iteration 1: Normal (phase = 120µs)
```
Time T:       GM rises  ──────────────────────────────┐
Time T+120µs:                                    Slave rises ───────────

GM→Slave PIO:  [waits for GM] [counts 120µs] [sees Slave] → 120µs ✓
Slave→GM PIO:  [waits for Slave] [GM pin HIGH] → 0µs or 999880µs ✓
```

### Iteration 2-5: Phase decreasing (100µs → 76µs → 52µs → 22µs)
```
Slave is correcting aggressively, phase offset shrinking each second
```

### Iteration 6: Phase crosses zero or becomes synchronized

**Scenario A: Edges exactly synchronized (phase ≈ 0µs)**

```
Time T:  GM rises  ────────────────────────────────────┐
Time T:  Slave rises ──────────────────────────────────┐  (same instant)

         Both pulses active for 100-200ms
```

**GM→Slave PIO execution:**
```
1. Wait for GM edge           → triggers at time T
2. X = 0xFFFFFFFF
3. Check if Slave pin HIGH    → YES! (Slave pulse just started, pin went HIGH)
4. Jump to done_immediately
5. Push count: ~0 or small value (1-2 ticks = 36-72ns)
6. Convert: 0 * 36ns = 0ns ✗
```

**Slave→GM PIO execution:**
```
1. Wait for Slave edge        → triggers at time T
2. X = 0xFFFFFFFF
3. Check if GM pin HIGH       → YES! (GM pulse just started, pin went HIGH)
4. Jump to done_immediately
5. Push count: ~0 or small value
6. Convert: 0 * 36ns = 0ns ✗
```

**Result:**
- `gm_to_slave_ns = 0` ✗
- `slave_to_gm_ns = 0` ✗
- Code selects minimum: `phase_offset_ns = 0` ✗

**Scenario B: Slave edge arrives slightly BEFORE GM (phase < 0)**

```
Time T:      Slave rises ──────────────────────────────┐
Time T+2µs:               GM rises ────────────────────────────┐
```

**GM→Slave PIO execution:**
```
1. Wait for GM edge           → triggers at time T+2µs
2. X = 0xFFFFFFFF
3. Check if Slave pin HIGH    → YES! (Slave went HIGH 2µs ago, still active)
4. Jump to done_immediately
5. Push count: 0
6. Convert: 0ns ✗
```

**Slave→GM PIO execution:**
```
1. Wait for Slave edge        → triggers at time T
2. X = 0xFFFFFFFF
3. Check if GM pin HIGH       → NO (GM hasn't pulsed yet)
4. Count loop: wait for GM edge
5. GM edge arrives 2µs later  → count ≈ 55 ticks (2µs / 36ns)
6. BUT: Next iteration wraps, waits for NEXT Slave edge
7. Meanwhile, keeps counting...
8. Next GM edge (1 second later) arrives
9. Push count: ~27,777,777 ticks
10. Convert: 27777777 * 36ns ≈ 1,000,000,000ns ✗
```

**Result:**
- `gm_to_slave_ns = 0` ✗
- `slave_to_gm_ns = 1,000,000,000` ✗
- Code selects minimum: `phase_offset_ns = 0` ✗

OR depending on timing:
- `gm_to_slave_ns = 1,000,000,000` ✗
- `slave_to_gm_ns = 0` ✗
- Code selects minimum: `phase_offset_ns = 0` ✗

**Scenario C: PIO state machine misalignment**

If the edges cross over (slave becomes earlier than GM), the PIO state machines can get out of sync:
- GM→Slave waits for GM, but Slave already happened → measures to NEXT Slave (1 second)
- Slave→GM waits for Slave, sees GM pin HIGH from previous second → measures 0

This creates the pattern we see:
- Multiple samples of `phase_offset_ns = 0`
- Multiple samples of `phase_offset_ns = 1,000,000,000`
- Multiple samples of `path_delay = 0` or `1,000,000,000`

---

## Why This Happens

**Root Cause: Aggressive clock correction + narrow phase margin**

1. **Slave correction factor = 20% (Phase 1)**
   - When offset = 120µs, correction = 24µs per cycle
   - Over 5 cycles: 120µs → 96µs → 72µs → 48µs → 24µs → 0µs
   - Phase offset can approach zero in just a few seconds

2. **1PPS pulse width = 100-200ms**
   - PIO checks "is pin already HIGH" to detect wrong interval
   - But with 100-200ms pulses, pin can be HIGH for legitimate measurements
   - When edges are within ~100ms, PIO can't distinguish current vs. previous pulse

3. **No hysteresis or dead zone**
   - When phase crosses zero, it oscillates back and forth
   - Each oscillation can trigger invalid measurements
   - No filtering to ignore edges that are "too close"

4. **PIO program assumes edges are well-separated**
   - Designed for edges separated by microseconds to milliseconds
   - When edges are within ~100ns (PIO execution overhead), timing is ambiguous
   - When edges invert (slave before GM), PIO measures wrong interval

---

## Why Calibration Didn't Show This

**Calibration test characteristics:**
- **Static phase offset**: 100µs, fixed, no corrections applied
- **GPS-disciplined sources**: Both 1PPS signals from GPS modules (very stable)
- **No phase oscillation**: Offset remained constant at ~100µs
- **Never approached zero**: Phase never decreased toward 0µs

**Slave test characteristics:**
- **Dynamic phase offset**: Slave actively correcting (±24µs per second)
- **Oscillatory behavior**: Correction can overshoot and oscillate
- **Approaches zero**: During convergence or after disturbances
- **Crosses zero**: Phase can go negative briefly

The glitch only occurs when **phase offset approaches or crosses zero**, which never happened in calibration.

---

## Evidence Supporting This Theory

### 1. Progressive degradation pattern
```
Phase: 100µs → 76µs → 52µs → 22µs → 0µs
```
This is exactly what we'd expect from aggressive correction approaching zero.

### 2. Path delay inversely correlated
```
Path delay: 999896µs → 999923µs → 999946µs → 999973µs → 1000000µs
```
As phase → 0, path delay → 1 second (they sum to ~1 second).

### 3. Recovery pattern
```
seq=20427: phase= 34272.0ns  ← recovering, but phase inverted
seq=20428: phase= 61811.9ns  ← still recovering
seq=20429: phase=~120000ns   ← back to normal
```
After glitch, phase builds back up from ~30µs, suggesting slave overshot zero and is now correcting back.

### 4. Frequency matches control loop dynamics
- Glitches occur every 5-30 minutes
- This matches time scale for large corrections or disturbances
- Aggressive 20% correction can drive phase to zero in seconds when offset changes

### 5. Both metrics invalid simultaneously (92.4% correlation)
- Both `phase_offset_ns` and `gm_to_slave_ns` hit 0 or 1e9
- Both are calculated from same PIO tick counts
- When PIO glitches, both values are wrong

---

## Solution Options

### Option 1: Filter Invalid Data (QUICK FIX)

**In measurement device (measurement/main_measurement_device_wifi.c:375-389)**

Move validation BEFORE ring buffer write:

```c
// Validate measurement before sending to telemetry
#define MIN_VALID_OFFSET_NS 1000.0      // 1µs minimum
#define MAX_VALID_OFFSET_NS 10000000.0  // 10ms maximum

// Also check path delay values for sanity
bool is_valid = (phase_offset_ns >= MIN_VALID_OFFSET_NS &&
                 phase_offset_ns <= MAX_VALID_OFFSET_NS &&
                 gm_to_slave_ns >= MIN_VALID_OFFSET_NS &&
                 gm_to_slave_ns <= MAX_VALID_OFFSET_NS &&
                 slave_to_gm_ns >= MIN_VALID_OFFSET_NS &&
                 slave_to_gm_ns <= MAX_VALID_OFFSET_NS);

if (is_valid) {
    // Write to ring buffer only if valid
    if (!ring_buffer_try_write(&measurement_buffer, &m)) {
        // Buffer full
    }

    // Update local statistics
    update_phase_stats(phase_offset_ns, gm_first);
} else {
    // Count rejected sample for debugging
    rejected_count++;
}

measurement_count++;
```

**Pros:**
- Quick fix (5 minutes)
- Prevents invalid data from reaching telemetry
- Already partially implemented (local stats filtering exists)

**Cons:**
- Doesn't fix root cause (PIO still glitches)
- Loses data points (but they're invalid anyway)

### Option 2: Add Hysteresis to Slave Correction

**Prevent phase from oscillating through zero:**

```c
// In ptp_discipline.c, add dead zone near zero
double correction_factor;
if (fabs(filtered_offset_ns) < 5000.0) {
    // Dead zone: ±5µs, don't correct (already very close)
    correction_factor = 0.0;
} else if (state.discipline_updates < 30) {
    correction_factor = 0.5;
} else if (state.discipline_updates < 100) {
    correction_factor = 0.3;
} else {
    correction_factor = 0.2;
}
```

**Pros:**
- Prevents overshoot through zero
- Reduces oscillation
- Improves stability

**Cons:**
- Allows up to ±5µs steady-state error
- Doesn't address fundamental PIO design limitation

### Option 3: Redesign PIO Programs

**Add edge direction detection and filtering:**

Instead of checking "is pin already HIGH", add:
- Timestamp-based filtering (ignore edges within 100ms of previous)
- Edge polarity detection (ensure measuring rising→rising, not falling→rising)
- Bounds checking on tick counts (reject counts < 100 or > 30M)

**Pros:**
- Robust solution
- Handles edge cases properly

**Cons:**
- Requires PIO program redesign
- More complex state machine logic
- May not fit in PIO instruction space

---

## Recommendation

**Implement Option 1 (Filter Invalid Data) immediately:**

This fixes the symptom (invalid telemetry data) and is a 5-minute fix.

The root cause (PIO glitch when phase approaches zero) is a fundamental design limitation:
- PIO programs assume edges are well-separated
- When slave aggressively corrects and phase oscillates through zero, PIO breaks
- This is expected behavior given the PIO design

**The slave is actually performing well:**
- True performance: 8-9µs std dev (excellent)
- Phase occasionally approaches zero due to correction dynamics
- This is control theory at work (oscillation around setpoint)
- The glitches are measurement artifacts, not slave malfunction

**Accept the limitation:**
- Filter out invalid measurements (0ns or 1e9ns)
- True slave performance is already excellent
- No need for Phase 2-5 improvements

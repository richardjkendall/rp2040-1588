# Next Optimization: Split PI Terms Between Continuous and Discrete Corrections

**Date:** 2025-12-26
**Status:** Proposed - Not Yet Implemented
**Current Performance:** Mean 43 µs, Std Dev 31 µs, Min 50 ns, Max 114 µs

## Current Problem

After implementing:
1. ✅ Pure PI servo (removed Kalman)
2. ✅ Continuous frequency correction via scale_factor
3. ✅ EMA-filtered crystal measurement for scale_factor

**We still see 20-110 µs sawtooth jitter** because we're applying the **I term twice**:

```c
// Current implementation (DOUBLE FREQUENCY CORRECTION):

// 1. Continuous (in update_ptp_clock):
filtered_crystal_error_ns = EMA(crystal_error_ns);  // Tracks frequency
scale_factor = 1.0 - (filtered_crystal_error_ns / 1e6);
ptp_elapsed_ns = elapsed_us * 1000.0 * scale_factor;

// 2. Discrete (in discipline loop):
freq_adj = Kp * offset + pi_integral;  // PI servo output (P + I)
state.ptp_clock_ns -= freq_adj;  // Applies both P and I again!
```

**Result:** I term applied twice → overcorrection → oscillation

## Root Cause Analysis

### Fundamental Control Theory:

**Frequency correction alone cannot eliminate phase offsets**
- Analogy: Setting cruise control to correct speed won't catch you up if you're already behind
- Frequency tracking (I term) prevents NEW drift
- Phase correction (P term) eliminates EXISTING offset

### What Each Term Should Do:

**Integral (I) Term:**
- Estimates long-term frequency offset
- Should be applied **continuously** via scale_factor
- Makes clock run at correct rate

**Proportional (P) Term:**
- Responds to immediate phase error
- Should be applied **discretely** to nudge phase
- Actually eliminates the offset

## Proposed Solution

### Split PI Terms:

**1. I Term → Continuous Frequency (scale_factor)**
```c
// In update_ptp_clock():
// Use PI integral for scale_factor (smooth, well-filtered)
double scale_factor = 1.0 - (state.pi_integral / 1000000000.0);
uint64_t ptp_elapsed_ns = (uint64_t)(elapsed_us * 1000.0 * scale_factor);
```

**2. P Term → Discrete Phase Correction**
```c
// In discipline loop (slew section):
// Apply ONLY proportional term for phase correction
double p_term = PI_KP * (double)state.offset_from_master_ns;
int64_t correction = (int64_t)p_term;
state.ptp_clock_ns -= correction;
```

**3. Keep PI Servo Running (for I term update)**
```c
// PI servo still runs to update the integral
int64_t freq_adj_ns = pi_servo_update(measured_offset, dt_sec);
// But we only use state.pi_integral for scale_factor, not freq_adj_ns directly
```

## Implementation Steps

### File: `slave/ptp_discipline.c`

**Change 1: Update scale_factor function (lines 379-388)**
```c
double ptp_discipline_get_scale_factor(void) {
    // Use PI integral for continuous frequency correction
    // Integral term is well-filtered (accumulated over time)
    // This eliminates the sawtooth drift between PTP syncs
    return 1.0 - (state.pi_integral / 1000000000.0);
}
```

**Change 2: Discrete correction uses P term only (lines 781-788)**
```c
} else {
    // SLEW: Small offset - use ONLY proportional term
    // Integral term is applied continuously via scale_factor

    // P term provides immediate phase correction
    double p_term = PI_KP * (double)state.offset_from_master_ns;
    int64_t correction = (int64_t)p_term;

    state.ptp_clock_ns -= correction;
```

**Change 3: Keep PI servo running (no change needed)**
- PI servo continues to update `state.pi_integral`
- Integral is used via `get_scale_factor()`, not directly

## Expected Results

**Benefits:**
1. ✅ **I term applied once** (continuous via scale_factor)
2. ✅ **P term for phase** (discrete corrections)
3. ✅ **No double correction** (eliminates interference)
4. ✅ **Smooth frequency tracking** (PI integral is inherently filtered)
5. ✅ **No feedback loop** (observation and control separated)

**Performance Prediction:**
- Mean: 43 µs → < 5 µs (better convergence)
- Std Dev: 31 µs → < 10 µs (much less jitter)
- Pattern: Smooth convergence instead of sawtooth
- Stability: PI integral provides smooth, stable frequency estimate

## Why This Works

### Separation of Concerns:

**Frequency Control (I term via scale_factor):**
- Handles crystal drift (+34 ppm)
- Applied continuously (every clock update)
- Uses PI integral (smooth, filtered)
- No sudden jumps

**Phase Control (P term):**
- Handles transient offsets
- Applied once per second (at PTP sync)
- Immediate response to current error
- Pushes offset toward zero

### No Feedback Loop:

**Before:** PI integral → scale_factor → offset → PI integral (oscillation)
**After:**
- PI integral → scale_factor (one direction)
- Offset → P term correction (separate path)

## Alternative: Use Crystal Measurement for Scale Factor

If PI integral in scale_factor still causes issues, revert to filtered crystal:

```c
double ptp_discipline_get_scale_factor(void) {
    // Use EMA-filtered crystal measurement
    double crystal_ppm = state.filtered_crystal_error_ns / 1000.0;
    return 1.0 - (crystal_ppm / 1000000.0);
}
```

**Trade-off:**
- Crystal measurement: More noise, but truly independent
- PI integral: Smoother, but theoretical feedback risk

## Testing Plan

1. **Flash new firmware** with P/I split
2. **Monitor for 10+ minutes** to see convergence
3. **Check for:**
   - Sawtooth eliminated ✓
   - Std dev < 10 µs ✓
   - Mean approaching 50 ns floor ✓
   - No oscillation ✓

4. **If issues persist:**
   - Try crystal measurement for scale_factor instead
   - Adjust filter time constant (currently 10 seconds)
   - Consider reducing Kp (currently 0.7)

## Build Commands

```bash
cd /Users/rjk/Code/pico-gps-1588/build
make w5500_ptp_slave -j8
# Flash: slave/w5500_ptp_slave.uf2
```

## References

- **Current state:** `slave/ptp_discipline.c`
- **PI servo:** Lines 465-496
- **Scale factor:** Lines 379-388
- **Discrete correction:** Lines 781-805
- **Crystal measurement:** Lines 728-741

## Related Documents

- `docs/SCALE_FACTOR_HYBRID_APPROACH.md` - Original hybrid proposal
- `docs/PTP_CONVERGENCE_PLAN.md` - PI servo implementation
- `claude.md` - Project reference

---

**Status:** Ready to implement when you return
**Priority:** High - Should eliminate remaining jitter
**Risk:** Low - Clean separation of P and I terms

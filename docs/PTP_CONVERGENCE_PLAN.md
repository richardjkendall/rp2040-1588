# PTP Slave Convergence Improvement Plan
**Date:** 2024-12-24
**Status:** Active Development - Code Review Complete

## Executive Summary

**✓ Achievement:** Asymmetry correction (+181µs) successfully aligned PTP calculated offset with 1PPS measurements. The system now accurately measures its offset.

**✗ Critical Bug Found:** Kalman filter and manual clock corrections are fighting each other, causing:
- Boot divergence: 390µs → 1048µs before eventual convergence
- Wrong frequency estimate: +63,000 ppb (should be ~0 ppb)
- Very slow convergence: 48 minutes to reach 152µs (should be < 5 min)

**Root Cause:** Kalman filter tries to estimate crystal frequency by observing offset changes. But manual corrections create artificial offset changes that Kalman misinterprets as frequency drift, learning the wrong frequency and making scale_factor drive the clock in the wrong direction.

**Fix:** Decouple Kalman estimation from manual corrections (Phase 1 tasks).

---

## Current State Analysis

### Measurement Data (offset_for_review.csv)
- **Duration:** 48 minutes (2,880 samples @ 1 Hz)
- **Mean Offset:** 152.5 µs (slave ahead of GM)
- **Standard Deviation:** 31.3 µs
- **Range:** 64.1 to 338.8 µs (274.7 µs total)
- **Direction:** 100% slave-first (slave consistently ahead)

### Success: PTP Offset now aligned with 1PPS Measurement ✓
The asymmetry correction (+181µs) successfully aligned PTP calculated offset with independently measured 1PPS phase offset. This validates that our offset calculation is now **accurate**.

### Observed Error Components

#### 1. **Bias (DC Offset): ~152 µs**
- **Symptom:** Slave 1PPS arrives 152µs before GM 1PPS on average
- **Root Cause:** Slave PTP clock is running 152µs ahead of GM clock
- **Goal:** Drive this to < 20µs (sub-sample accuracy)
- **Status:** Slowly decreasing (servo is working, but very slow)
  - 0-10 min: ~166-172µs
  - 10-20 min: ~143-150µs
  - 20-45 min: ~140-150µs
  - **Convergence rate:** ~25µs reduction over 48 minutes = ~0.5µs/min

#### 2. **Oscillation (AC Component): ~40-50s period**
- **Symptom:** Offset oscillates between ~120µs and ~220µs
- **Amplitude:** ±30-50µs around mean
- **Period:** ~40-50 seconds
- **Root Causes (likely multiple):**
  - Path delay measurement noise/jumps
  - Kalman filter dynamics (overcorrection)
  - Correction factor too aggressive (currently 20-50%)
  - RX averaging fallback discontinuities (~13% of measurements)

#### 3. **High-Frequency Noise: ~5-10µs (estimated)**
- **Symptom:** Sample-to-sample jitter
- **Root Causes:**
  - HW timestamp correlation jitter
  - Network packet timing variance
  - PIO counter resolution (12ns, negligible)
  - Crystal phase noise

## Scientific Decomposition

### Error Budget Analysis

| Component | Magnitude | Type | Target | Priority |
|-----------|-----------|------|--------|----------|
| **Bias (mean offset)** | 152 µs | Systematic | < 20 µs | **HIGH** |
| **Oscillation** | ±30-50 µs | Periodic | < 10 µs | **HIGH** |
| **HF Noise** | ~5-10 µs | Random | < 5 µs | **MEDIUM** |
| **Convergence time** | Very slow | Dynamic | < 5 min | **HIGH** |

### Root Cause Hierarchy

```
152µs Offset (Goal: <20µs)
├─ Initial Boot Offset (~320µs) → Converging slowly
│  ├─ No step threshold after first 10 updates
│  └─ Correction factor insufficient for large errors
│
├─ Oscillation (40-50s period, ±30-50µs)
│  ├─ Path delay jumps (60-80µs observed in logs)
│  │  ├─ Forward/reverse path asymmetry variability
│  │  └─ RX averaging fallback discontinuities (13%)
│  ├─ Kalman filter dynamics
│  │  ├─ Q/R parameters may not match actual noise
│  │  └─ Prediction-update cycle at 1Hz may be too slow
│  └─ Aggressive correction (20-50% per cycle)
│      └─ Drives frequency offset, causes overshoot
│
└─ High-Frequency Noise (~5-10µs)
   ├─ HW timestamp correlation jitter
   ├─ Network packet timing variance
   └─ Path delay filter (α=0.1, 10s time constant)
```

## Improvement Plan (Prioritized)

### **CRITICAL BUG FOUND:** Kalman-Correction Conflict

**Root Cause:** The Kalman filter tries to estimate frequency offset by observing how offset changes over time. However, we also apply manual clock corrections each cycle (line 924: `ptp_clock_ns -= correction`). The Kalman filter cannot distinguish between:
- Offset changes due to crystal frequency drift (what it should track)
- Offset changes due to manual corrections (artificial, confounds the estimate)

**Result:** Kalman learns the WRONG frequency offset (e.g., +63,000 ppb instead of ~0 ppb), causing scale_factor to make the clock run in the wrong direction, fighting against corrections.

**Evidence:**
- Boot logs: offset increased 390µs → 708µs → 1048µs (diverging!)
- After 48 min: offset reduced to ~152µs (eventually converging, but very slowly)
- Servo corrections are fighting against incorrect scale_factor from Kalman

### Phase 1: Fix Kalman-Correction Conflict (CRITICAL)
**Objective:** Decouple Kalman state estimation from manual corrections

#### Task 1.1: Reset Kalman After Steps
**Current Issue:** After stepping the clock (line 874), Kalman state remains unchanged (line 881 comment says "Don't reset"), but the offset just jumped. Kalman interprets this as a large frequency change.

**Solution:**
```c
if (allow_step && abs_offset > step_threshold) {
    // STEP: Jump clock
    state.ptp_clock_ns -= (int64_t)filtered_offset_ns;

    // CRITICAL: Reset Kalman offset estimate after step
    state.kalman_x[0] = 0.0;  // Offset is now ~0 after step
    // Keep frequency estimate (don't reset kalman_x[1])

    printf("STEP: Reset Kalman offset to 0 after step\n");
}
```

**Expected Result:**
- Kalman frequency estimate remains stable after steps
- No divergence after stepping

**Files:** `slave/ptp_discipline.c:871-895`

#### Task 1.2: Account for Manual Corrections in Kalman
**Current Issue:** Kalman predict step assumes `offset(k+1) = offset(k) + freq*dt`, but we also apply manual correction, so actually `offset(k+1) = offset(k) + freq*dt - manual_correction`. Kalman doesn't know about `manual_correction`.

**Solution Option A - Feed corrections as control input to Kalman:**
```c
// After applying manual correction (line 924):
state.ptp_clock_ns -= correction;

// Update Kalman state to reflect this correction
// (Treat it as a known control input, not observation noise)
state.kalman_x[0] -= (double)correction;
```

**Solution Option B - Don't use scale_factor from Kalman during convergence:**
```c
// In update_ptp_clock() and get_ptp_time_ns():
double scale = state.scale_factor;
if (llabs(state.offset_from_master_ns) > 50000) {
    // During convergence: ignore Kalman frequency, use nominal rate
    scale = 1.0;
} else {
    // After lock: trust Kalman frequency estimate
    scale = state.scale_factor;
}
```

**Recommendation:** Use Option B for simplicity. Option A is theoretically better but more complex.

**Expected Result:**
- No frequency fighting during convergence
- Scale_factor only used after achieving lock (<50µs)

**Files:** `slave/ptp_discipline.c:195-217, 418-439`

#### Task 1.3: Adaptive Step Threshold
**Current Issue:** Step threshold (100µs) only active for first 10 updates. Boot logs showed offset at 390µs didn't get stepped because it exceeded threshold.

**Solution:**
```c
#define STEP_THRESHOLD_NS 500000         // 500µs (was 100µs)
#define STEP_UPDATES_MAX 50              // 50 updates (was 10)

bool allow_step = (state.discipline_updates < STEP_UPDATES_MAX);

if (allow_step && llabs((int64_t)filtered_offset_ns) > STEP_THRESHOLD_NS) {
    // STEP: Jump clock
    state.ptp_clock_ns -= (int64_t)filtered_offset_ns;
    state.kalman_x[0] = 0.0;  // Reset Kalman offset after step
}
```

**Expected Result:**
- Boot offset (~390µs) gets stepped down in first few updates
- Convergence time: < 1 minute to reach <50µs

**Files:** `slave/ptp_discipline.c:46-48, 868-895`

---

### Phase 1 Summary

Implementing Tasks 1.1-1.3 will fix the fundamental Kalman-correction conflict:
1. Reset Kalman offset after steps (prevent divergence)
2. Disable scale_factor during convergence (prevent frequency fighting)
3. Increase step threshold to 500µs for 50 updates (handle boot offset)

**Expected Result:** Convergence from 390µs → <50µs in < 2 minutes (currently >48 min)

---

### Phase 2: Reduce Oscillations
**Objective:** Reduce ±30-50µs oscillation to < ±10µs

#### Task 2.0: Adaptive Correction Factor (REVISED)
**Current Code Review Finding:** Correction factors are ALREADY aggressive (50% → 30% → 20%), which is GOOD. The slow convergence was due to Kalman conflict, not gentle corrections.

**New Understanding:** Keep current correction factors, but add logic to prevent overshoot near lock:

**Solution:**
```c
// Correction factor based on offset magnitude and lock state
double correction_factor;
int64_t abs_offset = llabs((int64_t)filtered_offset_ns);

if (!hw_timestamps_used) {
    correction_factor = 0.02;  // SW timestamps: minimal
} else if (abs_offset > 100000) {
    // Large offset: aggressive correction
    correction_factor = 0.4;
} else if (abs_offset > 50000) {
    // Medium offset: moderate correction
    correction_factor = 0.2;
} else if (abs_offset > 20000) {
    // Small offset: gentle correction
    correction_factor = 0.1;
} else {
    // Very small offset: very gentle (avoid oscillation)
    correction_factor = 0.05;
}
```

**Expected Result:**
- Faster convergence for large offsets
- Reduced oscillation near lock
- Smooth transition to tight lock

**Files:** `slave/ptp_discipline.c:893-908`

---

### Phase 2: Reduce Oscillations
**Objective:** Reduce ±30-50µs oscillation to < ±10µs

#### Task 2.1: Path Delay Filter Improvement
**Current Issue:** Path delay uses simple EMA (α=0.1), doesn't reject outliers

**Solution:**
```c
// Median filter (3-5 samples) + EMA for path delay
#define PATH_DELAY_HISTORY 5
static int64_t path_delay_history[PATH_DELAY_HISTORY] = {0};
static uint8_t path_delay_idx = 0;

// Add new measurement to circular buffer
path_delay_history[path_delay_idx] = path_delay_raw;
path_delay_idx = (path_delay_idx + 1) % PATH_DELAY_HISTORY;

// Calculate median
int64_t sorted[PATH_DELAY_HISTORY];
memcpy(sorted, path_delay_history, sizeof(sorted));
qsort(sorted, PATH_DELAY_HISTORY, sizeof(int64_t), compare_int64);
int64_t path_delay_median = sorted[PATH_DELAY_HISTORY/2];

// Apply EMA to median (not raw)
if (state.mean_path_delay_ns == 0) {
    state.mean_path_delay_ns = path_delay_median;
} else {
    state.mean_path_delay_ns = (int64_t)((double)state.mean_path_delay_ns * 0.9 +
                                          (double)path_delay_median * 0.1);
}
```

**Expected Result:**
- Reject path delay outliers/jumps
- Smoother offset calculation
- Reduced oscillation amplitude

**Files:** `slave/ptp_discipline.c:738-746`

#### Task 2.2: Kalman Filter Tuning
**Current Issue:** Q/R parameters may not match actual noise characteristics

**Solution:**
```c
// Re-tune based on observed statistics
#define KALMAN_Q_OFFSET 5e5          // Reduce process noise (was 1e6)
#define KALMAN_Q_FREQ 1e-5           // Reduce frequency random walk (was 1e-4)
#define KALMAN_R_MEASUREMENT_HW 5e8  // Increase measurement noise (was 1e8)
                                     // Accounts for oscillation (~30µs std dev)
```

**Rationale:**
- Observed offset is more stable than assumed (not 1µs process noise)
- Measurement includes ~30µs oscillation (not just 10µs HW jitter)
- Frequency drift is very slow (crystal is stable)

**Expected Result:**
- Smoother state estimates
- Reduced sensitivity to noisy measurements
- Less overshoot in corrections

**Files:** `slave/ptp_discipline.c:51-54`

#### Task 2.3: Reduce RX Averaging Fallback Rate
**Current Issue:** 13% of RX timestamps use averaging fallback, may cause discontinuities

**Solution:**
- Investigate why HW timestamp correlation fails 13% of time
- Improve correlation algorithm tolerance
- Better INT pin timing characterization

**Expected Result:**
- < 5% fallback rate
- More consistent timestamp corrections
- Reduced discontinuities

**Files:** `slave/ptp_discipline.c:240-292`, `slave/ptp_slave_w5500.c:118-163`

---

### Phase 3: Optimize for Tight Lock
**Objective:** Maintain offset < 20µs with < 5µs jitter

#### Task 3.1: Two-Stage Servo
**Current Approach:** Single Kalman + correction factor

**Improved Approach:**
```c
// Stage 1: Acquisition (|offset| > 50µs)
//   - Kalman filter for state estimation
//   - Aggressive correction (0.2-0.4)
//   - Step threshold active
//
// Stage 2: Lock (|offset| < 50µs)
//   - Pure PI servo (proportional-integral)
//   - Gentle correction (Kp=0.01, Ki=0.0001)
//   - No stepping, smooth slewing only

if (llabs(state.offset_from_master_ns) > 50000) {
    // Acquisition mode
    correction = (int64_t)(filtered_offset_ns * 0.2);
} else {
    // Lock mode: PI servo
    state.integral_term += filtered_offset_ns;
    correction = (int64_t)(SERVO_KP * filtered_offset_ns +
                           SERVO_KI * state.integral_term);
}
```

**Expected Result:**
- Fast acquisition
- Smooth lock
- Minimal jitter in steady state

**Files:** `slave/ptp_discipline.c:890-931`

#### Task 3.2: Frequency Holdover
**Concept:** Once locked, maintain frequency correction even during packet loss

**Implementation:**
- Save Kalman frequency estimate
- During packet loss/outliers, apply last known frequency correction
- Prevents large offset buildup during transient failures

**Expected Result:**
- Robust to temporary network issues
- Maintains lock during packet loss
- Faster re-acquisition

---

## Implementation Sequence

### Week 1: Convergence (Phase 1)
1. Implement adaptive step threshold (Task 1.1) ✓ Test with reboots
2. Implement adaptive correction factor (Task 1.2) ✓ Verify faster convergence
3. **Goal:** Convergence to <50µs in < 5 minutes

### Week 2: Oscillation Reduction (Phase 2)
4. Implement median+EMA path delay filter (Task 2.1) ✓ Measure oscillation amplitude
5. Re-tune Kalman Q/R parameters (Task 2.2) ✓ Validate with long runs
6. Investigate RX averaging fallback (Task 2.3) ✓ Reduce fallback rate to <5%
7. **Goal:** Oscillation amplitude < ±10µs

### Week 3: Tight Lock Optimization (Phase 3)
8. Implement two-stage servo (Task 3.1) ✓ Test lock stability
9. Add frequency holdover (Task 3.2) ✓ Test with packet loss
10. **Goal:** Steady-state offset < 20µs with < 5µs jitter

---

## Success Metrics

### Current Performance
- Mean offset: 152.5µs
- Std dev: 31.3µs (includes oscillation)
- Convergence: >48 minutes to 150µs
- Oscillation: ±30-50µs, 40-50s period

### Target Performance
- **Mean offset:** < 20µs (7.6x improvement)
- **Std dev:** < 5µs (6.3x improvement)
- **Convergence:** < 5 minutes to <50µs (>10x faster)
- **Oscillation:** < ±10µs (3-5x reduction)

### Measurement Plan
For each change:
1. Run 1-hour test with crossover cable
2. Capture both PTP logs and 1PPS measurements
3. Calculate:
   - Mean offset (bias)
   - Standard deviation (total noise)
   - Oscillation amplitude (band-pass filter 20-100s period)
   - High-frequency noise (band-pass filter 0-5s period)
   - Convergence time to <50µs threshold
   - Time in tight lock (<20µs)

---

## Related Files

### Analysis
- `tests/asymmetry_analysis_2024-12-24.md` - Asymmetry correction validation
- `tests/offset_for_review.csv` - Current performance data (48 min)
- `docs/PTP_SERVO_APPROACHES.md` - Algorithm comparison

### Code
- `slave/ptp_discipline.c` - Main discipline algorithm
- `slave/ptp_slave_w5500.c` - Protocol handler
- `slave/main_w5500_slave.c` - Timestamp capture

### Build
- Target: `w5500_ptp_slave`
- Location: `build/slave/w5500_ptp_slave.uf2`

---

---

## Code Review Findings vs Original Plan

### What Changed After Code Review

**Original Hypothesis:** Convergence is slow because correction factors are too gentle (20-50%) and step threshold is too restrictive.

**Code Review Reality:**
1. ✓ **Correction factors are actually GOOD** (50% → 30% → 20%) - more aggressive than expected
2. ✓ **Step threshold IS too restrictive** - only 10 updates, 100µs threshold
3. ✗ **NEW CRITICAL BUG:** Kalman-correction conflict causes wrong frequency estimate
4. ✓ **Oscillations are real** - path delay jumps, RX fallback discontinuities confirmed

**Key Insight:** The slow convergence wasn't due to gentle corrections, but due to corrections FIGHTING against wrong Kalman frequency estimate. The servo is like driving with the parking brake on.

### Revised Phase 1 Priority

**Old Plan:** Increase step threshold, make corrections more aggressive
**New Plan:** Fix Kalman-correction conflict FIRST (critical bug), THEN tune step threshold

**Rationale:** No amount of tuning will help if Kalman is driving clock in wrong direction.

---

**Next Action:** Implement Phase 1 (Tasks 1.1-1.3) to fix Kalman-correction conflict and enable proper convergence.

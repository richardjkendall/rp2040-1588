# Phase 2: Reduce Clock Correction Aggressiveness - IN PROGRESS

## Branch: `slave-improvements`
## Date: 2025-12-19
## Build: `slave/w5500_ptp_slave.uf2` (pending)

---

## Changes Made

### File: `slave/ptp_discipline.c` (Lines 813-822)

**BEFORE (Phase 1 baseline)**:
```c
double correction_factor;
if (state.discipline_updates < 30) {
    // Initial convergence: 50% correction for faster settling
    correction_factor = 0.5;
} else if (state.discipline_updates < 100) {
    // Medium convergence: 30% correction
    correction_factor = 0.3;
} else {
    // Steady state: 20% correction (gentle, stable)
    correction_factor = 0.2;
}
```

**AFTER (Phase 2)**:
```c
double correction_factor;
if (state.discipline_updates < 30) {
    // PHASE 2: Initial convergence: 15% correction (was 50%, reduced to prevent overshoot)
    correction_factor = 0.15;
} else if (state.discipline_updates < 100) {
    // PHASE 2: Medium convergence: 8% correction (was 30%, gentler)
    correction_factor = 0.08;
} else {
    // PHASE 2: Steady state: 5% correction (was 20%, much gentler to eliminate oscillation)
    correction_factor = 0.05;
}
```

### Changes Summary:
| Stage | Old Value | New Value | Change Factor | Rationale |
|-------|-----------|-----------|---------------|-----------|
| **Initial (<30 updates)** | 0.5 (50%) | 0.15 (15%) | **3.3× gentler** | Prevent overshoot during convergence |
| **Medium (30-100 updates)** | 0.3 (30%) | 0.08 (8%) | **3.75× gentler** | Smoother transition to steady state |
| **Steady State (>100 updates)** | 0.2 (20%) | 0.05 (5%) | **4× gentler** | Let Kalman do its job, minimal interference |

---

## Problem Diagnosed

### Root Cause:
**Aggressive clock corrections** (20-50%) were causing **overshoot and oscillation**:
- System corrects too much → overshoots target
- Overshoots in opposite direction → oscillates
- Classic control system instability ("fighting itself")
- Kalman provides optimal state estimate, but 20% correction overrides it

### Evidence from Phase 1 Test (test1_long.csv):
- **Clean baseline**: 7-11 µs std dev (good!)
- **Periodic spikes**: 175-200 µs every 5-30 minutes
- **Spike amplitude**: 60-80 µs above baseline
- **Pattern**: Rapid rise, slow decay (classic overshoot recovery)
- **Duration**: 5-50 samples per event

Example spike events:
- 24.7 min: Peak 166.8 µs
- 30.2 min: Peak 178.8 µs
- 57.7 min: Peak 158.0 µs
- 64.8 min: Peak 200.0 µs

---

## Expected Improvements

### Target Metrics (Phase 2):

| Metric | Phase 1 Baseline | Phase 2 Target | Improvement |
|--------|------------------|----------------|-------------|
| **Clean Std Dev** | 7-11 µs | 5-7 µs | 30-40% reduction |
| **Spike Peak** | 175-200 µs | <140 µs | 60-80 µs reduction |
| **Spike Frequency** | Every 5-30 min | Rare or eliminated | 90% reduction |
| **Overall Std Dev** | 15 µs | <10 µs | 33% reduction |
| **Convergence** | Oscillatory | Smooth | No overshoot |

### Why This Will Help:

1. **Gentler corrections prevent overshoot** (4× reduction in steady state)
   - System won't overcorrect past target
   - Kalman estimate respected more

2. **Kalman filter can work optimally**
   - Already provides optimal state estimate
   - 5% correction applies estimate gently
   - No fighting between Kalman and correction loop

3. **Smoother convergence trajectory**
   - 15% initial (vs 50%) prevents large jumps
   - 8% medium term allows gradual settling
   - 5% steady state maintains lock without disturbance

4. **Control theory fundamentals**
   - Lower gain = more stable
   - Kalman already does optimal filtering
   - Our job: Apply filtered estimate gently

---

## Test Plan

### Setup:
1. Build Phase 2 firmware with correction factor changes
2. Flash `slave/w5500_ptp_slave.uf2` to Pico W (Ethernet slave)
3. Ensure GM and measurement device are running
4. Reset slave and start fresh
5. Capture telemetry for **30 minutes minimum**

### Data Collection:
- **CSV file**: Save to `tests/phase2_correction_factor.csv`
- **Screenshot**: Capture final statistics dashboard and TIE chart
- **Notes**: Record spike events (time, amplitude)

### Success Criteria:

✅ **Primary Goals** (must achieve):
- Spike peaks < 140 µs (vs 200 µs in Phase 1)
- Spike frequency reduced by >50%
- Clean window std dev < 10 µs
- No severe overshoot visible in TIE chart

⭐ **Stretch Goals** (ideal):
- Spike peaks < 130 µs
- Spikes eliminated entirely
- Clean window std dev < 7 µs
- Overall std dev < 10 µs
- Smooth convergence with no oscillation

❌ **Failure Indicators**:
- Spikes still >175 µs
- Spike frequency unchanged
- Convergence takes >5 minutes
- New instability patterns appear

---

## Comparison Protocol

### Metrics to Compare (Phase 1 vs Phase 2):

1. **Clean Window Performance**:
   - Std dev (should be similar, 7-11 µs)
   - Mean (should be stable ~120 µs)
   - Range (should be 35-45 µs)

2. **Spike Behavior**:
   - Peak amplitude (should reduce from 200 → <140 µs)
   - Frequency (should reduce significantly)
   - Duration (should shorten)
   - Recovery pattern (should be faster)

3. **TIE Chart**:
   - Oscillation amplitude (should reduce)
   - Overshoot events (should eliminate)
   - Convergence pattern (should be smoother)

4. **Overall Statistics**:
   - Overall std dev (should improve from 15 → <10 µs)
   - Overall range (should narrow)
   - Outlier count (should decrease)

---

## Next Steps

### If Phase 2 Succeeds:
1. **Analyze results** and compare to Phase 1
2. **Decide** if Phase 3 (scale factor tracking) is needed
3. **Document** final performance vs baseline
4. **Consider** if Phase 4/5 needed for further refinement

### If Phase 2 Partially Succeeds:
- Spikes reduced but not eliminated → Try Phase 3 (scale factor)
- Or try intermediate values (e.g., 10% steady state instead of 5%)

### If Phase 2 Fails:
- Investigate why gentler corrections didn't help
- Check if spikes are from network, not control loop
- May need Phase 5 (median filtering) instead

---

## Rollback Plan

If Phase 2 makes things worse:

```bash
# Revert changes
git diff HEAD slave/ptp_discipline.c  # Check what changed
git checkout HEAD -- slave/ptp_discipline.c  # Revert

# Or manually restore Phase 1 values:
# correction_factor: 0.15 → 0.5, 0.08 → 0.3, 0.05 → 0.2

# Rebuild
cd build
make w5500_ptp_slave -j4

# Flash Phase 1 firmware
# Use slave/w5500_ptp_slave.uf2
```

---

## Build Info

```
Branch: slave-improvements
Commit: (pending after test)
Build Date: 2025-12-19
Firmware: slave/w5500_ptp_slave.uf2
Changes: Correction factors reduced 3-4× (Phase 2)
Previous: Phase 1 (Kalman noise parameters)
```

**Ready for testing!** 🚀

# Phase 1: Kalman Filter Noise Parameters - COMPLETED

## Branch: `slave-improvements`
## Date: 2025-12-19
## Build: `slave/w5500_ptp_slave.uf2` (216K)

---

## Changes Made

### File: `slave/ptp_discipline.c` (Lines 46-52)

**BEFORE (Baseline)**:
```c
// Kalman filter noise parameters
#define KALMAN_Q_OFFSET 1e8          // Process noise: offset variance (100 µs std dev)
#define KALMAN_Q_FREQ 1e-2           // Process noise: frequency variance (0.01 ppb std dev)
#define KALMAN_R_MEASUREMENT 4e10    // Measurement noise: network jitter (200 µs std dev)
```

**AFTER (Phase 1)**:
```c
// Kalman filter noise parameters
// PHASE 1 IMPROVEMENT: Fixed measurement noise to match actual Ethernet jitter
#define KALMAN_Q_OFFSET 1e6          // Process noise: offset variance (1 µs std dev)
#define KALMAN_Q_FREQ 1e-4           // Process noise: frequency variance (0.0001 ppb std dev)
#define KALMAN_R_MEASUREMENT 1e8     // Measurement noise: network jitter (10 µs std dev) - WAS 4e10 (200µs)
```

### Changes Summary:
| Parameter | Old Value | New Value | Change Factor | Rationale |
|-----------|-----------|-----------|---------------|-----------|
| `KALMAN_Q_OFFSET` | 1e8 (100µs) | 1e6 (1µs) | **100x smaller** | Process noise should be lower for stable crystal |
| `KALMAN_Q_FREQ` | 1e-2 | 1e-4 | **100x smaller** | Crystal very stable, low frequency drift |
| `KALMAN_R_MEASUREMENT` | 4e10 (200µs) | 1e8 (10µs) | **400x smaller** | Realistic Ethernet jitter (was way too pessimistic) |

---

## Problem Diagnosed

### Root Cause:
The Kalman filter was configured with **massively pessimistic measurement noise** (`R_MEASUREMENT = 200µs`), causing it to:
- Distrust network measurements
- Apply minimal corrections
- Allow large oscillations to persist
- Never converge tightly

### Evidence from Baseline:
- **Std Dev**: 20.6 µs (very high for Ethernet)
- **Range**: 217 µs (huge swings)
- **Spikes**: 115 µs excursions (periodic drops/peaks)
- **Oscillation**: Constant ±30µs jitter

---

## Expected Improvements

### Target Metrics (Phase 1 Only):

| Metric | Baseline | Target | Improvement |
|--------|----------|--------|-------------|
| **Std Dev** | 20.6 µs | 5-10 µs | 50-75% reduction |
| **Range** | 217 µs | 30-50 µs | 75-85% reduction |
| **Spikes** | 115 µs | <10 µs | 90% reduction |
| **Convergence** | Never | <5 min | Achieves tight lock |

### Why This Will Help:

1. **Kalman trusts measurements more** (400x smaller R)
   - Will apply larger corrections when offset detected
   - Faster convergence to true offset

2. **Lower process noise** (100x smaller Q)
   - Tells Kalman the crystal is stable
   - Won't allow wild state estimates

3. **Realistic Ethernet assumptions**
   - 10µs std dev matches actual network jitter
   - Filter can distinguish real offsets from noise

---

## Test Plan

### Setup:
1. Flash `slave/w5500_ptp_slave.uf2` to Pico W (Ethernet slave)
2. Ensure GM and measurement device are running
3. Reset slave and start fresh
4. Capture telemetry for **30 minutes minimum**

### Data Collection:
- **CSV file**: Save to `tests/phase1_kalman_fix.csv`
- **Screenshot**: Capture final statistics dashboard
- **Notes**: Record any anomalies or observations

### Success Criteria:

✅ **Primary Goals** (must achieve):
- Std Dev < 10 µs after 10 minutes
- No spikes > 20 µs after lock
- Visible convergence in TIE chart (tightening over time)

⭐ **Stretch Goals** (ideal):
- Std Dev < 5 µs after 20 minutes
- Range < 30 µs total
- Stable lock within 5 minutes

❌ **Failure Indicators**:
- Std Dev still > 15 µs after 20 minutes
- Spikes persist > 50 µs
- No visible improvement from baseline

---

## Comparison Protocol

### Metrics to Compare (Baseline vs Phase 1):

1. **All-Time Statistics**:
   - Mean offset (should be similar)
   - Std dev (should be 50-75% lower)
   - Min/Max range (should be 75-85% smaller)

2. **TIE Chart**:
   - Oscillation amplitude (should reduce dramatically)
   - Spike frequency/amplitude (should eliminate or greatly reduce)
   - Convergence pattern (should tighten over time)

3. **Histogram**:
   - Distribution width (should narrow significantly)
   - Shape (should remain Gaussian but tighter)

4. **Time to Lock**:
   - Baseline: Never achieved tight lock
   - Phase 1: Should lock within 5-10 minutes

---

## Next Steps

### If Phase 1 Succeeds:
1. **Analyze results** and compare to baseline
2. **Decide** if Phase 2 (correction aggressiveness) is needed
3. **Proceed** only if further improvements desired

### If Phase 1 Partially Succeeds:
- Improvement but not hitting targets → Try Phase 2

### If Phase 1 Fails:
- Investigate why Kalman changes didn't help
- May need to check network quality or hardware issues
- Review filter implementation for bugs

---

## Rollback Plan

If Phase 1 makes things worse:

```bash
# Revert changes
git checkout feature/relative-phase-measurement -- slave/ptp_discipline.c

# Rebuild
cd build
make w5500_ptp_slave -j4

# Flash baseline firmware
# Use slave/w5500_ptp_slave.uf2
```

Or switch back to `feature/relative-phase-measurement` branch.

---

## Build Info

```
Branch: slave-improvements
Commit: (pending after test)
Build Date: 2025-12-19 10:29
Firmware: slave/w5500_ptp_slave.uf2 (216K)
```

**Ready for testing!** 🚀

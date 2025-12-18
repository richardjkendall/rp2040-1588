# PTP Slave Performance Improvement Plan

## Problem Analysis

### Current Performance (from TIE chart):
- **Baseline Jitter**: 20-40µs peak-to-peak oscillation in stable regions
- **Periodic Disturbances**: Large dips (~60µs) every ~27-28 minutes
- **No Convergence**: Oscillation amplitude stays constant, no tightening over time
- **Mean Offset**: ~115µs (acceptable, just needs to be corrected)

### Root Causes Identified:

#### 1. **Kalman Filter Misconfiguration** (PRIMARY ISSUE)
**Location**: `slave/ptp_discipline.c:46-51`

```c
#define KALMAN_Q_OFFSET 1e8          // Process noise: offset (100 µs std dev)
#define KALMAN_Q_FREQ 1e-2           // Process noise: frequency (0.01 ppb std dev)
#define KALMAN_R_MEASUREMENT 4e10    // Measurement noise: network (200 µs std dev)
```

**Problem**: `KALMAN_R_MEASUREMENT = 4e10` (200µs std dev) is **WAY too high**
- Kalman filter doesn't trust measurements → barely corrects
- For Ethernet PTP, typical jitter is 5-20µs, not 200µs
- This causes slow convergence and allows oscillations

#### 2. **Over-Aggressive Clock Correction** (SECONDARY ISSUE)
**Location**: `slave/ptp_discipline.c:808-825`

```c
if (state.discipline_updates < 30) {
    correction_factor = 0.5;     // 50% correction
} else if (state.discipline_updates < 100) {
    correction_factor = 0.3;     // 30% correction
} else {
    correction_factor = 0.2;     // 20% steady state
}
int64_t correction = (int64_t)(filtered_offset_ns * correction_factor);
state.ptp_clock_ns -= correction;
```

**Problem**: 20-50% correction is too aggressive
- Causes overshoot → oscillation
- Classic control system instability (fighting itself)
- Should be 5-10% in steady state

#### 3. **Scale Factor Update Too Slow**
**Location**: `slave/ptp_discipline.c:829-836, 729-742`

```c
// Heavy filtering: 95% old, 5% new (20 second time constant)
state.scale_factor = state.scale_factor * 0.95 + kalman_scale * 0.05;

// Counter characterization: 98% old, 2% new (50 second time constant)
state.scale_factor = state.scale_factor * 0.98 + new_scale * 0.02;
```

**Problem**: Time constants way too long (20-50 seconds)
- Can't track actual crystal drift/temperature changes
- Causes accumulated timing errors
- Should be ~5-10 seconds for Ethernet

#### 4. **Periodic Pattern (~30min dips)**
**Hypothesis**: Likely correlation with:
- WiFi operations on measurement device (telemetry batches every 60s?)
- TCXO temperature cycling
- Network congestion patterns

**Need**: Better outlier rejection, measurement filtering

---

## Systematic Improvement Plan

### **PHASE 1: Fix Kalman Filter Noise Parameters** ⭐ HIGHEST PRIORITY
**File**: `slave/ptp_discipline.c`
**Lines**: 46-51

**Change**:
```c
// OLD VALUES:
#define KALMAN_Q_OFFSET 1e8          // 100 µs std dev - TOO HIGH
#define KALMAN_Q_FREQ 1e-2           // 0.01 ppb std dev
#define KALMAN_R_MEASUREMENT 4e10    // 200 µs std dev - WAY TOO HIGH

// NEW VALUES (realistic for Ethernet):
#define KALMAN_Q_OFFSET 1e6          // 1 µs std dev (process noise)
#define KALMAN_Q_FREQ 1e-4           // 0.0001 ppb std dev (very stable crystal)
#define KALMAN_R_MEASUREMENT 1e8     // 10 µs std dev (realistic Ethernet jitter)
```

**Rationale**:
- Ethernet jitter is typically 5-20µs, not 200µs
- Trusting measurements more allows faster convergence
- Process noise should be lower than measurement noise for stable crystal

**Expected Result**:
- Kalman will trust measurements more
- Faster convergence to true offset
- Reduced oscillation amplitude (from 40µs down to ~5-10µs)

**Test Duration**: 10 minutes
**Success Criteria**: Peak-to-peak jitter < 10µs after 5 minutes

---

### **PHASE 2: Reduce Clock Correction Aggressiveness**
**File**: `slave/ptp_discipline.c`
**Lines**: 808-825

**Change**:
```c
// OLD VALUES:
if (state.discipline_updates < 30) {
    correction_factor = 0.5;     // 50% - causes overshoot
} else if (state.discipline_updates < 100) {
    correction_factor = 0.3;     // 30% - still too high
} else {
    correction_factor = 0.2;     // 20% - marginal stability
}

// NEW VALUES (gentler, more stable):
if (state.discipline_updates < 30) {
    correction_factor = 0.15;    // 15% during convergence
} else if (state.discipline_updates < 100) {
    correction_factor = 0.08;    // 8% medium term
} else {
    correction_factor = 0.05;    // 5% steady state (GENTLE!)
}
```

**Rationale**:
- 20% correction means system overcorrects → oscillation
- 5% gentle correction allows Kalman to do its job
- Kalman already provides optimal state estimate - don't fight it

**Expected Result**:
- Smoother convergence without overshoot
- Elimination of oscillation
- Stable lock within 2-3 minutes

**Test Duration**: 15 minutes
**Success Criteria**: No visible oscillation, offset converges smoothly to <1µs

---

### **PHASE 3: Speed Up Scale Factor Tracking**
**File**: `slave/ptp_discipline.c`
**Lines**: 729-742 (counter characterization), 829-836 (Kalman-derived)

**Change**:
```c
// OLD: 98% old, 2% new (50 second time constant) - TOO SLOW
state.scale_factor = state.scale_factor * 0.98 + new_scale * 0.02;

// NEW: 90% old, 10% new (10 second time constant)
state.scale_factor = state.scale_factor * 0.90 + new_scale * 0.10;

// Kalman-derived scale factor (line 835):
// OLD: 95% old, 5% new (20 second time constant)
state.scale_factor = state.scale_factor * 0.95 + kalman_scale * 0.05;

// NEW: 85% old, 15% new (~7 second time constant)
state.scale_factor = state.scale_factor * 0.85 + kalman_scale * 0.15;
```

**Rationale**:
- Crystal drift changes with temperature (minutes timescale)
- 50-second time constant can't track these changes
- 7-10 seconds is fast enough to track drift, slow enough to filter jitter

**Expected Result**:
- Scale factor tracks TCXO drift better
- Reduced accumulated timing errors
- Better long-term stability

**Test Duration**: 30 minutes
**Success Criteria**: Scale factor tracks temperature-induced drift, no wandering offset

---

### **PHASE 4: Improve Path Delay Filtering** (Optional)
**File**: `slave/ptp_discipline.c`
**Lines**: 687-694

**Current**: α = 0.1 (10 second time constant)

**Change** (if Phase 1-3 don't fully solve it):
```c
// OLD:
#define KALMAN_ALPHA_LPF 0.1         // 10 sec time constant

// NEW (if needed):
#define KALMAN_ALPHA_LPF 0.05        // 20 sec time constant (more stable)
```

**Rationale**: Path delay should be very stable on a local network, can afford heavier filtering

**Test Duration**: 10 minutes
**Success Criteria**: Path delay std dev < 2µs

---

### **PHASE 5: Add Measurement Pre-Filtering** (If periodic dips persist)
**File**: `slave/ptp_discipline.c`
**Location**: Before Kalman update (around line 700)

**Add**: Median filter on measured offset (window size = 3-5)

```c
// Add near line 700, before Kalman update:
static double offset_buffer[5] = {0};
static int offset_idx = 0;

// Store measurement
offset_buffer[offset_idx] = measured_offset;
offset_idx = (offset_idx + 1) % 5;

// Use median instead of raw measurement
double sorted[5];
memcpy(sorted, offset_buffer, sizeof(sorted));
// Simple bubble sort for median
for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4-i; j++) {
        if (sorted[j] > sorted[j+1]) {
            double tmp = sorted[j];
            sorted[j] = sorted[j+1];
            sorted[j+1] = tmp;
        }
    }
}
double filtered_measurement = sorted[2];  // Median

// Use filtered_measurement instead of measured_offset for Kalman
```

**Rationale**: Median filter removes outlier spikes while preserving true signal

**Test Duration**: 30 minutes
**Success Criteria**: Periodic dips eliminated or reduced to <10µs

---

## Implementation Strategy

### **Scientific Approach** (One Change at a Time):

1. **Baseline Measurement** (30 minutes):
   - Current performance capture
   - Download CSV, note oscillation amplitude and period
   - Record MTIE, Allan deviation baseline

2. **Phase 1: Kalman Noise** (MOST IMPORTANT):
   - Change ONLY Kalman noise parameters
   - Test for 30 minutes
   - Download data, compare to baseline
   - **Decision**: If this alone fixes it → DONE. Otherwise proceed to Phase 2.

3. **Phase 2: Correction Factor**:
   - Add reduced correction factors
   - Test for 30 minutes
   - Download data, compare
   - **Decision**: Check for overshoot elimination

4. **Phase 3: Scale Factor**:
   - Add faster scale factor tracking
   - Test for 1 hour (need time to see drift tracking)
   - **Decision**: Check long-term stability

5. **Phase 4 & 5: Optional refinements**:
   - Only if still seeing issues after Phase 1-3

### **Testing Protocol for Each Phase**:

**Before**:
1. Reset statistics on telemetry server
2. Note current time
3. Let run for test duration

**After**:
1. Download CSV data
2. Check outlier count
3. Look at TIE chart:
   - Peak-to-peak amplitude
   - Convergence trend
   - Periodic patterns
4. Check MTIE chart:
   - Should be flat or slowly increasing
   - Not oscillating
5. Check Allan Deviation:
   - Should decrease with τ initially (white noise)
   - Then flatten (crystal stability)

**Success Metrics**:
- TIE: < 5µs peak-to-peak after 10 minutes
- MTIE (1s): < 10µs
- Allan Dev (1s): < 5µs
- No visible oscillation
- No periodic dips > 10µs

---

## Predicted Outcomes

### **After Phase 1** (Kalman Fix):
- Oscillation amplitude: 40µs → 10µs (75% improvement)
- Convergence time: Never → 5 minutes
- Lock quality: Loose (100µs) → Moderate (10µs)

### **After Phase 2** (Correction Factor):
- Oscillation amplitude: 10µs → 2µs (95% improvement)
- Overshoot: Eliminated
- Lock quality: Moderate → Good (2µs)

### **After Phase 3** (Scale Factor):
- Long-term drift: Corrected
- Temperature tracking: Improved
- Lock quality: Good → Excellent (<1µs)

### **Final Target** (all phases):
- **Short-term (1-10s)**: < 1µs jitter
- **Long-term (>1min)**: < 100ns RMS offset
- **MTIE (1s)**: < 5µs
- **MTIE (1000s)**: < 20µs
- **No periodic disturbances** > 5µs

---

## Rollback Plan

Each phase is independent. If a change makes things worse:
1. Revert that specific change
2. Rebuild firmware
3. Flash slave
4. Return to previous working state

Keep baseline measurements to compare against!

---

## Next Steps

1. **Review this plan** - Approve approach
2. **Capture baseline** - Run current firmware for 30 min, download data
3. **Phase 1 implementation** - Start with Kalman noise parameters
4. **Iterate** - One phase at a time, measuring results

**Question**: Ready to proceed with Phase 1 (Kalman noise parameters)?

# PTP Clock Servo Approaches

## Overview

A PTP servo is the control algorithm that synchronizes a slave clock to a master clock. The servo receives offset measurements from the PTP protocol and adjusts the local clock to minimize this offset. Different approaches exist for handling the initial offset and maintaining synchronization.

## Key Concepts

### Offset vs Frequency

- **Offset**: The time difference between slave and master clocks (e.g., slave is 143µs ahead)
- **Frequency**: The rate difference between clocks (e.g., slave runs 0.999992x master rate)

### Clock Adjustment Methods

1. **STEP (Jump)**: Immediately change clock value
   - Pros: Fast convergence, eliminates large offsets instantly
   - Cons: Discontinuous, can break applications expecting monotonic time
   - When: Large offsets (>threshold)

2. **SLEW (Frequency adjustment)**: Gradually adjust by changing clock frequency
   - Pros: Smooth, no time discontinuities
   - Cons: Slow convergence, can take hours for large offsets
   - When: Small offsets (<threshold)

## Common Approaches

### 1. Pure Frequency Control (Our Current Implementation)

**Description**: Only adjust frequency, never step the clock.

**Algorithm**:
```c
// Calculate offset
offset = (t2 - t1) - path_delay;

// Use PI controller or Kalman filter to adjust frequency
frequency_adjustment = servo_update(offset);
scale_factor = 1.0 + frequency_adjustment;
```

**Characteristics**:
- Offset converges **asymptotically** (never reaches exactly zero)
- Convergence time: hours to days for large offsets
- Can create oscillations if fighting persistent bias
- **Not recommended for offsets >1µs**

**When to use**:
- Systems requiring strictly monotonic time
- When step threshold is set to infinity
- Not typically used in real PTP implementations

**Our Results**:
- Persistent 143µs offset
- 60-80s oscillations (100µs amplitude)
- Scale factor: 0.999992 (constantly compensating)
- Actual performance: 23µs std dev (good stability, poor accuracy)

---

### 2. Step Threshold (Industry Standard)

**Description**: Step for large offsets, slew for small offsets.

**Algorithm**:
```c
offset = (t2 - t1) - path_delay;

if (abs(offset) > STEP_THRESHOLD) {
    // STEP: Directly adjust clock base
    clock_base += offset;
    offset = 0;
    LOG("Stepped clock by %lld ns", offset);
} else {
    // SLEW: Use servo to adjust frequency
    frequency_adjustment = servo_update(offset);
    scale_factor = 1.0 + frequency_adjustment;
}
```

**Characteristics**:
- Fast initial acquisition (sub-second)
- Smooth tracking after convergence
- Configurable threshold (1s default, can be 1ms-100ms)
- Industry standard approach

**Common Thresholds**:
- **LinuxPTP default**: 1 second
- **High-precision systems**: 1-10ms
- **Ultra-precision (our case)**: 50-100µs

**When to use**:
- All real-world PTP implementations
- When fast convergence is needed
- When offset accuracy is critical

**Expected Results for Our System**:
- Initial step eliminates 143µs offset
- Residual offset: <10µs after convergence
- Oscillations eliminated
- Scale factor converges to ~1.0

---

### 3. Adaptive Step Threshold

**Description**: Dynamic threshold based on system state.

**Algorithm**:
```c
// Adjust threshold based on lock status
if (!locked) {
    step_threshold = 1000000;  // 1ms during acquisition
} else if (recently_locked) {
    step_threshold = 100000;   // 100µs after initial lock
} else {
    step_threshold = 10000;    // 10µs when stable
}

if (abs(offset) > step_threshold) {
    clock_base += offset;
} else {
    frequency_adjustment = servo_update(offset);
}
```

**Characteristics**:
- More aggressive during acquisition
- Tighter threshold when locked
- Can handle transient disturbances
- More complex to implement

**When to use**:
- Systems with varying environmental conditions
- When both fast acquisition and high stability are needed
- Advanced implementations

---

### 4. PI Servo (LinuxPTP Style)

**Description**: Proportional-Integral controller for frequency adjustment.

**Algorithm**:
```c
// PI controller gains
kp = 0.7;   // Proportional gain
ki = 0.3;   // Integral gain

// Update integral term
integral += offset * dt;

// Calculate frequency adjustment
freq_adj = kp * offset + ki * integral;

// Apply with limits
scale_factor = 1.0 + clamp(freq_adj, -MAX_PPM, +MAX_PPM);
```

**Characteristics**:
- Well-understood control theory
- Tunable gains for different systems
- Can eliminate steady-state error (via integral term)
- Standard in many PTP implementations

**Tuning**:
- Higher kp: faster response, more overshoot
- Higher ki: eliminates steady-state error, can cause oscillations
- Typical values: kp=0.7, ki=0.3 for PTP

---

### 5. Kalman Filter Servo (Our Current Approach)

**Description**: Statistical estimator combining offset and frequency estimates.

**Algorithm**:
```c
// Kalman filter predicts offset and frequency drift
state_prediction = A * state + B * control;
uncertainty = A * P * A' + Q;

// Update with new offset measurement
innovation = measured_offset - predicted_offset;
kalman_gain = uncertainty * H' / (H * uncertainty * H' + R);
state = state_prediction + kalman_gain * innovation;
```

**Characteristics**:
- Optimal for systems with Gaussian noise
- Can track both offset and frequency drift
- More complex than PI servo
- Better noise rejection

**Tuning**:
- Q: Process noise (how much we trust the model)
- R: Measurement noise (how much we trust measurements)
- Trade-off between responsiveness and stability

---

## Comparison Table

| Approach | Convergence Time | Accuracy | Stability | Complexity | Monotonic Time |
|----------|-----------------|----------|-----------|------------|----------------|
| Pure Frequency | Hours-Days | Poor (>100µs) | Good | Low | Yes |
| Step Threshold | <1 second | Excellent (<1µs) | Excellent | Medium | No (during step) |
| Adaptive Step | <1 second | Excellent (<1µs) | Excellent | High | No (during step) |
| PI Servo | Minutes | Good (<10µs) | Good | Medium | Yes (if no step) |
| Kalman Filter | Minutes | Good (<10µs) | Excellent | High | Yes (if no step) |

## Latency Asymmetry Considerations

### The Problem

PTP assumes symmetric path delays:
```
Forward delay = Reverse delay
```

But in reality:
```
Forward delay  = GM_TX_latency + network + Slave_RX_latency
Reverse delay  = Slave_TX_latency + network + GM_RX_latency
```

If `GM_TX != Slave_TX` or `GM_RX != Slave_RX`, the offset calculation is biased.

### Our Measured Asymmetry

From diagnostics:
```
Forward:  GM_TX(265µs) + Slave_RX(135µs) = 400µs
Reverse:  Slave_TX(208µs) + GM_RX(115µs) = 323µs
Asymmetry: 77µs
Offset bias: ~39µs (half the asymmetry)
```

### Solutions

1. **Hardware timestamps closer to wire** - Reduce latency variability
2. **Asymmetry compensation** - Add known bias to offset calculation
3. **E2E transparent clocks** - Network devices correct for their delays
4. **Better timestamping points** - Timestamp at PHY instead of application

### Asymmetry Compensation Example

```c
// Measured from diagnostics
const int64_t asymmetry_correction_ns = 39000;  // 39µs

// Apply to offset calculation
measured_offset = (t2 - t1) - path_delay;
corrected_offset = measured_offset - asymmetry_correction_ns;

// Use corrected offset in servo
servo_update(corrected_offset);
```

**Caution**: This assumes asymmetry is constant. If latencies vary (thermal drift, load changes), the correction becomes invalid.

---

## Recommendations for Our System

### Current Status
- Offset: 143µs (persistent)
- Latency asymmetry: 77µs → contributes ~39µs error
- Remaining error: ~104µs (likely clock initialization offset)
- Oscillations: 60-80s period, 100µs amplitude

### Recommended Approach: **Step Threshold + PI Servo**

**Phase 1: Implement Step Threshold**
```c
#define STEP_THRESHOLD_NS 50000  // 50µs

if (abs(offset) > STEP_THRESHOLD_NS) {
    ptp_clock_base_ns += offset;
    offset = 0;
}
```

**Expected improvement:**
- Initial offset: 143µs → <10µs (after first step)
- Convergence time: Hours → <1 second
- Oscillations: Eliminated

**Phase 2: Tune Existing Servo**
- Keep Kalman filter for stability
- Adjust Q/R parameters if needed
- Monitor for overshoot

**Phase 3: Consider Asymmetry Compensation (Optional)**
- If residual error >10µs persists
- Add constant 39µs correction
- Validate with measurements

### Performance Targets

| Metric | Current | Target (Step) | Target (+ Compensation) |
|--------|---------|---------------|------------------------|
| Mean Offset | 143µs | <10µs | <5µs |
| Std Dev | 23µs | <5µs | <5µs |
| Convergence | Never | <1s | <1s |
| Oscillations | 100µs p-p | <10µs p-p | <10µs p-p |

---

## Implementation Priorities

1. **✅ Hardware timestamping** - Already implemented, working well (87% RX, 100% TX)
2. **✅ RX averaging fallback** - Already implemented, improved stability
3. **🔄 Step threshold** - Next critical improvement
4. **🔜 Servo tuning** - After step threshold working
5. **🔜 Asymmetry compensation** - If needed for <5µs accuracy

## References

- IEEE 1588-2008 Standard (Annex J: Best Master Clock Algorithm)
- LinuxPTP documentation (servo algorithms)
- "PTP Servo Design" - Intel PTP for Linux whitepaper
- "Time Synchronization in Packet Networks" - Eidson (2006)

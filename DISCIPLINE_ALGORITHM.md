# GPS Clock Discipline Algorithm

## Overview

This document explains how the GPS-disciplined clock algorithm works in this project. The algorithm uses a **software Phase-Locked Loop (PLL)** with a **PI (Proportional-Integral) controller** to discipline a local clock to GPS time.

---

## The Problem

The Raspberry Pi Pico has a crystal oscillator that's supposed to run at a precise frequency, but in reality, all crystals have some error:

- Crystal might be **±50 ppm** (parts per million) off spec when manufactured
- Temperature changes cause **±10-20 ppm** drift
- Aging causes gradual frequency shift over time

**Example:** A crystal that's 100 ppm slow will drift **8.64 seconds per day**.

The GPS provides a **1 PPS (Pulse Per Second)** signal that's extremely accurate (±50 nanoseconds relative to UTC), and we use it to correct our local clock.

---

## High-Level Concept

Think of the algorithm as a feedback loop:

```
          ┌─────────────────────────────────┐
          │                                 │
          │         PI Controller           │
          │                                 │
          └────────────┬────────────────────┘
                       │
                       ↓ frequency_offset_ppb
          ┌─────────────────────────────────┐
          │                                 │
GPS PPS ──→    Disciplined Clock            ├──→ Output (100 PPS)
1 Hz      │    (software counter)           │
          │                                 │
          └────────────┬────────────────────┘
                       │
                       ↓ phase_error
          ┌────────────────────────────────┐
          │   Compare: where are we        │
          │   vs. where should we be?      │
          └────────────────────────────────┘
```

1. **Disciplined Clock** runs continuously, counting nanoseconds
2. **GPS PPS** arrives every second (ground truth)
3. **Measure phase error**: How far off is our clock from GPS?
4. **PI Controller** calculates frequency correction
5. **Apply correction** to future clock updates
6. **Repeat** - clock converges to GPS time

---

## Data Structures

### Disciplined Clock State

```c
typedef struct {
    uint64_t nanoseconds;           // Current disciplined time (ns)
    int64_t phase_error_ns;         // Last measured phase error
    int32_t frequency_offset_ppb;   // Frequency correction (parts per billion)
    uint64_t last_update_time_us;   // Hardware timer at last update
    uint64_t last_reference_time_us; // Hardware timer at last GPS PPS
    bool locked;                    // Are we locked to GPS?

    // PI controller state
    double integral;                // Integral term accumulator
    uint32_t lock_counter;          // Consecutive good measurements
    double kp;                      // Proportional gain
    double ki;                      // Integral gain
} disciplined_clock_t;
```

---

## Algorithm Components

### 1. Clock Update (`discipline_update_time()`)

Called **continuously** (every loop iteration) to advance the disciplined clock.

```c
void discipline_update_time(disciplined_clock_t *clock) {
    // 1. Read hardware timer
    uint64_t now_us = time_us_64();
    uint64_t elapsed_us = now_us - clock->last_update_time_us;

    // 2. Convert to nanoseconds
    uint64_t elapsed_ns = elapsed_us * 1000ULL;

    // 3. Apply frequency correction
    //    corrected_time = elapsed_ns * (1 + frequency_offset/1e9)
    int64_t correction_ns = (elapsed_ns * frequency_offset_ppb) / 1000000000LL;
    int64_t corrected_elapsed_ns = elapsed_ns + correction_ns;

    // 4. Advance the disciplined clock
    clock->nanoseconds += corrected_elapsed_ns;
    clock->last_update_time_us = now_us;
}
```

**Example:**

- Hardware timer advances: **1,000,000 μs** (1 second)
- Frequency offset: **+100,000 ppb** (clock running slow, need to speed up)
- Correction: 1,000,000,000 ns × (100,000 / 1e9) = **+100,000 ns**
- Disciplined clock advances: 1,000,000,000 + 100,000 = **1,000,100,000 ns**

Result: Even though hardware timer only advanced 1 second, our disciplined clock advanced 1.0001 seconds, compensating for the slow crystal.

---

### 2. Reference Update (`discipline_update_reference()`)

Called **once per second** when GPS PPS arrives to measure and correct.

#### Step 1: Update Clock to Current Moment

```c
discipline_update_time(clock);  // Advance clock to GPS PPS arrival time
```

#### Step 2: Measure Phase Error

```c
// Where is our clock vs. where should it be?
int64_t phase_error = clock->nanoseconds - expected_interval_ns;
```

**Interpretation:**
- **phase_error = +3000 ns**: Clock reached 1,000,003,000 ns → running **FAST** by 3 μs
- **phase_error = -5000 ns**: Clock reached 999,995,000 ns → running **SLOW** by 5 μs
- **phase_error = 0**: Perfect! Clock reached exactly 1,000,000,000 ns

#### Step 3: PI Controller

The PI controller calculates how much to adjust the frequency based on the error.

##### Proportional Term (P)
Reacts to **current error**:

```c
double p_term = Kp * phase_error;
```

- `Kp = 0.1` means apply 10% of the error immediately
- Large errors → large correction (fast response)
- Can overshoot if too aggressive

**Example:**
- phase_error = +10,000 ns (10 μs fast)
- p_term = 0.1 × 10,000 / 1e9 = 1e-6 seconds

##### Integral Term (I)
Eliminates **long-term drift**:

```c
integral += phase_error;  // Accumulate all past errors
double i_term = Ki * integral;
```

- `Ki = 0.001` means apply 0.1% of accumulated error
- If crystal consistently drifts, integral grows → larger correction
- Slow but eliminates steady-state error

**Example:**
- Errors over time: +10μs, +8μs, +6μs, +5μs, +4μs
- integral = sum of all errors / 1e9 = 33e-6 seconds
- i_term = 0.001 × 33e-6 = 3.3e-8 seconds

##### Combined Correction

```c
double correction = -(p_term + i_term);  // Negate for proper sign
frequency_offset_ppb = (int32_t)(correction * 1e9);
```

**Why negate?**
- Positive phase error (running fast) → need **negative** correction (slow down)
- Negative phase error (running slow) → need **positive** correction (speed up)

**Example:**
- phase_error = +10,000 ns (fast)
- p_term + i_term = 1.033e-6 seconds
- correction = -1.033e-6 seconds
- frequency_offset_ppb = -1033 ppb ← This slows down the clock

#### Step 4: Apply Limits

```c
// Clamp to ±1000 ppm (±1,000,000 ppb)
if (frequency_offset_ppb > 1000000) frequency_offset_ppb = 1000000;
if (frequency_offset_ppb < -1000000) frequency_offset_ppb = -1000000;
```

Prevents runaway corrections if something goes wrong.

#### Step 5: Update Lock Status

```c
if (abs(phase_error) < 1ms for 5 consecutive samples) {
    locked = true;
}
```

"Locked" means the clock is synchronized and tracking GPS accurately.

#### Step 6: Reset Clock

```c
clock->nanoseconds = 0;  // Reset to zero for next epoch
```

Clock resets to 0 on each GPS PPS and counts up to 1 second again. This is an **epoch-based** discipline approach.

---

## Convergence Example

Let's walk through what happens over time with a crystal that's **100 ppm slow** (runs 0.01% slow):

### Second 0 (GPS PPS #1 arrives)
- Initialize: phase_error = 0, frequency_offset = 0, integral = 0
- Clock resets to 0

### Second 1 (GPS PPS #2 arrives)
Hardware timer advanced: 1,000,100 μs (0.01% slow)

**Clock update (no correction yet):**
- elapsed_ns = 1,000,100,000 ns
- correction_ns = 0 (frequency_offset still 0)
- clock->nanoseconds = 1,000,100,000 ns

**Phase error measurement:**
- Expected: 1,000,000,000 ns
- Actual: 1,000,100,000 ns
- phase_error = **+100,000 ns** (we're fast relative to expectations because we measured too much time)

**PI Controller:**
- p_term = 0.1 × 100,000/1e9 = 1e-5
- integral = 100,000/1e9 = 1e-4
- i_term = 0.001 × 1e-4 = 1e-7
- correction = -(1e-5 + 1e-7) = -1.01e-5
- **frequency_offset_ppb = -10,100** (slow down by 10.1 ppm)

Wait, that seems backwards! The crystal is slow (100 ppm), but we're slowing it down more?

**Here's the key insight:** The phase error is **positive** because our software clock counted 1,000,100,000 ns when it should have counted 1,000,000,000 ns. This happened because the hardware timer was slow (took 1.0001 seconds to reach what we call "1 second").

Actually, I need to reconsider this. Let me trace through more carefully:

- Hardware timer is 100 ppm slow, so 1 real second = 1.0001 hardware seconds
- Wait, that's wrong. If crystal is 100 ppm slow, 1 hardware second = 0.9999 real seconds
- So in 1 real second, hardware counts 999,900 μs
- Our clock would count 999,900,000 ns
- phase_error = 999,900,000 - 1,000,000,000 = **-100,000 ns** (running slow)
- correction = -(-1.01e-5) = +1.01e-5 (positive = speed up)
- frequency_offset_ppb = **+10,100** (speed up by 10.1 ppm)

That makes more sense!

### Second 2 (GPS PPS #3 arrives)
Hardware timer advances another 999,900 μs.

**Clock update (WITH correction now):**
- elapsed_ns = 999,900,000 ns
- correction_ns = 999,900,000 × 10,100/1e9 = 10,098,990 ns
- clock->nanoseconds = 999,900,000 + 10,098,990 = **1,009,998,990 ns**

**Phase error:**
- Expected: 1,000,000,000 ns
- Actual: 1,009,998,990 ns
- phase_error = **+9,998,990 ns** (now we overshot!)

**PI Controller adjusts:**
- Integral grows, but P term dominates
- frequency_offset_ppb reduces toward optimal value

### Seconds 3-10 (Convergence)
Over the next several seconds, the PI controller finds the equilibrium:

- phase_error oscillates: +10ms → -5ms → +2ms → -1ms → +500μs → -200μs
- frequency_offset_ppb converges to **≈+100,000 ppb** (exactly compensating for 100 ppm slow crystal)
- After 5 consecutive samples with phase_error < 1ms → **LOCKED**

### Steady State (After ~10 seconds)
- phase_error: ±100-500 ns (very small jitter)
- frequency_offset_ppb: +100,000 (stable)
- Clock accurately tracks GPS time

---

## Why This Works

1. **Fast Initial Response**: Proportional term quickly reduces large errors
2. **Eliminates Steady-State Error**: Integral term compensates for crystal drift
3. **Stable Convergence**: Gains (Kp=0.1, Ki=0.001) chosen to avoid oscillation
4. **Continuous Correction**: Clock update applies correction every microsecond
5. **GPS Reference**: 1 PPS provides ground truth for calibration

---

## Performance Expectations

### Phase Error
- **Initial**: Can be ±100 ms when starting up
- **After lock (10 seconds)**: ±1-10 μs typical
- **Long-term**: Limited by GPS PPS jitter (±50 ns) and software timestamping (±1 μs)

### Frequency Offset
- **Typical crystal**: ±50 ppm (±50,000 ppb)
- **With temperature**: ±100 ppm (±100,000 ppb)
- **Algorithm range**: ±1000 ppm (±1,000,000 ppb)

### Lock Time
- **Time to lock**: 5-15 seconds typical
- **Depends on**: Initial error, crystal stability, PI gains

---

## Tuning the Controller

### Proportional Gain (Kp)
- **Higher Kp**: Faster response, but can overshoot
- **Lower Kp**: Slower but more stable
- **Current value**: 0.1 (good balance)

### Integral Gain (Ki)
- **Higher Ki**: Faster elimination of steady-state error, risk of windup
- **Lower Ki**: Slower convergence, more stable
- **Current value**: 0.001 (conservative)

### To tune:
1. If phase error oscillates wildly → reduce Kp
2. If phase error settles but has offset → increase Ki
3. If lock takes too long → increase both gains proportionally
4. If system is unstable → reduce both gains

---

## Comparison to Other Approaches

### Simple Reset (No Discipline)
Just reset clock to 0 on each GPS PPS:
- ✅ Simple
- ❌ Clock jumps discontinuously
- ❌ Can't generate accurate high-frequency outputs (100 PPS, 1000 PPS)

### PI Controller (This Project)
- ✅ Smooth, continuous time
- ✅ Can generate arbitrary frequencies
- ✅ Learns crystal characteristics
- ⚠️ Requires tuning

### Full PLL with VCO
Hardware oscillator controlled by feedback:
- ✅ Best performance (sub-nanosecond)
- ✅ Analog smoothness
- ❌ Requires specialized hardware (OCXO, TCXO)
- ❌ Expensive

---

## Implementation Files

- **`common/discipline.h`**: Data structures and function declarations
- **`common/discipline.c`**: Algorithm implementation
  - `discipline_init()`: Initialize controller
  - `discipline_update_time()`: Advance clock with correction
  - `discipline_update_reference()`: Measure error and adjust
- **`grandmaster/discipline_core.c`**: Core 1 main loop that calls these functions

---

## Further Reading

- **Phase-Locked Loops**: "Frequency Synthesis by Phase Lock" by William F. Egan
- **Control Theory**: "Feedback Control of Dynamic Systems" by Franklin, Powell, Emami-Naeini
- **GPS Disciplined Oscillators**: "The Art of Electronics" (3rd Ed), Section 15.3.6
- **IEEE 1588 PTP**: Uses similar concepts for network time synchronization
- **NTP (Network Time Protocol)**: Uses PLL to discipline system clock to network time

---

## Debugging Tips

### Problem: Phase error is stuck at maximum
- **Cause**: Frequency offset hitting clamp limit (±1,000,000 ppb)
- **Solution**: Check if crystal is within ±1000 ppm spec, increase clamp if needed

### Problem: System won't lock (oscillates)
- **Cause**: PI gains too aggressive
- **Solution**: Reduce Kp and Ki by 50%, test, repeat

### Problem: Phase error has constant offset
- **Cause**: Integral term not accumulating (bug) or Ki too small
- **Solution**: Check integral accumulation logic, increase Ki slightly

### Problem: Lock achieved but then lost repeatedly
- **Cause**: GPS signal intermittent or noisy environment
- **Solution**: Check GPS antenna placement, add hysteresis to lock detection

### Serial Output to Monitor
Watch the statistics every 5 seconds:
```
Core 1 Stats: PPS count=27, Phase error=3000 ns, Freq offset=376 ppb, Locked=YES
```

- **Phase error decreasing**: Controller converging ✅
- **Phase error stable near 0**: Locked ✅
- **Freq offset constant**: Found equilibrium ✅
- **Freq offset at ±1,000,000**: Hitting limit ❌

---

## Conclusion

This GPS discipline algorithm provides a software-based, low-cost way to achieve microsecond-level time accuracy using commodity hardware. While not as accurate as hardware GPSDOs (GPS-Disciplined Oscillators), it demonstrates the fundamental principles of frequency synthesis and control theory in an accessible, educational implementation.

The same concepts apply to:
- **IEEE 1588 PTP**: Network time synchronization
- **NTP**: Internet time synchronization
- **Audio/Video PLLs**: Clock recovery in digital communication
- **Radio frequency synthesis**: Generating precise RF carriers

Understanding this algorithm provides a foundation for any application requiring precise timing and synchronization.

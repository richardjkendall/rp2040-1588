# Discipline Algorithm and Output Generation Improvements

This document explores various approaches to improve timing precision beyond the current software-based implementation running on Core 1.

---

## Current Implementation (Phase 1 & 2a)

### Architecture
- **Core 1**: Runs discipline loop in `tight_loop_contents()`
- **Clock Update**: `discipline_update_time()` called every loop iteration (~microseconds)
- **Output Generation**: Direct GPIO toggle via `gpio_put()` when threshold crossed
- **Pulse Width**: 10 μs (short enough to minimize blocking)

### Performance
- **Phase Error**: ±5-10 μs typical (excellent for software)
- **Output Jitter**: ±10-50 μs estimated (varies with loop timing)
- **CPU Usage**: Core 1 dedicated to timing (100%)

### Limitations
- **Loop Jitter**: Time between threshold check and GPIO toggle varies
- **Interrupt Latency**: Any IRQ can delay threshold detection
- **Scalability**: Works for 100 PPS, marginal for 1 kHz, impossible for >10 kHz

---

## Improvement Option 1: Hardware Timer Alarms

### Concept

Use the RP2040's hardware timer to generate interrupts at precise times. Instead of polling in a loop, schedule an alarm for the exact moment the next pulse should occur.

### Architecture

```c
// Global state
static uint64_t next_pulse_time_us;
static disciplined_clock_t* clock_ptr;

// Alarm IRQ handler (runs in interrupt context)
void alarm_irq_handler() {
    // Generate output pulse (hardware-precise edge)
    gpio_put(OUTPUT_PIN, 1);
    busy_wait_us(1);  // Short pulse
    gpio_put(OUTPUT_PIN, 0);

    // Calculate next pulse time from disciplined clock
    next_pulse_time_us = calculate_next_pulse_from_disciplined_clock(clock_ptr);

    // Schedule next alarm
    hardware_alarm_set_target(ALARM_NUM, (uint32_t)(next_pulse_time_us & 0xFFFFFFFF));

    // Clear IRQ
    hardware_alarm_clear_irq(ALARM_NUM);
}

// Core 1 main loop
void core1_entry() {
    // Initialize hardware alarm
    hardware_alarm_claim(ALARM_NUM);
    hardware_alarm_set_callback(ALARM_NUM, alarm_irq_handler);

    // Schedule first alarm
    next_pulse_time_us = time_us_64() + 1000000;  // 1 second from now
    hardware_alarm_set_target(ALARM_NUM, (uint32_t)next_pulse_time_us);

    while (true) {
        // Just run discipline loop
        discipline_update_time(&disciplined_clock);

        // Check for GPS PPS
        gps_pps_t pps;
        if (gps_get_pps(&pps)) {
            discipline_update_reference(...);
        }

        // No output generation here - alarm handles it!
        tight_loop_contents();
    }
}
```

### Advantages
✅ **Hardware-timed edges** - Alarm fires at precise hardware timer value
✅ **Interrupt-based** - No polling overhead
✅ **Lower jitter** - ±1-2 μs (interrupt latency only)
✅ **Core 1 freed up** - Can do other work between alarms
✅ **Scales to ~10 kHz** - Fast IRQ handling

### Disadvantages
⚠️ **IRQ latency** - Still 1-2 μs from interrupt to GPIO toggle
⚠️ **32-bit timer** - Hardware alarm uses lower 32 bits only (wraps every ~71 minutes)
⚠️ **Complexity** - Need to manage alarm rescheduling, handle wraparound
⚠️ **Shared resource** - Only 4 hardware alarms available

### Implementation Complexity
**Medium** - Requires IRQ handling, wraparound management, careful synchronization with disciplined clock.

### Best For
- 100 PPS to 10 kHz output rates
- When Core 1 needs to do other work besides timing
- When ±1-2 μs jitter is acceptable

---

## Improvement Option 2: PIO-Based Output Generation

### Concept

Use the RP2040's Programmable I/O (PIO) to generate output pulses with **hardware-level precision**. The PIO state machine can directly read the hardware timer and toggle GPIO pins without CPU involvement.

### Architecture Overview

**Method A: PIO Waits for Timer Value**

PIO state machine waits until hardware timer reaches a target value, then toggles output:

```pio
.program pps_output_timer

.wrap_target
    pull block              ; Wait for timestamp from CPU (when to pulse)
    mov x, osr              ; Store target time in X register

wait_loop:
    in pins, 32             ; Read current timer value (need timer on input pins - tricky!)
    mov y, isr
    jmp x!=y wait_loop      ; Wait until timer matches target

    set pins, 1             ; Rising edge (hardware-precise!)
    set pins, 0 [31]        ; Falling edge after delay
.wrap
```

**Problem**: PIO can't directly read the system timer! We'd need to route timer to GPIO inputs (complex) or use a different approach.

**Method B: PIO Generates Fixed-Rate Output, CPU Adjusts**

PIO generates pulses at a base rate, CPU adjusts the rate periodically:

```pio
.program pps_output_fixed

.wrap_target
    pull noblock            ; Check for rate adjustment from CPU
    mov x, osr              ; Update delay count

    set pins, 1             ; Rising edge
    mov y, x                ; Load delay count
delay_high:
    jmp y-- delay_high [31] ; Delay while high

    set pins, 0             ; Falling edge
    mov y, x
delay_low:
    jmp y-- delay_low [31]  ; Delay while low
.wrap
```

**Problem**: Rate changes aren't instant - takes effect at next cycle. Phase alignment to GPS is indirect.

**Method C: DMA + PIO for Precise Edges**

Use DMA to feed precise timing values to PIO. PIO uses clock divider and delay loops to generate edges:

```pio
.program pps_output_dma

.wrap_target
    pull block              ; Get delay value from DMA
    out x, 32               ; Load into X

    set pins, 1             ; Rising edge

delay_loop:
    jmp x-- delay_loop      ; Precise delay

    set pins, 0             ; Falling edge
.wrap
```

CPU/DMA calculates delay values based on disciplined clock and feeds PIO FIFO.

### PIO State Machine Integration

```c
// Core 1 feeds timestamps to PIO
void core1_entry() {
    // Initialize PIO
    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &pps_output_program);
    pio_sm_init(pio, sm, offset, &config);

    while (true) {
        discipline_update_time(&disciplined_clock);

        // When output should occur
        uint64_t current_ns = discipline_get_time_ns(&disciplined_clock);
        if (current_ns >= next_output_pulse_ns) {
            // Calculate delay for PIO
            uint32_t delay_cycles = calculate_pio_delay(current_ns, next_output_pulse_ns);

            // Send to PIO (non-blocking if FIFO full)
            if (!pio_sm_is_tx_fifo_full(pio, sm)) {
                pio_sm_put(pio, sm, delay_cycles);
            }

            next_output_pulse_ns += OUTPUT_INTERVAL_NS;
        }

        // GPS discipline
        gps_pps_t pps;
        if (gps_get_pps(&pps)) {
            discipline_update_reference(...);
        }
    }
}
```

### Advantages
✅ **Hardware-precise edges** - Sub-microsecond jitter (<100 ns)
✅ **Zero CPU involvement** - PIO runs independently
✅ **Scales to MHz** - PIO can generate very high frequencies
✅ **Multiple outputs** - Can drive multiple pins simultaneously
✅ **Deterministic** - No interrupt latency

### Disadvantages
⚠️ **Complex implementation** - PIO programming is tricky
⚠️ **Limited PIO resources** - Only 8 state machines total (4 per PIO block)
⚠️ **Indirect timer access** - PIO can't directly read system timer
⚠️ **Synchronization challenge** - Aligning PIO output to GPS PPS requires careful design

### Implementation Complexity
**High** - Requires deep understanding of PIO, DMA, and precise synchronization mechanisms.

### Best For
- >10 kHz output rates
- Multiple synchronized outputs
- Absolute minimum jitter requirements (<1 μs)
- When CPU resources are precious

---

## Improvement Option 3: Hybrid Alarm + PIO

### Concept

Combine hardware alarms with PIO for best of both worlds:
- **Hardware Alarm**: Triggers at precise GPS-disciplined time
- **PIO**: Generates clean output pulse when triggered

### Architecture

```c
// Alarm IRQ triggers PIO
void alarm_irq_handler() {
    // Trigger PIO to generate pulse
    pio_sm_put(pio, sm, 1);  // Send trigger to PIO

    // Schedule next alarm
    next_pulse_time_us += pulse_interval_us;
    hardware_alarm_set_target(ALARM_NUM, next_pulse_time_us);
}
```

```pio
.program pps_output_triggered

.wrap_target
    pull block              ; Wait for trigger from CPU/alarm
    set pins, 1             ; Rising edge (immediate)
    set pins, 0 [31]        ; Falling edge after precise delay
.wrap
```

### Advantages
✅ **Simple PIO program** - Just generates pulse on command
✅ **Alarm handles timing** - Easier to synchronize with disciplined clock
✅ **Clean edges** - PIO ensures consistent pulse shape
✅ **Moderate complexity** - Easier than pure PIO solution

### Disadvantages
⚠️ **IRQ latency** - ~1-2 μs from alarm to PIO trigger
⚠️ **Not as precise as pure PIO** - But better than pure software

### Implementation Complexity
**Medium** - Simpler than pure PIO, more complex than pure alarm.

### Best For
- 1 kHz to 100 kHz rates
- When you want better jitter than software but simpler than pure PIO
- Good balance of precision and complexity

---

## Improvement Option 4: External Hardware Disciplined Oscillator (GPSDO)

### Concept

Instead of software discipline, use the GPS PPS to discipline a **hardware oscillator** via analog control loop.

### Architecture

```
GPS PPS ──→ [Phase Detector] ──→ [Loop Filter] ──→ [VCXO/OCXO] ──→ High-precision clock
                    ↑                                      │
                    └──────── [Frequency Divider] ←────────┘
```

- **Phase Detector**: Compares GPS PPS to divided oscillator output
- **Loop Filter**: Analog or digital PI/PID controller
- **VCXO/OCXO**: Voltage-controlled or oven-controlled crystal oscillator
- **Output**: Extremely stable, low-jitter clock signal

### Example Hardware
- **OCXO**: ±0.01 ppm stability, <1 ns jitter
- **TCXO**: ±0.5 ppm stability, <10 ns jitter
- **Phase Detector**: XOR gate, edge-triggered flip-flop, or dedicated IC (e.g., ADF4002)

### Advantages
✅ **Best possible performance** - Sub-nanosecond jitter
✅ **Analog smoothness** - No digital quantization
✅ **Free-running capability** - Continues even if GPS lost temporarily
✅ **No CPU load** - Completely external

### Disadvantages
❌ **Expensive** - OCXO modules cost $50-$500
❌ **Complex analog design** - Loop filter tuning is difficult
❌ **External hardware** - Not using Pico capabilities
❌ **Power consumption** - OCXO needs heater (watts)

### Implementation Complexity
**Very High** - Requires analog design skills, external components, PCB design.

### Best For
- Professional-grade timing applications
- When cost is not a concern
- Metrology, telecom, scientific instruments
- Learning about analog PLLs and GPSDOs

---

## Improvement Option 5: Better PI Controller Tuning

### Concept

Keep the current software architecture but optimize the PI controller for better performance.

### Advanced Techniques

#### 1. Adaptive Gains
Adjust Kp and Ki based on current phase error:

```c
// Aggressive gains when far from lock
if (abs(phase_error) > 100000) {  // >100 μs
    kp = 0.5;   // Fast response
    ki = 0.01;
}
// Conservative gains when close to lock
else if (abs(phase_error) < 10000) {  // <10 μs
    kp = 0.05;  // Gentle
    ki = 0.0005;
}
// Normal gains
else {
    kp = 0.1;
    ki = 0.001;
}
```

#### 2. Derivative Term (PID Controller)
Add D term to reduce overshoot:

```c
double d_term = kd * (phase_error - last_phase_error);
correction = -(p_term + i_term + d_term);
```

#### 3. Kalman Filter
Use optimal estimation for noisy GPS PPS:

```c
// Predict
x_predicted = x_estimate + velocity * dt;

// Update with GPS measurement
kalman_gain = covariance / (covariance + measurement_noise);
x_estimate = x_predicted + kalman_gain * (measurement - x_predicted);
```

#### 4. Integral Anti-Windup with Clamping
Better integral management:

```c
// Only integrate if not saturated
if (abs(freq_offset) < MAX_FREQ_OFFSET) {
    integral += error;
} else {
    // Back off integral if we're saturating
    integral *= 0.9;
}
```

### Advantages
✅ **No hardware changes** - Pure software improvement
✅ **Faster convergence** - Adaptive gains reach lock quicker
✅ **Lower steady-state error** - Better tracking
✅ **More robust** - Handles disturbances better

### Disadvantages
⚠️ **Still software-limited** - Can't beat fundamental loop jitter
⚠️ **Tuning complexity** - More parameters to optimize
⚠️ **Diminishing returns** - Only marginal improvement possible

### Implementation Complexity
**Low to Medium** - Depending on which techniques you use.

### Best For
- Quick wins with current architecture
- Learning control theory
- Incremental improvements before hardware changes

---

## Comparison Matrix

| Approach | Jitter | Complexity | CPU Load | Cost | Max Rate | Best For |
|----------|--------|------------|----------|------|----------|----------|
| **Current (GPIO polling)** | ±10-50 μs | Low | High | $0 | ~1 kHz | Learning, 100 PPS |
| **Hardware Alarm** | ±1-2 μs | Medium | Low | $0 | ~10 kHz | 1 kHz, balanced |
| **PIO Output** | <1 μs | High | Minimal | $0 | >1 MHz | High freq, precision |
| **Hybrid Alarm+PIO** | ±1-5 μs | Medium | Low | $0 | ~100 kHz | Good balance |
| **External GPSDO** | <1 ns | Very High | None | $50-500 | >100 MHz | Professional use |
| **Better PI Tuning** | ±5-20 μs | Low-Med | High | $0 | ~1 kHz | Quick improvement |

---

## Recommendations by Use Case

### Educational / Proof of Concept (This Project)
**Current approach is perfect:**
- Core 1 GPIO polling
- Maybe upgrade to Hardware Alarms for Phase 2b
- Focus on understanding concepts

### Hobbyist PTP Testbed
**Hardware Alarm approach:**
- Good balance of precision and complexity
- ±1-2 μs jitter is excellent for WiFi-based PTP
- Learn interrupt-driven design

### High-Precision Laboratory Instrument
**PIO or Hybrid approach:**
- Sub-microsecond jitter
- Can generate multiple synchronized outputs
- Complex but achievable on Pico

### Professional GPSDO Product
**External hardware oscillator:**
- OCXO or TCXO disciplined by GPS
- Analog loop filter
- Pico used just for monitoring/control

---

## Implementation Priority for This Project

### Phase 2a (Current): 100 PPS with GPIO Polling
- ✅ Already implemented
- ✅ Good enough to demonstrate concepts
- ✅ Measure actual jitter with Phase 4 monitor

### Phase 2b (Optional): Upgrade to Hardware Alarms
- ⚠️ Only if GPIO polling shows >50 μs jitter
- Moderate complexity, significant improvement
- Good learning experience with interrupts

### Phase 3 (Advanced): PIO Output Generation
- Only if pursuing >1 kHz rates or <1 μs jitter
- Significant complexity
- Diminishing educational returns for PTP demo

### Phase 4 (Monitoring): Build Monitor First
- **Most important next step**
- Measure actual performance
- Make data-driven decision about improvements
- See what's "good enough" vs "needs improvement"

---

## Further Reading

### Hardware Timers
- RP2040 Datasheet Chapter 4.6: Timer
- Pico SDK: `hardware_timer.h`, `hardware_alarm.h`

### PIO Programming
- RP2040 Datasheet Chapter 3: PIO
- Pico SDK: `hardware_pio.h`
- "Raspberry Pi Pico PIO" by Shawn Hymel

### Control Theory
- "Digital Control Engineering" by M. Sami Fadali
- "Understanding PID Control" series by Brian Douglas (YouTube)

### GPS Disciplined Oscillators
- HP Application Note 1279: "Fundamentals of Quartz Oscillators"
- "The Art of Electronics" (3rd Ed), Section 15.3.6
- Symmetricom GPS-Disciplined Oscillator Application Notes

### Phase-Locked Loops
- "Phase-Locked Loops: Design, Simulation, and Applications" by Roland Best
- "PLL Performance, Simulation and Design" by Dean Banerjee

---

## Conclusion

The current Core 1 GPIO polling approach is **excellent for educational purposes** and adequate for 100 PPS output. The Phase 4 monitor will reveal actual jitter and help decide if improvements are needed.

For most PTP learning applications, the current implementation is sufficient. Hardware alarms provide a good next step if more precision is needed. PIO and external hardware are for advanced users pursuing professional-grade performance.

**Recommendation**: Complete Phase 2a (100 PPS), build Phase 4 (monitor), measure performance, then decide on improvements based on actual data rather than speculation.

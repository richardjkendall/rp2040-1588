# GPS-Disciplined PTP Grandmaster Project Design
## RP2040W Implementation

---

## Project Overview

Build a precision timing testbed using PTP (Precision Time Protocol / IEEE 1588) to demonstrate clock synchronization between microcontrollers. The system consists of three RP2040W devices:

1. **PTP Grandmaster**: GPS-disciplined clock acting as the timing authority
2. **PTP Slave**: Ordinary clock synchronized to the grandmaster
3. **Reference Monitor**: Independent GPS reference for measuring synchronization quality

---

## System Architecture

```
┌─────────────────────┐
│   GPS Module #1     │
│   (MTK3339)         │
└──────┬──────────────┘
       │ PPS + NMEA
       ↓
┌─────────────────────┐         PTP Sync/Follow_Up
│  RP2040W #1         │─────────────────────────→
│  (Grandmaster)      │                           │
│  - GPS Discipline   │                           │
│  - PTP Protocol     │                           ↓
│  - Output: 100 PPS  │                  ┌─────────────────────┐
└──────┬──────────────┘                  │  RP2040W #2         │
       │ 100 PPS                          │  (Slave)            │
       │                                  │  - PTP Client       │
       │                                  │  - Clock Discipline │
       │                                  │  - Output: 100 PPS  │
       │                                  └──────┬──────────────┘
       │                                         │ 100 PPS
       │                                         │
┌──────┴──────────────┐                         │
│   GPS Module #2     │                         │
│   (Reference)       │                         │
└──────┬──────────────┘                         │
       │ 1 PPS (reference)                      │
       │                                         │
       └────────────→ RP2040W #3 ←──────────────┘
                     (Monitor)
                     - Triple Phase Comparison:
                       * GPS 1 PPS (ground truth)
                       * GM 100 PPS (disciplined)
                       * Slave 100 PPS (synced)
                     - Logging & Analysis
```

---

## Component Breakdown

### 1. PTP Grandmaster (RP2040W #1)

**Purpose**: Maintain GPS-disciplined time and distribute it via PTP protocol

**Hardware Connections**:
- GPS PPS → GPIO (PIO-capable pin, e.g., GPIO 2)
- GPS TX → UART RX (GPIO 1)
- GPS RX → UART TX (GPIO 0)
- 100 PPS Output → GPIO (PIO-capable pin, e.g., GPIO 3)
- WiFi: Built-in CYW43 module

**Software Components**:

#### A. GPS PPS Capture (PIO-based)
- **Reuse**: Adapt existing `pps_capture.pio` from freq-counter project
- **Function**: Capture GPS PPS rising edge with hardware timestamp
- **Implementation**:
  - PIO state machine waits for rising edge
  - On edge detection, reads hardware timer (64-bit microsecond counter)
  - Pushes timestamp to PIO FIFO
  - Main code reads timestamp from FIFO

#### B. GPS NMEA Parser
- **Purpose**: Validate GPS fix status and extract UTC time
- **Implementation**:
  - UART interrupt/DMA to receive NMEA sentences
  - Parse $GPRMC or $GPGGA sentences
  - Extract: fix quality, number of satellites, UTC time
  - Only trust PPS when fix quality indicates valid 3D fix

#### C. Time Discipline Algorithm
- **Purpose**: Maintain a corrected timebase disciplined to GPS
- **Core Data Structure**:
```c
typedef struct {
    uint64_t nanoseconds;      // Current disciplined time
    int64_t phase_error;       // Last measured phase error (ns)
    int32_t frequency_offset;  // Frequency correction (ppb)
    uint32_t last_pps_time;    // Hardware timer value at last PPS
    bool locked;               // Are we locked to GPS?
} disciplined_clock_t;
```

- **Algorithm** (runs on Core 1):
  1. When GPS PPS occurs at hardware time `t_pps`:
     - Expected time for PPS: `last_pps_time + 1,000,000` (1 second in μs)
     - Phase error: `e = t_pps - expected`
  2. PI Controller:
     - Proportional: `P = Kp * e`
     - Integral: `I = I + (Ki * e)`
     - Correction: `freq_offset = P + I`
  3. Update clock:
     - Between PPS events, increment `nanoseconds` based on elapsed time
     - Apply frequency correction: `ns += (elapsed_us * 1000) * (1 + freq_offset/1e9)`

- **Constants**: Start with Kp ≈ 0.1, Ki ≈ 0.001 (tune empirically)

#### E. High-Frequency Output Generation (100 PPS)
- **Purpose**: Generate precise 100 Hz signal for phase comparison monitoring
- **Why 100 PPS**: 
  - 10ms period gives 100 samples/second for phase error measurement
  - Higher resolution than 1 PPS for characterizing jitter and dynamics
  - Still low enough for easy PIO processing
  - Alternative: 1000 PPS (1ms period) for even finer resolution

- **Implementation Options**:

**Option 1: PIO-based generation (recommended)**
```c
// PIO program to generate square wave at threshold crossings
// Grandmaster Core 1 signals PIO when to toggle
.program pps_output
.wrap_target
    wait 1 gpio N      ; Wait for signal from main code
    set pins, 1        ; Rising edge
    set pins, 0 [31]   ; Falling edge with delay for pulse width
.wrap
```

- Core 1 discipline loop checks disciplined clock:
```c
// In Core 1 main loop
uint64_t last_pulse_ns = 0;
const uint64_t PULSE_PERIOD_NS = 10000000;  // 10ms = 100 Hz

while (1) {
    uint64_t current_ns = disciplined_clock.nanoseconds;
    
    if ((current_ns - last_pulse_ns) >= PULSE_PERIOD_NS) {
        // Trigger PIO to generate pulse
        gpio_put(TRIGGER_PIN, 1);
        gpio_put(TRIGGER_PIN, 0);  // Brief trigger pulse
        last_pulse_ns += PULSE_PERIOD_NS;  // Use += not = to avoid drift
    }
    
    // Continue with discipline algorithm
    tight_loop_contents();
}
```

**Option 2: Hardware Alarm (simpler but less precise)**
```c
// Configure alarm to fire every 10ms based on disciplined clock
// This has ~1-2μs jitter from interrupt latency
void alarm_callback(uint alarm_num) {
    gpio_put(OUTPUT_PPS_PIN, 1);
    busy_wait_us(10);  // Short pulse
    gpio_put(OUTPUT_PPS_PIN, 0);
    
    // Reschedule based on disciplined clock
    uint64_t next_pulse = disciplined_clock.nanoseconds + 10000000;
    hardware_alarm_set_target(alarm_num, (uint32_t)(next_pulse / 1000));
}
```

**Option 3: Direct GPIO toggle (simplest, Core 1)**
```c
// Core 1 directly toggles GPIO when threshold crossed
// No PIO needed, but Core 1 must be dedicated to timing
uint64_t next_edge_ns = 0;

while (1) {
    if (disciplined_clock.nanoseconds >= next_edge_ns) {
        gpio_put(OUTPUT_PPS_PIN, 1);
        busy_wait_us(1);  // ~1μs pulse width
        gpio_put(OUTPUT_PPS_PIN, 0);
        next_edge_ns += 10000000;  // 10ms
    }
    // Minimal other processing to avoid missing edges
}
```

- **Recommended**: Option 1 (PIO) or Option 3 (direct GPIO) depending on whether you need Core 1 for other tasks
- **Pulse characteristics**: 
  - Very short pulse (1-10μs) is sufficient for edge detection
  - Rising edge is what matters for phase measurement
  - PIO can generate precise pulse widths if needed

#### F. Network Configuration
- **Static IP recommended**: Avoids DHCP variability
- **lwIP configuration**:
  - Use `pico_cyw43_arch_lwip_threadsafe_background` for better control
  - Consider disabling Nagle's algorithm for lower latency
  - MTU: Default 1500 is fine

#### G. PTP Protocol Stack
- **Runs on**: Core 0
- **Transport**: UDP over WiFi (lwIP stack)
- **Messages to Implement**:

**1. Announce Message** (every 1 second)
```c
typedef struct {
    // PTP Common Header
    uint8_t messageType;           // 0x0B for Announce
    uint8_t versionPTP;            // 0x02
    uint16_t messageLength;        // 64 bytes
    uint8_t domainNumber;          // 0 (default domain)
    uint8_t reserved1;
    uint16_t flagField;            // 0x0008 (PTP timescale)
    int64_t correctionField;       // 0
    uint32_t reserved2;
    uint8_t sourcePortIdentity[10]; // Clock ID + port number
    uint16_t sequenceId;
    uint8_t controlField;          // 0x05
    int8_t logMessageInterval;     // 0 (1 second = 2^0)
    
    // Announce-specific fields
    int16_t currentUtcOffset;      // 37 (as of 2024)
    uint8_t reserved3;
    uint8_t grandmasterPriority1;  // 128
    uint8_t grandmasterClockQuality[4];
    uint8_t grandmasterPriority2;  // 128
    uint8_t grandmasterIdentity[8];
    uint16_t stepsRemoved;         // 0
    uint8_t timeSource;            // 0x20 (GPS)
} __attribute__((packed)) ptp_announce_msg_t;
```
- **Purpose**: Declare this clock as grandmaster
- **Destination**: Multicast 224.0.1.129:319 or unicast to slave

**2. Sync Message** (every 1 second, or faster)
```c
typedef struct {
    // PTP Common Header (same structure as Announce)
    uint8_t messageType;           // 0x00 for Sync
    // ... other common fields ...
    
    // Sync-specific fields
    uint64_t originTimestamp_seconds;
    uint32_t originTimestamp_nanoseconds;
} __attribute__((packed)) ptp_sync_msg_t;
```
- **Implementation**:
  1. Prepare packet with estimated timestamp
  2. Call `udp_send()` from lwIP
  3. Immediately after send, read disciplined clock value (t1)
  4. Store t1 for corresponding Follow_Up message

**3. Follow_Up Message** (immediately after each Sync)
```c
typedef struct {
    // PTP Common Header
    uint8_t messageType;           // 0x08 for Follow_Up
    // ... other common fields ...
    
    // Follow_Up-specific fields
    uint64_t preciseOriginTimestamp_seconds;
    uint32_t preciseOriginTimestamp_nanoseconds;
} __attribute__((packed)) ptp_followup_msg_t;
```
- **Purpose**: Convey the precise t1 timestamp from the Sync message
- **Implementation**: Send immediately after capturing post-Sync timestamp

#### F. Core Assignment
- **Static IP recommended**: Avoids DHCP variability
- **lwIP configuration**:
  - Use `pico_cyw43_arch_lwip_threadsafe_background` for better control
  - Consider disabling Nagle's algorithm for lower latency
  - MTU: Default 1500 is fine

#### H. Core Assignment
- **Core 0**:
  - Main application loop
  - WiFi/network stack
  - PTP protocol transmission
  - User interface / logging
  
- **Core 1**:
  - Time discipline loop (runs continuously)
  - PIO management
  - Disciplined clock updates
  - 100 PPS output generation (threshold detection and GPIO/PIO triggering)
  - Timestamp generation for PTP

- **Inter-core Communication**:
  - Use multicore FIFO or mutex-protected shared structure
  - Core 0 requests timestamp from Core 1 for PTP messages
  - Core 1 publishes locked status to Core 0

---

### 2. PTP Slave (RP2040W #2)

**Purpose**: Synchronize local clock to grandmaster via PTP

**Hardware Connections**:
- WiFi: Built-in CYW43 module
- 100 PPS Output → GPIO (PIO-capable pin, e.g., GPIO 3) for monitoring comparison

**Software Components**:

#### A. PTP Client Protocol
- **Runs on**: Core 0
- **Messages to Receive**:

**1. Announce Message Handler**
- Parse received Announce
- Verify this is the best master (compare clock quality, priority)
- Update grandmaster identity

**2. Sync Message Handler**
- Record arrival time t2 (use hardware timer immediately upon receipt)
- Store sequence number
- Wait for corresponding Follow_Up

**3. Follow_Up Message Handler**
- Extract precise origin timestamp t1
- Match with stored Sync via sequence number
- Calculate offset: `offset = t2 - t1` (simplified, assumes zero network delay)
- More accurate: Implement delay measurement (see optional Delay_Req/Delay_Resp below)

#### B. Clock Discipline (Slave)
- **Purpose**: Adjust local clock based on measured offset from grandmaster
- **Core Data Structure**:
```c
typedef struct {
    uint64_t nanoseconds;        // Current slave time
    int64_t offset_from_master;  // Last measured offset (ns)
    int32_t frequency_offset;    // Frequency correction (ppb)
    bool synced;                 // Are we synced to master?
} slave_clock_t;
```

- **Algorithm** (runs on Core 1):
  1. Receive offset measurement from Core 0: `offset = t2 - t1`
  2. PI Controller (similar to grandmaster):
     - `P = Kp * offset`
     - `I = I + (Ki * offset)`
     - `freq_offset = P + I`
  3. Update clock:
     - `ns += (elapsed_us * 1000) * (1 + freq_offset/1e9)`
  4. If offset < threshold (e.g., 1ms), set `synced = true`

#### C. Disciplined 100 PPS Output
- **Purpose**: Generate 100 Hz output signal aligned to slave's disciplined time for phase comparison
- **Why essential**: This is how we measure synchronization quality - the phase relationship between this output and the grandmaster's 100 PPS output reveals the actual time offset

- **Implementation** (identical approach to grandmaster):

**Method 1: Direct GPIO Toggle on Core 1** (recommended for simplicity)
```c
// In slave Core 1 discipline loop
uint64_t next_pulse_ns = 0;
const uint64_t PULSE_PERIOD_NS = 10000000;  // 10ms = 100 Hz

while (1) {
    // Update slave clock based on PTP corrections
    update_slave_clock();
    
    // Generate output pulse when threshold crossed
    if (slave_clock.nanoseconds >= next_pulse_ns) {
        gpio_put(OUTPUT_100PPS_PIN, 1);
        busy_wait_us(1);  // Brief pulse
        gpio_put(OUTPUT_100PPS_PIN, 0);
        next_pulse_ns += PULSE_PERIOD_NS;
    }
    
    tight_loop_contents();
}
```

**Method 2: PIO-based** (if Core 1 needs to do other work)
- Use same PIO approach as grandmaster
- Core 1 signals PIO when nanosecond counter crosses 10ms boundaries
- PIO generates precise pulse

**Key concept**: 
- If slave is perfectly synchronized, its nanosecond counter equals grandmaster's counter
- Therefore, both cross the 10ms threshold at the same physical instant
- Rising edges of 100 PPS outputs should be perfectly aligned
- Any phase offset directly measures synchronization error

#### D. Core Assignment
- **Core 0**: Network stack, PTP client, offset calculation
- **Core 1**: Clock discipline, 100 PPS output generation

---

### 3. Reference Monitor (RP2040W #3)

**Purpose**: Measure synchronization quality by comparing slave PPS against independent GPS reference

**Hardware Connections**:
- GPS PPS (1 Hz) → GPIO (PIO-capable, e.g., GPIO 2)
- GPS TX → UART RX
- Grandmaster 100 PPS → GPIO (PIO-capable, e.g., GPIO 3)
- Slave 100 PPS → GPIO (PIO-capable, e.g., GPIO 4)
- Optional: USB serial for logging, or SD card

**Software Components**:

#### A. Triple Signal Capture
- **Signals to measure**:
  1. GPS 1 PPS (reference ground truth)
  2. Grandmaster 100 PPS (GPS-disciplined output)
  3. Slave 100 PPS (PTP-synchronized output)

- **Implementation approach**:

**Strategy**: Since we have three signals but they're at different frequencies, we use the GPS 1 PPS as the common reference point and measure the other two relative to it.

**PIO State Machine Allocation**:
- **PIO SM 0**: Capture GPS 1 PPS rising edges → timestamp in FIFO
- **PIO SM 1**: Capture GM 100 PPS rising edges → timestamp in FIFO
- **PIO SM 2**: Capture Slave 100 PPS rising edges → timestamp in FIFO

**Measurement Method 1: Nearest Edge Comparison**
```c
// For each GPS PPS edge (once per second)
uint64_t gps_timestamp = read_gps_pps_timestamp();

// Find the nearest GM 100 PPS edge (within ±5ms window)
uint64_t gm_timestamp = find_nearest_edge(gm_edges_buffer, gps_timestamp, 5000);
int64_t gm_offset = (int64_t)gm_timestamp - (int64_t)gps_timestamp;

// Find the nearest Slave 100 PPS edge (within ±5ms window)
uint64_t slave_timestamp = find_nearest_edge(slave_edges_buffer, gps_timestamp, 5000);
int64_t slave_offset = (int64_t)slave_timestamp - (int64_t)gps_timestamp;

// Key metrics:
// 1. GM offset from GPS: Should be ~0 (GM is GPS-disciplined)
// 2. Slave offset from GPS: Measures synchronization error
// 3. Slave-GM delta: (slave_offset - gm_offset) = direct PTP sync error
```

**Measurement Method 2: Continuous 100 PPS Comparison**
```c
// Alternative: Compare every 100 PPS edge pair directly
// Don't wait for GPS 1 PPS - compare GM vs Slave continuously

while (1) {
    if (gm_edge_available() && slave_edge_available()) {
        uint64_t gm_time = read_gm_edge();
        uint64_t slave_time = read_slave_edge();
        
        // Phase error: how far apart are they?
        int64_t phase_error = (int64_t)slave_time - (int64_t)gm_time;
        
        // This gives 100 measurements/second instead of 1/second
        log_measurement(phase_error);
    }
}
```

- **Recommendation**: Use Method 2 for rich data (100 samples/sec), periodically validate against GPS 1 PPS to ensure neither clock has drifted significantly from absolute time

#### B. Phase Error Measurement & Analysis
- **Purpose**: Quantify synchronization quality with high temporal resolution

- **Primary Metric: Slave-to-Grandmaster Phase Error**
  - This is the key measurement: how synchronized is the slave to the GM?
  - Measured 100 times per second (at each 100 PPS edge)
  - Formula: `phase_error = slave_edge_time - gm_edge_time`
  - Positive = slave is ahead, Negative = slave is behind

- **Secondary Validation: Grandmaster-to-GPS Offset**
  - Verifies GM discipline quality
  - Measured once per second (at GPS 1 PPS edge)
  - Should be near zero (±10μs) if GPS discipline is working
  - If this drifts, GM has a problem - don't trust slave measurements

- **Metrics to Calculate & Track**:
  ```c
  typedef struct {
      int64_t current_error_ns;      // Latest phase error
      int64_t mean_error_ns;         // Running average
      int64_t std_dev_ns;            // Standard deviation (jitter)
      int64_t min_error_ns;          // Best case
      int64_t max_error_ns;          // Worst case
      uint32_t samples_collected;    // Number of measurements
      bool slave_is_synced;          // Is |error| < threshold?
  } phase_stats_t;
  ```

- **Real-time Analysis**:
  1. **Convergence tracking**: Measure time from power-on until `|phase_error| < 1ms` sustained
  2. **Jitter characterization**: Calculate standard deviation over 1-second windows
  3. **Outlier detection**: Flag measurements > 3σ from mean
  4. **Lock indication**: Assert "locked" when error stays within ±1ms for >5 seconds

- **Frequency Domain Analysis** (optional, advanced):
  - Calculate Allan Deviation to characterize stability over different timescales
  - FFT of phase error to identify periodic disturbances (e.g., 60Hz noise, WiFi beacon intervals)

#### C. Data Logging
- **Options**:
  1. USB serial output (printf to host computer) - easiest for development
  2. SD card logging (requires SPI SD card module) - for long-term unattended testing
  3. Network logging (send to syslog server or HTTP endpoint) - for remote monitoring

- **Logging Strategy**:
  - **High-rate data** (100 Hz): Slave-GM phase error, running statistics
  - **Low-rate data** (1 Hz): GM-GPS offset, lock status, summary statistics
  - **Event logging**: Loss of lock, outliers, errors

- **Format Example** (CSV for easy plotting):
```csv
# High-rate log (100 samples/second)
timestamp_ms, gm_edge_us, slave_edge_us, phase_error_us, instantaneous_jitter_us

# Low-rate log (1 sample/second) 
timestamp_ms, gm_gps_offset_us, mean_phase_error_us, std_dev_us, min_us, max_us, locked

# Example rows:
1000, 1000125, 1002340, 2215, 45
1010, 1010130, 1012355, 2225, 10
...
2000, 15, 2218, 127, -450, 5230, true
```

- **Data Rate Considerations**:
  - 100 Hz × ~50 bytes/line = 5 KB/sec = 18 MB/hour
  - For USB serial: No problem, can stream continuously
  - For SD card: Batch writes every 1 second to reduce wear
  - For network: Send summary statistics only (1 Hz) to reduce bandwidth

- **Visualization**:
  - Stream to PC, plot real-time with Python/matplotlib
  - Post-process for publication-quality graphs
  - Key plots: Phase error vs. time, histogram, Allan deviation

#### D. GPS Validation
- Parse GPS NMEA to ensure reference has valid fix
- Don't trust measurements if GPS loses lock

---

## Implementation Phases

### Phase 1: Foundation (Grandmaster Core)
1. **GPS PPS capture**: Port `pps_capture.pio`, verify timestamps
2. **NMEA parsing**: Implement basic parser, confirm fix status
3. **Discipline algorithm**: Implement PI controller, verify lock to GPS
4. **Core 1 setup**: Move discipline to Core 1, verify stability
5. **1 PPS output**: Generate simple 1 Hz output (LED or GPIO) locked to disciplined clock

**Success Criteria**: LED blinks at 1 Hz locked to GPS PPS, visually confirmed synchronization

### Phase 2a: High-Frequency Output (Grandmaster)
1. **Upgrade to 100 PPS**: Modify output generation to produce 100 Hz instead of 1 Hz
2. **Verify on oscilloscope**: Confirm clean 100 Hz square wave
3. **Validate against GPS**: Use scope to compare 100 PPS edges against GPS 1 PPS - every 100th edge should align

**Success Criteria**: Stable 100 PPS output, confirmed relationship to GPS PPS (100:1 ratio)

**Rationale**: Validate higher-frequency output generation before introducing network complexity. This ensures any issues are in the output logic, not PTP/network.

### Phase 2b: PTP Protocol (Grandmaster)
1. **Network setup**: Configure WiFi, lwIP, static IP
2. **PTP packet structures**: Define C structures for Announce, Sync, Follow_Up
3. **Message transmission**: Implement Announce (1/sec), Sync (1/sec), Follow_Up
4. **Timestamp integration**: Insert disciplined clock value into PTP messages

**Success Criteria**: Wireshark on network shows valid PTP packets

### Phase 3a: PTP Slave Protocol
1. **Network setup**: Configure WiFi, receive PTP multicast/unicast
2. **Message parsing**: Implement handlers for Announce, Sync, Follow_Up
3. **Offset calculation**: Extract t1, capture t2, calculate offset
4. **Clock discipline**: Implement PI controller to adjust local clock
5. **1 PPS output**: Generate simple 1 Hz output to verify discipline

**Success Criteria**: Slave achieves lock to grandmaster, 1 PPS output indicates synchronization

### Phase 3b: Slave High-Frequency Output
1. **Upgrade to 100 PPS**: Modify slave output to produce 100 Hz signal
2. **Verify on oscilloscope**: Confirm clean 100 Hz square wave from slave
3. **Visual comparison**: Use dual-channel scope to compare Slave 100 PPS vs GM 100 PPS

**Success Criteria**: Slave 100 PPS output visible, phase relationship to GM measurable on scope

**Note**: At this point you can already see synchronization quality with just an oscilloscope - the phase offset between the two 100 PPS signals is your sync error!

**Optional checkpoint**: You have a working PTP system at this point. Phase 4 adds automated measurement and logging, but isn't required to demonstrate the concept.

### Phase 4: Monitoring & Validation
1. **Triple signal capture**: Implement three PIO state machines for GPS 1 PPS, GM 100 PPS, Slave 100 PPS
2. **Error measurement**: Calculate phase error between Slave and GM at 100 Hz rate
3. **GM validation**: Verify GM-GPS offset remains small (validates discipline quality)
4. **Statistics**: Calculate mean, std dev, min/max of phase errors
5. **Logging**: Output high-rate (100 Hz) phase measurements via USB serial or SD card
6. **Analysis**: Collect data, plot phase error over time, characterize performance

**Success Criteria**: Continuous 100 Hz measurement of synchronization error, documented accuracy with statistical analysis

### Phase 5: Optimization (Optional)
1. **Delay measurement**: Implement Delay_Req/Delay_Resp for asymmetric delay compensation
2. **Faster sync rate**: Increase from 1Hz to 2-4Hz
3. **Better timestamping**: Minimize latency between lwIP and timestamp capture
4. **Tuning**: Optimize PI controller gains for faster lock and lower jitter

---

## Key Design Decisions & Rationale

### Why 100 PPS Output (not just 1 PPS)?
**Problem with 1 PPS**: Only one sample per second makes it difficult to characterize jitter, see transient behavior, or get quick feedback during development/tuning.

**100 PPS advantages**:
- 100 measurements/second provides rich temporal data
- 10ms sample spacing reveals dynamics of WiFi jitter and PI controller response
- Statistical confidence achieved much faster (seconds vs. minutes)
- Still low enough rate for easy PIO/CPU processing

**1000 PPS alternative**: Could use 1 kHz (1ms period) for even finer resolution, but:
- 10× more processing overhead
- Diminishing returns given WiFi jitter is typically 1-10ms
- 100 Hz is the sweet spot for this application

### Why Two-Step Sync (Sync + Follow_Up)?
Hardware timestamping (inserting timestamp into packet as it leaves PHY) isn't available on RP2040W. Two-step allows software timestamping: send packet, immediately read clock, report in Follow_Up.

### Why PI Controller for Discipline?
Classic control theory approach: Proportional term responds to current error, Integral term eliminates steady-state error (frequency offset). Simple, proven, easy to tune.

### Why Separate Monitor Device?
Using an independent GPS reference eliminates circular validation. You're measuring against an external standard rather than trusting the PTP exchange alone.

### Why Core 0/Core 1 Split?
Isolating timing-critical operations (discipline, timestamping) from network stack (which has unpredictable latency) on separate cores dramatically improves determinism.

### Simplified vs. Full PTP?
Full IEEE 1588 includes Best Master Clock Algorithm (BMCA), transparent clocks, peer-to-peer delay, etc. For a controlled experiment with one grandmaster and one slave, these add complexity without benefit.

---

## Expected Performance

### Realistic Expectations
- **Grandmaster to GPS**: Should maintain lock within ±10μs given clean GPS PPS
- **Slave to Grandmaster**: 
  - Best case: ~100μs - 1ms (limited by WiFi jitter, lack of hardware timestamping)
  - Typical: 1-10ms
  - Occasional excursions to 50-100ms during network congestion

### Benefits of 100 PPS Monitoring
- **High temporal resolution**: 100 measurements/second reveals dynamics invisible at 1 Hz
- **Jitter characterization**: Can measure instantaneous jitter between consecutive samples (10ms apart)
- **Transient detection**: See PI controller response to disturbances in real-time
- **Faster validation**: Achieve statistical confidence in minutes rather than hours
- **Phase relationship**: Direct measurement of synchronization error with 10ms granularity

### Limiting Factors
1. **WiFi variability**: CYW43 firmware introduces unpredictable latency (1-10ms)
2. **Software timestamping**: Gap between packet transmission and timestamp capture
3. **lwIP stack delays**: Protocol processing adds jitter
4. **Lack of asymmetric delay compensation**: Network path may have different delays in each direction

### Comparison to Real PTP
Industrial PTP switches with hardware timestamping achieve sub-microsecond accuracy. This project demonstrates the concepts but won't reach that level due to platform limitations.

---

## Testing & Validation

### Unit Tests
- GPS PPS capture accuracy (compare multiple captures of same PPS)
- NMEA parser (feed known sentences, verify parsing)
- PTP packet construction (verify byte layout matches IEEE 1588)
- PI controller stability (feed known errors, verify convergence)

### Integration Tests
- GPS discipline lock time (cold start to locked)
- PTP slave lock time (power on to synced)
- Network stress test (add background traffic, measure impact)

### Performance Characterization
- Phase error histogram (distribution of sync errors)
- Allan deviation (frequency stability metric)
- Time to convergence after power cycle
- Impact of WiFi channel congestion

---

## Development Tools

### Required Hardware
- 3x RP2040W boards (Raspberry Pi Pico W)
- 2x GPS modules (Adafruit Ultimate GPS or similar with PPS)
- Breadboards, jumper wires
- USB cables for power and programming
- Optional: Oscilloscope to visualize PPS signals

### Software Tools
- Pico SDK (latest version)
- CMake build system
- GDB for debugging (via SWD)
- Wireshark for network analysis (PTP dissector built-in)
- Python/matplotlib for data analysis and plotting

### Debugging Strategies
- Use USB serial for logging (printf redirected to USB CDC)
- Blink LEDs to indicate state (GPS lock, PTP sync, errors)
- Log phase errors and controller state to identify instability
- Network capture to verify PTP message correctness

---

## Code Structure Recommendation

```
ptp-gps-project/
├── CMakeLists.txt
├── common/
│   ├── ptp_protocol.h         # PTP packet structures
│   ├── ptp_protocol.c         # Packet construction/parsing
│   └── discipline.h/.c        # PI controller, common discipline code
├── grandmaster/
│   ├── CMakeLists.txt
│   ├── main.c                 # Core 0: main loop, network
│   ├── gps.h/.c               # GPS PPS capture, NMEA parsing
│   ├── discipline_core.c      # Core 1: GPS discipline
│   ├── output_100pps.c        # 100 PPS generation
│   └── ptp_grandmaster.h/.c   # PTP Announce/Sync/Follow_Up
├── slave/
│   ├── CMakeLists.txt
│   ├── main.c                 # Core 0: main loop, network
│   ├── ptp_client.h/.c        # PTP message reception/parsing
│   ├── discipline_core.c      # Core 1: clock discipline
│   └── output_100pps.c        # Generate disciplined 100 PPS
├── monitor/
│   ├── CMakeLists.txt
│   ├── main.c
│   ├── triple_capture.c       # PIO-based triple signal measurement
│   ├── statistics.c           # Phase error analysis
│   └── logging.c              # Data collection and output
└── pio/
    ├── pps_capture.pio        # Adapted from freq-counter (1 PPS input)
    ├── edge_capture.pio       # Generic edge timestamp (for 100 PPS inputs)
    └── pps_output.pio         # Generate PPS/100PPS signals (optional)
```

---

## Reference Documents

### IEEE 1588-2008 Specification
- Sections 13.3-13.5: Message formats
- Section 11: Best Master Clock Algorithm (can skip for simplified implementation)
- Annex C: Default profiles

### MTK3339 GPS Module
- Datasheet: PPS characteristics, timing accuracy
- NMEA sentence reference: $GPRMC, $GPGGA format

### RP2040 Datasheet
- Chapter 3: PIO
- Chapter 4: Timers
- Section 2.3.1: Multicore

### Existing Reference Implementations
- linuxptp: For full-featured PTP stack (reference for packet formats)
- ptpd: Simpler, older implementation (easier to understand)

---

## Next Steps

When you're ready to start implementation, work through the phases sequentially. Each phase builds on the previous and has clear success criteria.

Begin with Phase 1 (GPS discipline on grandmaster) since:
1. It's self-contained and testable without network
2. It reuses your existing freq-counter PIO code
3. It establishes the foundation for accurate timekeeping

Use Claude Code to help with:
- PIO program adaptation and debugging
- PTP packet structure definitions and byte ordering (network byte order!)
- PI controller implementation and tuning
- Network stack configuration and debugging

This design gives you a complete, working system that demonstrates real PTP concepts while being achievable on microcontroller hardware. The measured performance data will clearly show both the capabilities and limitations of software-based timing on WiFi-connected microcontrollers.

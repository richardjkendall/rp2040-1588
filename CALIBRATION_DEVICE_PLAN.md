# PTP Measurement Calibration Device - Implementation Plan

## Purpose

Create a **GPS-disciplined test signal generator** to validate the measurement device's accuracy before using it to measure slave improvements.

## Concept

```
┌─────────────────────────────────────────────────────────────────┐
│                     CALIBRATION TEST SETUP                      │
└─────────────────────────────────────────────────────────────────┘

GM (GPS-disciplined)
  └── GPIO 15 (1PPS) ──┬──> Measurement Device Input A
                       │
                       └──> Calibration Device Input (GPIO 2)
                                      │
                     Calibration Device disciplines to GM 1PPS
                                      │
                       Generates offset 1PPS (GPIO 15)
                                      │
                                      └──> Measurement Device Input B

Measurement Device reports: offset = Input B - Input A
Expected: offset = TEST_OFFSET_NS ± measurement accuracy
```

## Design

### **Hardware**
- **Board**: Pico W (reuses measurement device hardware)
- **Input**: GPIO 2 (GM 1PPS reference)
- **Output**: GPIO 15 (offset 1PPS to measurement device)
- **No other hardware needed** (no GPS, no Ethernet, no WiFi on this device)

### **Firmware Architecture**

```
Core 0 (Main):
  ├─ PIO Counter (free-running at 83.33 MHz)
  ├─ PIO PPS Capture (detects edges on GPIO 2)
  ├─ Discipline Loop (disciplines to GM 1PPS)
  │    └─ Characterizes crystal against GM 1PPS
  │    └─ Updates scale_factor
  └─ PPS Scheduler (generates offset 1PPS on GPIO 15)
       └─ Uses TEST_OFFSET_NS compile-time constant
```

### **No Network, No Cores**
- Single core operation (simpler than GM)
- No WiFi, no Ethernet, no PTP protocol
- Just: count GM pulses → discipline crystal → generate offset pulse

---

## Code Reuse Strategy

### **Files to Reuse** (via `#include`, NO modifications)

1. **`grandmaster/discipline_v3.h/c`** - GPS discipline logic
   - Reuse discipline loop structure
   - Reuse Kalman filter
   - Reuse scale factor calculation
   - **Modification**: Replace GPS NMEA with simple PPS counting

2. **`slave/pps_scheduler.h/c`** - PPS generation
   - Reuse PIO-based 1PPS scheduler
   - No changes needed

3. **`pio/counter_simple.pio`** - Free-running counter
   - No changes needed

4. **`pio/pps_capture.pio`** - PPS edge capture
   - Already exists in `grandmaster/` for GPS PPS
   - No changes needed

### **New Files to Create**

1. **`calibration_test/main_calibration_device.c`**
   - Main entry point
   - Initialize PIO counter
   - Initialize PPS capture on GPIO 2
   - Run discipline loop (simplified, no GPS NMEA)
   - Generate offset 1PPS

2. **`calibration_test/pps_discipline.c/h`**
   - Simplified discipline (no GPS time calculation)
   - Just count PPS edges, characterize crystal
   - Reuse Kalman filter structure from `discipline_v3.c`

3. **`calibration_test/CMakeLists.txt`**
   - Build configuration

---

## Implementation Details

### **Discipline Algorithm** (Simplified from GPS GM)

```c
// On each GM 1PPS edge:
void on_pps_edge(uint32_t counter_value) {
    uint32_t elapsed_ticks = prev_counter - counter_value;  // Counter counts down

    // GM 1PPS should be exactly 1 second apart
    uint64_t expected_ticks = EXPECTED_TICKS_PER_SECOND;  // 83333333

    // Calculate crystal error
    int64_t error_ticks = elapsed_ticks - expected_ticks;
    crystal_error_ns = error_ticks * 12;  // 12ns per tick

    // Update scale factor (how to convert crystal time to true time)
    double measured_scale = (double)expected_ticks / (double)elapsed_ticks;
    scale_factor = scale_factor * 0.98 + measured_scale * 0.02;  // EMA filter

    // Update PPS time (just count seconds)
    pps_time_seconds++;

    prev_counter = counter_value;
}
```

### **Time Function** (Replaces `get_gps_time_ns()`)

```c
uint64_t get_calibration_time_ns(void) {
    // Current second boundary
    uint64_t base_time_ns = pps_time_seconds * 1000000000ULL;

    // Microseconds since last PPS edge
    uint64_t us_since_pps = time_us_64() - last_pps_time_us;

    // Scale to nanoseconds using characterized crystal
    uint64_t ns_since_pps = (uint64_t)((double)us_since_pps * 1000.0 * scale_factor);

    return base_time_ns + ns_since_pps;
}
```

### **Offset PPS Generation**

```c
// Schedule 1PPS with programmed offset
void schedule_offset_pps(void) {
    uint64_t target_time_ns = get_calibration_time_ns();

    // Find next second boundary + offset
    uint64_t ns_in_second = target_time_ns % 1000000000ULL;
    uint64_t next_second_ns = target_time_ns - ns_in_second + 1000000000ULL;
    uint64_t target_with_offset = next_second_ns + TEST_OFFSET_NS;

    // Schedule PPS at target time
    pps_scheduler_schedule_at(&scheduler, target_with_offset);
}
```

---

## Configuration

### **Compile-Time Offsets**

User edits `calibration_test/main_calibration_device.c`:

```c
// Test offset in nanoseconds
// Change this value, reflash, and measure
#define TEST_OFFSET_NS 100000  // 100µs

// Uncomment one of these presets or use custom value above:
//#define TEST_OFFSET_NS 0         // Zero reference
//#define TEST_OFFSET_NS 10000     // 10µs
//#define TEST_OFFSET_NS 50000     // 50µs
//#define TEST_OFFSET_NS 100000    // 100µs
//#define TEST_OFFSET_NS 500000    // 500µs
//#define TEST_OFFSET_NS 1000000   // 1ms
//#define TEST_OFFSET_NS -100000   // -100µs (test polarity)
```

### **Pin Configuration**

```c
#define GM_PPS_INPUT_PIN   2   // Input from GM 1PPS
#define TEST_PPS_OUTPUT_PIN 15 // Output to measurement device
```

---

## Testing Protocol

### **Phase 1: Zero Offset Validation**
1. Set `TEST_OFFSET_NS = 0`
2. Build and flash calibration device
3. Connect GM GPIO 15 → Measurement Input A AND Calibration Input (GPIO 2)
4. Connect Calibration GPIO 15 → Measurement Input B
5. Measurement should read: **0ns ± 500ns**

**Success Criteria**: Reading stable at 0 ± 500ns for 30 minutes

### **Phase 2: Positive Offset Sweep**
Test offsets: 10µs, 50µs, 100µs, 500µs, 1ms

For each:
1. Edit `TEST_OFFSET_NS`
2. Rebuild and reflash
3. Wait 5 minutes for discipline to lock
4. Measure for 10 minutes
5. Verify reading matches `TEST_OFFSET_NS ± 1µs`

**Success Criteria**: All offsets read correctly within ±1µs

### **Phase 3: Negative Offset Test**
1. Set `TEST_OFFSET_NS = -100000` (-100µs)
2. Measure
3. Verify reading is **-100µs ± 1µs**

**Success Criteria**: Negative polarity works correctly

### **Phase 4: Long-Term Stability**
1. Set `TEST_OFFSET_NS = 100000`
2. Measure for 2 hours
3. Download CSV
4. Check Allan deviation at τ=1s, 10s, 100s, 1000s

**Success Criteria**:
- Allan dev (1s) < 500ns
- Allan dev (100s) < 100ns
- No drift > 1µs over 2 hours

### **Phase 5: Outlier Rejection Validation**
1. Set `TEST_OFFSET_NS = 100000`
2. During measurement, briefly disconnect GM 1PPS input
   - This will cause calibration device to lose lock
   - Should see outlier measurements
3. Reconnect GM 1PPS
4. Verify:
   - Outlier count increases during disconnect
   - Measurements stabilize after reconnect
   - Outliers are correctly rejected from statistics

**Success Criteria**: Outlier detection working correctly

---

## Expected Results

### **If Measurement Device is Accurate**:
- All test offsets read correctly within ±1µs
- Long-term stability < 100ns RMS
- Allan deviation shows measurement is limited by counter resolution (~12ns)
- **→ Proceed with slave improvement measurements confidently**

### **If Measurement Device Has Issues**:
- Offsets read incorrectly or drift over time
- Large Allan deviation at short τ (indicating noise)
- Outliers not detected or wrongly filtered
- **→ Fix measurement device before measuring slave improvements**

---

## Build Instructions

```bash
cd pico-gps-1588
mkdir -p build/calibration_test
cd build/calibration_test
cmake ../..
make calibration_device
```

Output: `calibration_device.uf2`

---

## Code Changes Required

### **New Files** (to create):
- `calibration_test/main_calibration_device.c`
- `calibration_test/pps_discipline.c`
- `calibration_test/pps_discipline.h`
- `calibration_test/CMakeLists.txt`

### **Existing Files** (NO modifications):
- Reuse via `#include` and relative paths
- `grandmaster/discipline_v3.h` (reference for Kalman structure)
- `slave/pps_scheduler.c/h` (used as-is)
- `pio/*.pio` (used as-is)
- `common/ptp_protocol.h` (NOT needed - no PTP)

### **Potential Shared Code Extraction** (Optional future):
If we want cleaner reuse, could create:
- `common/kalman_filter.c/h` (extracted from discipline_v3.c)
- `common/pps_capture.c/h` (wrapper around PIO)

**But for now**: Just include existing files and adapt as needed in new code.

---

## Advantages of This Approach

1. **Validates measurement chain** before trusting slave improvements
2. **Independent test** - doesn't modify GM/slave/measurement code
3. **Reuses proven code** - discipline logic already works in GM
4. **Simple to use** - just change a #define and reflash
5. **Scientific rigor** - establishes known reference before measurements
6. **Debugging tool** - can verify measurement device anytime

---

## Timeline

**Implementation**: 2-3 hours
- Set up CMakeLists.txt
- Adapt discipline code for PPS-only (no GPS NMEA)
- Integrate PPS scheduler
- Add offset configuration

**Testing**: 4-6 hours
- Phase 1: 30 minutes (zero offset)
- Phase 2: 1 hour (offset sweep)
- Phase 3: 15 minutes (negative offset)
- Phase 4: 2 hours (long-term stability)
- Phase 5: 30 minutes (outlier rejection)

**Total**: 1 day to build and validate measurement system

---

## Next Steps

1. **Approve this plan**
2. **Create calibration_test/ folder and code**
3. **Build and flash to spare Pico W**
4. **Run test protocol**
5. **If measurement device validates** → proceed with slave improvements
6. **If measurement device has issues** → fix measurement device first

Ready to proceed with implementation?

# PTP GPS 1588 Project - Claude Reference

**Last Updated:** 2025-12-26

## Project Objective

Build a high-precision PTP (IEEE 1588-2008) time synchronization system using Raspberry Pi Pico microcontrollers with:
- **GPS-disciplined Grandmaster** (primary time source)
- **PTP Slave** synchronized to Grandmaster via Ethernet
- **Independent measurement device** to verify synchronization performance

**Target Performance:** Sub-microsecond synchronization accuracy

## Repository Structure

```
pico-gps-1588/
├── grandmaster/          # GPS-disciplined PTP Grandmaster
├── slave/                # PTP Slave with PI servo
├── measurement/          # Independent 1PPS phase measurement device
├── common/               # Shared PTP protocol code
├── docs/                 # Architecture and design documentation
├── tests/                # Test data and logs
├── tools/                # Telemetry server and analysis tools
├── pico-sdk/             # Pico SDK (submodule)
└── build/                # Build artifacts (CMake)
```

## Build System

### SDK Location
```
PICO_SDK_PATH=/Users/rjk/Code/pico-gps-1588/pico-sdk
```

### Build Directory
```
/Users/rjk/Code/pico-gps-1588/build
```

### Network Backend Configuration

The project supports two network backends:

**Ethernet (W5500) - Default for GM and Slave:**
```bash
cd /Users/rjk/Code/pico-gps-1588/build
cmake -DPICO_SDK_PATH=/Users/rjk/Code/pico-gps-1588/pico-sdk -DUSE_ETHERNET=ON ..
```

**WiFi (Pico W) - For measurement device:**
```bash
cd /Users/rjk/Code/pico-gps-1588/build
cmake -DPICO_SDK_PATH=/Users/rjk/Code/pico-gps-1588/pico-sdk -DUSE_ETHERNET=OFF ..
```

## Build Targets

### Grandmaster (GPS + PTP Grandmaster)
**Hardware:** Raspberry Pi Pico with W5500 Ethernet module + GPS module
**Target:** `w5500_dual_core`
**Build:**
```bash
cd /Users/rjk/Code/pico-gps-1588/build
cmake -DUSE_ETHERNET=ON ..
make w5500_dual_core -j8
```
**Firmware:** `grandmaster/w5500_dual_core.uf2`

### Slave (PTP Slave with PI Servo)
**Hardware:** Raspberry Pi Pico with W5500 Ethernet module
**Target:** `w5500_ptp_slave`
**Build:**
```bash
cd /Users/rjk/Code/pico-gps-1588/build
cmake -DUSE_ETHERNET=ON ..
make w5500_ptp_slave -j8
```
**Firmware:** `slave/w5500_ptp_slave.uf2`

### Measurement Device (1PPS Phase Meter)
**Hardware:** Raspberry Pi Pico W (WiFi for telemetry)
**Target:** `measurement_device_wifi`
**Build:**
```bash
cd /Users/rjk/Code/pico-gps-1588/build
cmake -DUSE_ETHERNET=OFF ..
make measurement_device_wifi -j8
```
**Firmware:** `measurement/measurement_device_wifi.uf2`

## Key Implementation Details

### Current Servo Architecture (PI Servo - Implemented Dec 2024)

**Algorithm:** Industry-standard PI (Proportional-Integral) control
**Location:** `slave/ptp_discipline.c`

**Parameters:**
- `Kp = 0.7` (proportional gain)
- `Ki = 0.0012` (integral gain - adjusted for 1Hz updates)
- Anti-windup: ±1 second
- Max frequency adjustment: ±100 µs per update

**Key Functions:**
- `pi_servo_update()` - lines 452-484
- `ptp_discipline_get_scale_factor()` - lines 375-384 (for 1PPS scheduler)
- `update_ptp_clock()` - lines 194-218 (simplified, no scale_factor)

### Asymmetry Correction

**Value:** +181 µs (empirically calibrated)
**Location:** `slave/ptp_discipline.c:52-55`
**Documentation:** `docs/ASYMMETRY_CORRECTION.md`

**Breakdown:**
- Hardware latency asymmetry: 37.5 µs
- Path asymmetry: 96.1 µs
- Systematic bias: 85.1 µs (unexplained)

### 1PPS Pulse Generation

**Pulse Width:** 96 ns (8 PIO cycles @ 12ns/cycle)
**PIO Programs:**
- `slave/pio/scheduled_1pps.pio`
- `grandmaster/pio/scheduled_1pps.pio`

**Scheduler:** `slave/pps_scheduler.c`
- Calculates ticks to next second boundary
- Applies scale_factor correction for crystal drift
- Resolution: 12 ns per tick

### Phase Measurement Resolution

**Hardware:** PIO-based edge-to-edge timing
**Resolution:** 36 ns (3 PIO cycles @ 12ns/cycle)
**Location:** `measurement/pio/gm_to_slave_counter.pio`, `measurement/pio/slave_to_gm_counter.pio`

**Measurement Thresholds:**
- `MIN_VALID_OFFSET_NS = 50.0` ns (captures down to 50 ns)
- `INVALID_THRESHOLD_NS = 150.0` ns (detects PIO glitch when pin already HIGH)
- Measurements below threshold reported as 50 ns (conservative floor)

## Current Performance (Dec 2024)

**Achieved:**
- ✓ Mean offset: ~58 µs (converging)
- ✓ Best case: < 50 ns (below measurement threshold!)
- ✓ Rejection rate: < 1% (only when offset < 50 ns)
- ✓ Stable lock: 100% uptime
- ✓ 1PPS generation: Continuous

**Remaining Jitter:** 20-120 µs peak-to-peak
- **Cause:** Clock runs at nominal rate between PTP syncs
- **Mechanism:** Crystal drifts ~34 µs/second → discrete correction → sawtooth pattern

## Known Issues & Optimization Opportunities

### 1. Clock Update Rate Jitter
**Issue:** Clock runs at 1:1 ratio between PTP sync messages (~1 second)
**Location:** `slave/ptp_discipline.c:216`
```c
uint64_t ptp_elapsed_ns = elapsed_us * 1000;  // No frequency correction!
state.ptp_clock_ns += ptp_elapsed_ns;
```

**Proposed Fix:** Apply scale_factor continuously (hybrid approach)
**Documentation:** `docs/SCALE_FACTOR_HYBRID_APPROACH.md`
**Impact:** Would reduce jitter from ~20-120 µs to < 1 µs

### 2. Clock Update Quantization
**Issue:** Clock updates in 1 µs steps (`time_us_64()`)
**Impact:** Adds ~1 µs quantization noise
**Potential Fix:** Use hardware timer for sub-microsecond resolution

## Important Documentation

- `docs/ASYMMETRY_CORRECTION.md` - Network asymmetry calibration
- `docs/PTP_CONVERGENCE_PLAN.md` - PI servo implementation plan
- `docs/SCALE_FACTOR_HYBRID_APPROACH.md` - Proposed optimization
- `BLOG_POST_OUTLINE.md` - Project overview and motivation

## Telemetry System

### Server
**Location:** `tools/telemetry-server/`
**Language:** Go
**Build:**
```bash
cd /Users/rjk/Code/pico-gps-1588/tools/telemetry-server
go build
```
**Run:**
```bash
./telemetry-server -tcp 5001
```

### Measurement Device WiFi Config
**SSID:** bhop
**Server:** 10.10.143.151:5001

### Data Format
- Sequence number
- Timestamp (µs)
- Phase offset (ns)
- GM→Slave counter (ns)
- Slave→GM counter (ns)
- Scale factor
- Crystal error (ns)

## Common Development Workflow

### Rebuild Slave After Code Changes
```bash
cd /Users/rjk/Code/pico-gps-1588/build
cmake -DUSE_ETHERNET=ON ..
make w5500_ptp_slave -j8
# Flash: slave/w5500_ptp_slave.uf2
```

### Rebuild Grandmaster After Code Changes
```bash
cd /Users/rjk/Code/pico-gps-1588/build
cmake -DUSE_ETHERNET=ON ..
make w5500_dual_core -j8
# Flash: grandmaster/w5500_dual_core.uf2
```

### Rebuild Measurement Device After Code Changes
```bash
cd /Users/rjk/Code/pico-gps-1588/build
cmake -DUSE_ETHERNET=OFF ..
make measurement_device_wifi -j8
cmake -DUSE_ETHERNET=ON ..  # Restore to Ethernet mode
# Flash: measurement/measurement_device_wifi.uf2
```

### View Logs
```bash
# Slave/GM logs (screen session)
tail -f /Users/rjk/Code/pico-gps-1588/tests/screenlog.0

# Telemetry server logs
# Check running bash sessions for telemetry-server output
```

## Git Status & Branches

**Current Branch:** `feature/relative-phase-measurement`

**Main Branch:** Not specified (check with user)

**Pending Changes (as of 2025-12-26):**
- Modified: `libraries/WIZnet-PICO-LWIP-C` (submodule)
- Modified: `measurement/CMakeLists.txt`
- Untracked: `measurement/main_measurement_device_wifi.c`
- Untracked: `measurement/ring_buffer.*`, `measurement/telemetry.*`
- Untracked: `tools/` directory

## Next Steps (Dec 2024)

1. **Apply scale_factor to clock updates** (reduce jitter to < 1 µs)
2. **Long-term stability testing** (multi-day run)
3. **Temperature sensitivity testing** (verify crystal compensation)
4. **Network switch testing** (recalibrate asymmetry correction)

## Contact & Context

This file is maintained for Claude Code sessions to provide context across conversations.
When starting a new session, reference this file to understand the project state.

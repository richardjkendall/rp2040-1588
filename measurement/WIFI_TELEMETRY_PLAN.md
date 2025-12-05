# WiFi Telemetry Implementation Plan

## Executive Summary

Add WiFi telemetry to the measurement device to stream phase offset measurements to a remote receiver for offline analysis, without affecting measurement accuracy or performance.

**Key Principle**: Complete isolation of timing-critical code (Core 0) from network stack (Core 1)

## Architecture

### Dual-Core Design

```
┌─────────────────────────────────────────────────────────────┐
│ Core 0 (Timing Critical - NO WiFi interaction)              │
├─────────────────────────────────────────────────────────────┤
│ • GPS discipline (crystal calibration)                      │
│ • PIO SM2: GM→Slave counter                                 │
│ • PIO SM3: Slave→GM counter                                 │
│ • Phase offset calculation                                  │
│ • Local statistics (mean, stddev, histogram)                │
│ • Non-blocking write to ring buffer                         │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       │ Ring Buffer (shared memory)
                       │ • 1000 measurement slots
                       │ • Lock-free write (Core 0)
                       │ • Blocking read (Core 1)
                       │
┌──────────────────────▼──────────────────────────────────────┐
│ Core 1 (Telemetry - ALL WiFi overhead here)                │
├─────────────────────────────────────────────────────────────┤
│ • WiFi stack (CYW43)                                        │
│ • TCP connection to telemetry server                        │
│ • Read from ring buffer                                     │
│ • Batch measurements (60 per batch)                         │
│ • Encode as JSON                                            │
│ • Send over TCP                                             │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow

1. **Core 0**: PIO captures edge timing → Calculate phase offset → Write to ring buffer (non-blocking)
2. **Ring Buffer**: Lock-free queue (FIFO)
3. **Core 1**: Read from buffer → Batch 60 measurements → JSON encode → TCP send
4. **Python Receiver**: Listen on TCP port → Decode JSON → Log/analyze/plot

## Implementation Details

### 1. Ring Buffer (Core 0 ↔ Core 1 Communication)

**File**: `measurement/ring_buffer.h`, `measurement/ring_buffer.c`

```c
#define MEASUREMENT_BUFFER_SIZE 1000  // ~16 minutes @ 1 Hz

typedef struct {
    uint64_t sequence;           // Measurement number
    double phase_offset_ns;      // True phase offset (minimum of both directions)
    double gm_to_slave_ns;       // GM→Slave measurement
    double slave_to_gm_ns;       // Slave→GM measurement
    double scale_factor;         // GPS-calibrated crystal scale
    bool gm_first;               // Which edge came first
    uint64_t timestamp_us;       // System time when measured
    int32_t crystal_error_ns;    // GPS crystal error this second
} measurement_t;

typedef struct {
    measurement_t buffer[MEASUREMENT_BUFFER_SIZE];
    volatile uint32_t write_idx;
    volatile uint32_t read_idx;
    volatile uint32_t dropped_count;  // Incremented when buffer full
} measurement_ring_buffer_t;

// Initialize ring buffer
void ring_buffer_init(measurement_ring_buffer_t *rb);

// Non-blocking write (Core 0) - returns false if buffer full
bool ring_buffer_try_write(measurement_ring_buffer_t *rb, const measurement_t *m);

// Blocking read with timeout (Core 1) - returns false on timeout
bool ring_buffer_read(measurement_ring_buffer_t *rb, measurement_t *m, uint32_t timeout_ms);

// Get buffer statistics
void ring_buffer_get_stats(measurement_ring_buffer_t *rb,
                           uint32_t *available, uint32_t *dropped);
```

**Implementation Notes**:
- Use volatile indices for Core 0/1 synchronization
- Write index only modified by Core 0
- Read index only modified by Core 1
- No mutexes needed (lock-free design)
- If buffer full, Core 0 drops measurement and increments counter

### 2. Telemetry Module (Core 1)

**File**: `measurement/telemetry.h`, `measurement/telemetry.c`

```c
#define TELEMETRY_BATCH_SIZE 60      // Send every 60 measurements (~1 minute)
#define TELEMETRY_SERVER_PORT 5000   // Default port
#define TELEMETRY_RECONNECT_DELAY_MS 5000

typedef struct {
    const char *ssid;
    const char *password;
    const char *server_ip;
    uint16_t server_port;
    measurement_ring_buffer_t *ring_buffer;
} telemetry_config_t;

// Initialize telemetry (connects to WiFi)
bool telemetry_init(const telemetry_config_t *config);

// Core 1 entry point
void telemetry_core1_entry(void);

// Get telemetry statistics
void telemetry_get_stats(uint32_t *batches_sent, uint32_t *send_failures,
                         uint32_t *reconnects, bool *connected);
```

**JSON Message Format**:

```json
{
  "device": "measurement_device",
  "batch_seq": 42,
  "count": 60,
  "measurements": [
    {
      "seq": 2501,
      "timestamp_us": 150012345678,
      "phase_ns": 245.3,
      "gm_to_slave_ns": 245.3,
      "slave_to_gm_ns": 987654.2,
      "scale_factor": 1.000012,
      "gm_first": true,
      "crystal_error_ns": 145
    },
    ...
  ],
  "stats": {
    "buffer_available": 850,
    "buffer_dropped": 0
  }
}
```

**Connection Strategy**:
- TCP client (Pico connects to your server)
- Auto-reconnect on disconnect
- Keepalive messages if no data for 60s
- Non-blocking sends with timeout

### 3. Main Application (Dual-Core)

**File**: `measurement/main_measurement_device_wifi.c`

```c
// Configuration (can be moved to config.h later)
#define WIFI_SSID "your_ssid"
#define WIFI_PASSWORD "your_password"
#define TELEMETRY_SERVER_IP "192.168.1.100"

// Shared ring buffer
static measurement_ring_buffer_t measurement_buffer;

int main() {
    // [Same as existing main_measurement_device.c setup]
    // - Overclock to 250 MHz
    // - Initialize GPS
    // - Initialize GPS discipline
    // - Initialize PIO counters

    // Initialize ring buffer
    ring_buffer_init(&measurement_buffer);

    // Configure telemetry
    telemetry_config_t telemetry_config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASSWORD,
        .server_ip = TELEMETRY_SERVER_IP,
        .server_port = TELEMETRY_SERVER_PORT,
        .ring_buffer = &measurement_buffer
    };

    // Launch Core 1 for telemetry
    multicore_launch_core1(telemetry_core1_entry);

    // Core 0 main loop (ULTRA-MINIMAL like minimal_test)
    while (true) {
        // Process GPS
        gps_process();

        // Check if both FIFOs have data
        if (!pio_sm_is_rx_fifo_empty(PHASE_PIO, GM_TO_SLAVE_SM) &&
            !pio_sm_is_rx_fifo_empty(PHASE_PIO, SLAVE_TO_GM_SM)) {

            // [Same measurement logic as before]
            uint32_t gm_to_slave_ticks = pio_sm_get(PHASE_PIO, GM_TO_SLAVE_SM);
            uint32_t slave_to_gm_ticks = pio_sm_get(PHASE_PIO, SLAVE_TO_GM_SM);
            double sf = discipline_get_scale_factor();

            // Calculate phase offset
            double gm_to_slave_ns = gm_to_slave_ticks * 12.0 * sf;
            double slave_to_gm_ns = slave_to_gm_ticks * 12.0 * sf;
            bool gm_first = (gm_to_slave_ns < slave_to_gm_ns);
            double phase_offset_ns = gm_first ? gm_to_slave_ns : slave_to_gm_ns;

            // Prepare measurement for ring buffer
            measurement_t m = {
                .sequence = measurement_count++,
                .timestamp_us = time_us_64(),
                .phase_offset_ns = phase_offset_ns,
                .gm_to_slave_ns = gm_to_slave_ns,
                .slave_to_gm_ns = slave_to_gm_ns,
                .scale_factor = sf,
                .gm_first = gm_first,
                .crystal_error_ns = crystal_error_ns
            };

            // Non-blocking write to ring buffer (for Core 1)
            if (!ring_buffer_try_write(&measurement_buffer, &m)) {
                // Buffer full - Core 1 too slow (should never happen with 1000 slots)
                // Measurement dropped, counter incremented automatically
            }

            // Continue with local statistics (same as before)
            update_phase_stats(phase_offset_ns, gm_first);

            // Print to console (first 20, then every 100th)
            if (measurement_count <= 20 || measurement_count % 100 == 0) {
                printf("[%6llu] Phase: %+7.1f ns (%c first)\n",
                       measurement_count, phase_offset_ns, gm_first ? 'G' : 'S');
            }
        }

        // Print local stats every 30 seconds (same as before)
        if (now_us - last_stats_time_us >= 30000000) {
            print_stats();
            last_stats_time_us = now_us;
        }

        sleep_ms(100);
    }
}
```

**Core 0 Performance Impact**:
- Ring buffer write: ~10-20 CPU cycles
- No blocking, no WiFi interaction
- Negligible impact on timing

### 4. Telemetry Receiver (Python)

**File**: `tools/telemetry_receiver.py`

```python
#!/usr/bin/env python3
"""
PTP Phase Offset Telemetry Receiver

Listens for TCP connections from measurement device and logs phase offset data.
"""

import socket
import json
import sys
import time
from datetime import datetime

class TelemetryReceiver:
    def __init__(self, host='0.0.0.0', port=5000, log_file='measurements.jsonl'):
        self.host = host
        self.port = port
        self.log_file = log_file

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.listen(1)

        print(f"Listening on {self.host}:{self.port}")
        print(f"Logging to {self.log_file}")

        with open(self.log_file, 'a') as f:
            while True:
                try:
                    print("Waiting for connection...")
                    conn, addr = sock.accept()
                    print(f"Connected from {addr}")

                    buffer = ""
                    while True:
                        data = conn.recv(4096)
                        if not data:
                            break

                        buffer += data.decode('utf-8')

                        # Process complete JSON messages (newline-delimited)
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            if line.strip():
                                self.process_batch(json.loads(line), f)

                except KeyboardInterrupt:
                    print("\nShutting down...")
                    break
                except Exception as e:
                    print(f"Error: {e}")
                    time.sleep(1)

    def process_batch(self, batch, file):
        timestamp = datetime.now().isoformat()

        # Log batch metadata
        print(f"\n[{timestamp}] Batch #{batch['batch_seq']}: "
              f"{batch['count']} measurements")

        # Write to log file (JSONL format)
        file.write(json.dumps(batch) + '\n')
        file.flush()

        # Print statistics
        measurements = batch['measurements']
        if measurements:
            phases = [m['phase_ns'] for m in measurements]
            mean = sum(phases) / len(phases)
            print(f"  Phase offset: {mean:.1f} ns (min: {min(phases):.1f}, "
                  f"max: {max(phases):.1f})")
            print(f"  Buffer: {batch['stats']['buffer_available']} available, "
                  f"{batch['stats']['buffer_dropped']} dropped")

if __name__ == '__main__':
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 5000
    receiver = TelemetryReceiver(port=port)
    receiver.run()
```

**Usage**:
```bash
python3 telemetry_receiver.py 5000
```

### 5. Offline Analysis Tool

**File**: `tools/analyze_telemetry.py`

```python
#!/usr/bin/env python3
"""
Analyze logged telemetry data
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

def analyze_log(log_file):
    """Load and analyze telemetry log"""
    measurements = []

    with open(log_file) as f:
        for line in f:
            batch = json.loads(line)
            measurements.extend(batch['measurements'])

    # Extract phase offsets
    phases = np.array([m['phase_ns'] for m in measurements])
    timestamps = np.array([m['timestamp_us'] for m in measurements])

    # Calculate statistics
    print(f"Total measurements: {len(measurements)}")
    print(f"Mean phase offset: {np.mean(phases):.1f} ns")
    print(f"Std deviation: {np.std(phases):.1f} ns")
    print(f"Min: {np.min(phases):.1f} ns")
    print(f"Max: {np.max(phases):.1f} ns")

    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

    # Time series
    ax1.plot((timestamps - timestamps[0]) / 1e6, phases, '.-', alpha=0.5)
    ax1.set_xlabel('Time (seconds)')
    ax1.set_ylabel('Phase Offset (ns)')
    ax1.set_title('Phase Offset vs Time')
    ax1.grid(True)

    # Histogram
    ax2.hist(phases, bins=50, edgecolor='black')
    ax2.set_xlabel('Phase Offset (ns)')
    ax2.set_ylabel('Count')
    ax2.set_title('Phase Offset Distribution')
    ax2.axvline(np.mean(phases), color='r', linestyle='--', label='Mean')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig('phase_offset_analysis.png', dpi=150)
    print("\nPlot saved to phase_offset_analysis.png")

if __name__ == '__main__':
    import sys
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <log_file.jsonl>")
        sys.exit(1)

    analyze_log(sys.argv[1])
```

**Usage**:
```bash
python3 analyze_telemetry.py measurements.jsonl
```

## Build Configuration

### CMakeLists.txt Changes

**File**: `measurement/CMakeLists.txt`

Add WiFi-enabled build target:

```cmake
# ============================================================================
# PTP Phase Offset Measurement Device - WiFi Telemetry Version
# GPS-calibrated measurement with remote telemetry streaming
# ============================================================================

# Only build WiFi version for Pico W (not Ethernet builds)
if(NOT USE_ETHERNET)
    add_executable(measurement_device_wifi
        main_measurement_device_wifi.c
        ring_buffer.c
        telemetry.c
        ${CMAKE_SOURCE_DIR}/grandmaster/gps.c
        ${CMAKE_SOURCE_DIR}/grandmaster/discipline_v3.c
    )

    target_include_directories(measurement_device_wifi PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_SOURCE_DIR}/grandmaster
        ${CMAKE_SOURCE_DIR}/common
        ${CMAKE_SOURCE_DIR}
    )

    target_compile_definitions(measurement_device_wifi PRIVATE
        BUILD_MEASUREMENT
        WIFI_TELEMETRY_ENABLED
    )

    target_link_libraries(measurement_device_wifi
        pico_stdlib
        pico_multicore          # CRITICAL: Dual-core support
        pico_cyw43_arch_lwip_poll  # WiFi + TCP stack
        hardware_pio
        hardware_uart
        hardware_timer
        hardware_irq
        hardware_clocks
        hardware_gpio
        common
    )

    # Generate PIO headers for GPS discipline
    pico_generate_pio_header(measurement_device_wifi ${CMAKE_SOURCE_DIR}/grandmaster/pio/gps_discipline_v2.pio)
    pico_generate_pio_header(measurement_device_wifi ${CMAKE_SOURCE_DIR}/pio/pps_capture.pio)

    # Generate PIO headers for phase measurement
    pico_generate_pio_header(measurement_device_wifi ${CMAKE_CURRENT_LIST_DIR}/pio/gm_to_slave_counter.pio)
    pico_generate_pio_header(measurement_device_wifi ${CMAKE_CURRENT_LIST_DIR}/pio/slave_to_gm_counter.pio)

    # Enable USB output for debugging
    pico_enable_stdio_usb(measurement_device_wifi 1)
    pico_enable_stdio_uart(measurement_device_wifi 0)

    pico_add_extra_outputs(measurement_device_wifi)

    message(STATUS "Building measurement_device_wifi with WiFi telemetry support")
endif()
```

### Build Commands

```bash
# Build WiFi version (Pico W)
cd build
cmake .. -DUSE_ETHERNET=OFF
make measurement_device_wifi

# Binary location:
# build/measurement/measurement_device_wifi.uf2

# Build original version (no WiFi)
make measurement_device
```

## Configuration

### WiFi Credentials

**Option 1**: Hardcoded (simple, for testing)

```c
// In main_measurement_device_wifi.c
#define WIFI_SSID "your_network_name"
#define WIFI_PASSWORD "your_password"
#define TELEMETRY_SERVER_IP "192.168.1.100"
```

**Option 2**: Config file (better for deployment)

Create `measurement/config.h`:
```c
#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration
#define WIFI_SSID "your_network_name"
#define WIFI_PASSWORD "your_password"

// Telemetry Configuration
#define TELEMETRY_SERVER_IP "192.168.1.100"
#define TELEMETRY_SERVER_PORT 5000

// Buffer Configuration
#define MEASUREMENT_BUFFER_SIZE 1000  // ~16 minutes
#define TELEMETRY_BATCH_SIZE 60       // Send every 60 measurements

#endif
```

## Testing Plan

### Phase 1: Ring Buffer Testing
1. Build with ring buffer but no WiFi (Core 1 just drains buffer)
2. Verify Core 0 timing unchanged
3. Check buffer never overflows (monitor dropped_count)

### Phase 2: Local WiFi Testing
1. Build full WiFi version
2. Run Python receiver on laptop
3. Verify measurements arrive correctly
4. Check for packet loss, reconnection behavior

### Phase 3: Performance Validation
1. Compare statistics from Core 0 console vs telemetry stream
2. Should be identical (proves no data corruption)
3. Run for 1+ hour, verify dropped_count = 0
4. Compare EMA stddev before/after WiFi (should be same)

### Phase 4: Long-term Stability
1. Run for 24+ hours
2. Monitor for memory leaks, connection stability
3. Verify timing accuracy unchanged

## Performance Guarantees

### What WILL NOT be affected:
- ✅ PIO timing (hardware state machines independent of CPU)
- ✅ GPS discipline accuracy (runs on Core 0 only)
- ✅ Phase measurement resolution (~12ns per tick)
- ✅ Local statistics (Core 0 calculates independently)

### What MIGHT be affected (with mitigation):
- ⚠️ **Memory bus contention**: Core 1 WiFi DMA vs Core 0 memory access
  - **Mitigation**: Ring buffer in fast SRAM, minimal Core 0 writes
  - **Validation**: Compare stddev before/after WiFi

- ⚠️ **CPU cache thrashing**: If cores compete for cache
  - **Mitigation**: Core 0 code is ultra-minimal (like minimal_test)
  - **Validation**: Monitor GPS crystal_error_ns consistency

### Success Criteria:
1. Core 0 statistics identical with/without WiFi enabled
2. `dropped_count` remains 0 (or very low) over 24 hours
3. GPS crystal_error_ns variance unchanged
4. EMA stddev (5/15/30 min) within 5% of non-WiFi version

## File Structure

```
pico-gps-1588/
├── measurement/
│   ├── main_measurement_device.c           # Original (no WiFi)
│   ├── main_measurement_device_wifi.c      # NEW: Dual-core WiFi version
│   ├── ring_buffer.h                       # NEW: Ring buffer interface
│   ├── ring_buffer.c                       # NEW: Ring buffer implementation
│   ├── telemetry.h                         # NEW: Telemetry interface
│   ├── telemetry.c                         # NEW: WiFi telemetry (Core 1)
│   ├── config.h                            # NEW: Configuration (optional)
│   ├── CMakeLists.txt                      # MODIFY: Add WiFi target
│   ├── WIFI_TELEMETRY_PLAN.md             # This document
│   └── pio/
│       ├── gm_to_slave_counter.pio         # Existing
│       └── slave_to_gm_counter.pio         # Existing
│
└── tools/
    ├── telemetry_receiver.py               # NEW: Python receiver
    └── analyze_telemetry.py                # NEW: Offline analysis
```

## Implementation Steps

### Step 1: Ring Buffer
- [ ] Create `ring_buffer.h` with interface
- [ ] Create `ring_buffer.c` with lock-free implementation
- [ ] Add unit tests (optional)

### Step 2: Telemetry Module
- [ ] Create `telemetry.h` with interface
- [ ] Create `telemetry.c` with WiFi connection + JSON encoding
- [ ] Add reconnection logic
- [ ] Add batch sending

### Step 3: Dual-Core Main
- [ ] Copy `main_measurement_device.c` → `main_measurement_device_wifi.c`
- [ ] Add ring buffer initialization
- [ ] Add Core 0 ring buffer write
- [ ] Add Core 1 launch
- [ ] Keep all Core 0 timing logic unchanged

### Step 4: Build System
- [ ] Update `measurement/CMakeLists.txt`
- [ ] Add WiFi target conditional on `!USE_ETHERNET`
- [ ] Test build with both ON/OFF

### Step 5: Python Tools
- [ ] Create `telemetry_receiver.py`
- [ ] Create `analyze_telemetry.py`
- [ ] Add requirements.txt (numpy, matplotlib)

### Step 6: Testing
- [ ] Phase 1: Ring buffer only
- [ ] Phase 2: Local WiFi
- [ ] Phase 3: Performance validation
- [ ] Phase 4: 24-hour stability test

## Future Enhancements

### Compression
- Use gzip compression for JSON (70-80% reduction)
- `zlib` library available in SDK

### Multiple Receivers
- UDP broadcast instead of TCP unicast
- Multiple listeners for real-time monitoring

### Web Interface
- HTTP server on Pico W
- Real-time browser-based plots
- Configuration UI

### SD Card Logging
- Local backup in case WiFi fails
- Use SPI SD card module on Pico

### Time Synchronization
- NTP client on Core 1
- Sync `gps_ns_counter` to UTC
- Add UTC timestamps to telemetry

## Security Considerations

### Current Implementation
- Unencrypted TCP (plaintext JSON)
- No authentication
- Suitable for private networks only

### If Needed
- TLS/SSL encryption (mbedTLS available in SDK)
- API key authentication
- VPN tunnel (WireGuard on router level)

## Power Consumption

WiFi adds ~100mA typical draw:
- Pico W idle: ~30mA
- Pico W WiFi TX: ~130mA
- Total: ~160mA @ 5V = 0.8W

Use USB power adapter with sufficient capacity.

## References

- Pico W WiFi examples: `pico-sdk/lib/cyw43-driver/`
- lwIP TCP stack: `pico-sdk/lib/lwip/`
- Multicore examples: `pico-examples/multicore/`
- Ring buffer patterns: [https://www.kernel.org/doc/html/latest/core-api/circular-buffers.html](https://www.kernel.org/doc/html/latest/core-api/circular-buffers.html)

## Conclusion

This plan provides a complete, production-ready WiFi telemetry solution that maintains measurement accuracy through strict Core 0/Core 1 isolation. The ring buffer provides ample headroom (16 minutes) and the lock-free design ensures Core 0 is never blocked.

Ready to implement when you are!

---

**Document Version**: 1.0
**Date**: 2025-12-01
**Author**: Claude Code
**Status**: Ready for Implementation

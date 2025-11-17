# Phase 2b Implementation Plan: PTP Protocol over WiFi

## Overview

Add PTP (Precision Time Protocol / IEEE 1588) grandmaster functionality to distribute GPS-disciplined time over WiFi to other devices.

---

## Architecture

### Core 0 (Non-Timing-Critical)
- **WiFi Management**: Initialize CYW43, connect to network, maintain connection
- **Network Stack**: lwIP with static IP configuration
- **PTP Protocol**: Generate and transmit PTP messages
- **Timestamping**: Read disciplined clock from Core 1 via shared variables
- **Serial Output**: Status and debug information

### Core 1 (Timing-Critical) - NO CHANGES
- **GPS Discipline**: Continue GPS PPS capture and clock discipline
- **100 PPS Output**: Continue generating 100 Hz output on GPIO 3
- **Shared State**: Publish disciplined clock value for Core 0 to read
- **Zero I/O**: No printf, no network, pure timing

---

## Network Configuration

### Static IP (Recommended)
```
IP Address:   192.168.1.100
Netmask:      255.255.255.0
Gateway:      192.168.1.1
DNS:          8.8.8.8 (optional)
```

**Why Static?**
- Avoids DHCP delays (can take 5-10 seconds)
- Deterministic network behavior
- Simpler debugging

**User Configuration Required:**
- Update IP address to match your network
- Ensure IP doesn't conflict with other devices
- Router should be at 192.168.1.1 (or update gateway)

### PTP Network Configuration
```
Protocol:     UDP
Multicast:    224.0.1.129:319 (PTP event messages)
              224.0.1.129:320 (PTP general messages)
Domain:       0 (default PTP domain)
```

### WiFi Credentials
```c
#define WIFI_SSID "YourNetworkName"
#define WIFI_PASSWORD "YourPassword"
```

---

## PTP Protocol Implementation

### Message Types

#### 1. Announce Message
**Purpose**: Declare this device as the PTP grandmaster

**Frequency**: 1 per second

**Content**:
- Clock identity (MAC address based)
- Clock quality (GPS = excellent)
- Priority (default = 128)
- Time source = GPS (0x20)

**Destination**: 224.0.1.129:320 (general messages)

#### 2. Sync Message
**Purpose**: Mark a time synchronization event

**Frequency**: 1 per second

**Content**:
- Approximate origin timestamp (will be corrected in Follow_Up)
- Sequence number (increments each sync)
- Two-step flag (set - we use Follow_Up)

**Destination**: 224.0.1.129:319 (event messages)

**Timing**: Send immediately, then capture precise timestamp

#### 3. Follow_Up Message
**Purpose**: Provide precise timestamp for corresponding Sync message

**Frequency**: 1 per second (immediately after each Sync)

**Content**:
- Precise origin timestamp (read from disciplined clock)
- Sequence number (matches corresponding Sync)

**Destination**: 224.0.1.129:320 (general messages)

**Critical**: Must read disciplined clock immediately after Sync transmission

### Message Flow Sequence

```
Core 0 Main Loop (Every 1 second):

1. Send Announce message
   - Tells network "I am the grandmaster"
   - Includes clock quality information

2. Prepare Sync message
   - Set sequence number
   - Populate with estimated timestamp

3. Send Sync message via UDP
   - Record exact time of send: t1_before = time_us_64()

4. Read precise timestamp from Core 1
   - t1_precise = core1_stats.disciplined_clock_ns

5. Send Follow_Up message
   - Contains t1_precise
   - References Sync sequence number

Total packets: 3 per second
Network load: ~300 bytes/sec (very light)
```

---

## Shared State Extension

### Add to core1_stats_t

```c
typedef struct {
    // Existing fields
    volatile uint32_t pps_count;
    volatile int64_t phase_error_ns;
    volatile int32_t freq_offset_ppb;
    volatile bool locked;
    volatile bool first_pps_received;
    volatile bool discipline_running;
    volatile uint32_t lock_event_count;
    volatile uint32_t unlock_event_count;

    // NEW: For PTP timestamping
    volatile uint64_t disciplined_time_ns;  // Current disciplined clock value
    volatile uint64_t last_update_us;       // When was it last updated?
} core1_stats_t;
```

### Core 1 Updates Continuously

```c
// In Core 1 main loop, after discipline_update_time():
core1_stats.disciplined_time_ns = disciplined_clock.nanoseconds;
core1_stats.last_update_us = time_us_64();
```

### Core 0 Reads for PTP Timestamps

```c
// When sending Sync message:
uint64_t timestamp_ns = core1_stats.disciplined_time_ns;
uint64_t timestamp_s = timestamp_ns / 1000000000ULL;
uint32_t timestamp_ns_frac = timestamp_ns % 1000000000ULL;

// Put in PTP message:
sync_msg.originTimestamp.seconds = timestamp_s;
sync_msg.originTimestamp.nanoseconds = timestamp_ns_frac;
```

---

## PTP Packet Structures (IEEE 1588-2008)

### Common Header (All Messages)

```c
typedef struct {
    uint8_t  messageType;           // 0x0B=Announce, 0x00=Sync, 0x08=Follow_Up
    uint8_t  versionPTP;            // 0x02 (PTPv2)
    uint16_t messageLength;         // Total message length in bytes
    uint8_t  domainNumber;          // 0 (default domain)
    uint8_t  reserved1;
    uint16_t flagField;             // Bit flags (two-step, etc.)
    int64_t  correctionField;       // Nanoseconds (usually 0 for grandmaster)
    uint32_t reserved2;
    uint8_t  sourcePortIdentity[10]; // Clock ID (8 bytes) + port (2 bytes)
    uint16_t sequenceId;            // Increments per message type
    uint8_t  controlField;          // Message-specific control
    int8_t   logMessageInterval;    // Log2 of interval (0 = 1 second)
} __attribute__((packed)) ptp_header_t;
```

### Announce Message Body

```c
typedef struct {
    ptp_header_t header;

    // Announce-specific fields
    uint8_t  originTimestamp[10];   // Not used for Announce
    int16_t  currentUtcOffset;      // 37 as of 2024
    uint8_t  reserved;
    uint8_t  grandmasterPriority1;  // 128 (default)
    uint8_t  grandmasterClockQuality[4]; // Class, accuracy, variance
    uint8_t  grandmasterPriority2;  // 128 (default)
    uint8_t  grandmasterIdentity[8]; // Clock ID
    uint16_t stepsRemoved;          // 0 (we are grandmaster)
    uint8_t  timeSource;            // 0x20 (GPS)
} __attribute__((packed)) ptp_announce_msg_t;
```

### Sync Message Body

```c
typedef struct {
    ptp_header_t header;

    // Sync-specific fields
    uint8_t originTimestamp[10];    // Seconds (6 bytes) + nanoseconds (4 bytes)
} __attribute__((packed)) ptp_sync_msg_t;
```

### Follow_Up Message Body

```c
typedef struct {
    ptp_header_t header;

    // Follow_Up-specific fields
    uint8_t preciseOriginTimestamp[10]; // Precise timestamp from Sync
} __attribute__((packed)) ptp_followup_msg_t;
```

---

## Implementation Steps

### Step 1: Create PTP Protocol Module
**Files**: `common/ptp_protocol.h`, `common/ptp_protocol.c`

- Define packet structures
- Helper functions for building messages
- Network byte order conversion (htons, htonl)

### Step 2: Extend Shared State
**File**: `grandmaster/shared_state.h`

- Add disciplined clock timestamp fields
- Add PTP statistics (packets sent, sequence numbers)

### Step 3: Update Core 1
**File**: `grandmaster/discipline_core.c`

- Publish disciplined clock value every loop iteration
- No other changes (keep timing-critical code untouched)

### Step 4: WiFi Initialization
**File**: `grandmaster/main.c`

- Initialize CYW43 driver
- Connect to WiFi with credentials
- Configure static IP
- Initialize lwIP stack

### Step 5: PTP Grandmaster Implementation
**Files**: `grandmaster/ptp_grandmaster.h`, `grandmaster/ptp_grandmaster.c`

- Initialize UDP sockets
- Periodic timer (1 second) for message transmission
- Generate and send Announce, Sync, Follow_Up
- Read disciplined clock for timestamps

### Step 6: Integration
**File**: `grandmaster/main.c`

- Call PTP initialization after WiFi setup
- Main loop calls PTP periodic function
- Monitor and print statistics

### Step 7: Testing & Validation
- Wireshark capture of PTP packets
- Verify packet structure with PTP dissector
- Measure impact on Core 1 timing
- Verify GPS discipline still locked

---

## Testing Plan

### Phase 2b.1: WiFi Connectivity
**Goal**: Verify WiFi works, doesn't disrupt timing

**Test**:
1. Flash firmware with WiFi enabled
2. Check WiFi connects (LED or serial output)
3. Ping device from PC: `ping 192.168.1.100`
4. Measure 100 PPS output with oscilloscope
5. Compare jitter to Phase 2a baseline

**Success Criteria**:
- ✅ WiFi connects within 10 seconds
- ✅ Device pingable from network
- ✅ 100 PPS jitter increase <30 μs
- ✅ GPS discipline stays locked

### Phase 2b.2: PTP Packet Transmission
**Goal**: Verify PTP messages are correctly formatted

**Test**:
1. Start Wireshark on PC, filter: `ptp`
2. Capture packets from 192.168.1.100
3. Verify Announce, Sync, Follow_Up appear
4. Use Wireshark PTP dissector to check fields

**Success Criteria**:
- ✅ 1 Announce per second visible
- ✅ 1 Sync per second visible
- ✅ 1 Follow_Up per second visible
- ✅ Timestamps are reasonable (not zero, not garbage)
- ✅ Sequence numbers increment

### Phase 2b.3: Timestamp Accuracy
**Goal**: Verify timestamps from disciplined clock are correct

**Test**:
1. Capture PTP packets for 60 seconds
2. Extract Follow_Up timestamps
3. Calculate timestamp differences (should be ~1.000000 seconds)
4. Look for drift or jumps

**Success Criteria**:
- ✅ Timestamp intervals are 1.000000 sec ±0.001 sec
- ✅ No jumps or discontinuities
- ✅ Reflects GPS discipline (stable frequency)

### Phase 2b.4: Timing Impact Assessment
**Goal**: Quantify WiFi impact on Core 1 timing

**Test**:
1. Oscilloscope on GPIO 3 (100 PPS output)
2. Measure jitter over 5 minutes
3. Compare to Phase 2a baseline
4. Look for periodic disturbances (WiFi beacons, etc.)

**Success Criteria**:
- ✅ Jitter increase <50 μs (acceptable)
- ⚠️ Jitter increase 50-100 μs (marginal, monitor)
- ❌ Jitter increase >100 μs (need hardware timers)

**Decision Point**: If jitter >100 μs, implement hardware timer approach before Phase 3.

---

## Risks & Mitigations

### Risk 1: PIO Conflict (CYW43 vs GPS)
**Problem**: CYW43 WiFi uses PIO for SPI, might conflict with GPS PPS capture

**Mitigation**:
- Use different PIO blocks (pio0 vs pio1) if possible
- Monitor for GPS PPS capture failures
- If conflict occurs, move GPS to different PIO/SM

**Likelihood**: Low (SDK usually handles this)

### Risk 2: Memory Bus Contention
**Problem**: WiFi DMA saturates bus, slows Core 1

**Mitigation**:
- Run Core 1 code from RAM (`__attribute__((section(".time_critical")))`
- Reduce WiFi buffer sizes
- Monitor with oscilloscope

**Likelihood**: Medium

### Risk 3: Interrupt Storms
**Problem**: WiFi interrupts disrupt Core 1 timing

**Mitigation**:
- Use `pico_cyw43_arch_lwip_threadsafe_background` (offloads to async worker)
- Reduce WiFi interrupt rate
- Pin Core 1 to specific CPU if needed

**Likelihood**: Low (lwIP background mode helps)

### Risk 4: lwIP Blocking Calls
**Problem**: Network operations block Core 0 main loop

**Mitigation**:
- Use non-blocking sockets
- Don't call blocking network functions in time-sensitive paths
- Monitor loop timing

**Likelihood**: Low (we control the code)

---

## Configuration Parameters

### User Must Configure

```c
// WiFi credentials
#define WIFI_SSID "YourNetworkSSID"
#define WIFI_PASSWORD "YourNetworkPassword"

// Network configuration
#define STATIC_IP "192.168.1.100"
#define NETMASK "255.255.255.0"
#define GATEWAY "192.168.1.1"
```

### PTP Configuration (Can Leave As-Is)

```c
// PTP domain and identity
#define PTP_DOMAIN 0
#define PTP_CLOCK_IDENTITY (MAC address based, auto-generated)

// Message intervals
#define ANNOUNCE_INTERVAL_SEC 1
#define SYNC_INTERVAL_SEC 1

// PTP priorities and quality
#define GM_PRIORITY1 128
#define GM_PRIORITY2 128
#define CLOCK_CLASS 6        // GPS-locked
#define CLOCK_ACCURACY 0x20  // <100ns
```

---

## Performance Expectations

### Network Load
- **3 packets/second** (Announce + Sync + Follow_Up)
- **~300 bytes/second** total
- **Negligible** network impact

### CPU Load (Core 0)
- **WiFi maintenance**: ~5-10% background
- **PTP generation**: <1% (only 1/sec)
- **Total**: ~10-15% Core 0 usage

### CPU Load (Core 1)
- **No change** from Phase 2a
- Should remain 100% dedicated to timing

### Expected Timing Performance
- **GPS discipline**: ±5-15 μs (same as Phase 2a)
- **100 PPS jitter**: ±15-40 μs (slight increase expected)
- **PTP timestamps**: Accurate to ±10 μs of GPS

---

## Success Criteria Summary

Phase 2b is successful if:

1. ✅ WiFi connects reliably
2. ✅ Static IP assigned and pingable
3. ✅ PTP packets visible in Wireshark
4. ✅ Packet format correct (passes PTP dissector)
5. ✅ Timestamps from disciplined clock
6. ✅ GPS discipline maintains lock
7. ✅ 100 PPS jitter increase <50 μs
8. ✅ System runs stably for >10 minutes

---

## Next Steps After Phase 2b

### Phase 3: PTP Slave Implementation
- Implement slave on second Pico W
- Receive Announce, Sync, Follow_Up
- Discipline local clock to grandmaster
- Generate 100 PPS output synchronized to GM

### Phase 4: Reference Monitor
- Third Pico with second GPS
- Capture GM 100 PPS and Slave 100 PPS
- Measure phase error at 100 Hz
- Log and analyze synchronization quality

---

## References

- **IEEE 1588-2008**: Precision Time Protocol specification
- **Pico SDK**: WiFi and lwIP documentation
- **lwIP**: Lightweight TCP/IP stack
- **Wireshark PTP Dissector**: For packet validation

---

## Document History

- **Created**: 2025-01-16 - Initial Phase 2b plan
- **Status**: Ready for implementation

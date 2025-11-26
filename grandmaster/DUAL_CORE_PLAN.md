# Dual-Core PTP Grandmaster Implementation Plan

## Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    RP2040 Pico                          │
├──────────────────────┬──────────────────────────────────┤
│     CORE 0           │         CORE 1                   │
│   (Timing)           │       (Network)                  │
├──────────────────────┼──────────────────────────────────┤
│                      │                                  │
│ - GPS PPS IRQ        │ - W5500 SPI/IRQ                 │
│ - PIO counter        │ - PTP protocol                  │
│ - gps_ns_counter     │ - Packet TX/RX                  │
│ - Stats updates      │ - Calls get_gps_time_ns()       │
│                      │                                  │
│ MINIMAL WORKLOAD     │ HEAVY WORKLOAD                  │
│ Time-critical        │ Can tolerate latency            │
└──────────────────────┴──────────────────────────────────┘
         │                         │
         └────── Shared Memory ────┘
              (volatile vars)
```

## Phase 1: Code Organization (No functionality changes)

### 1.1 Create new main file: `main_dual_core.c`

```c
// Core 0: Timing and discipline
void core0_timing_main(void);

// Core 1: Network and PTP
void core1_network_entry(void);

int main() {
    // Initialize hardware (both cores can access)
    stdio_init_all();

    // Core 0: Initialize timing subsystem
    discipline_init_v3();
    gps_init(...);

    // Launch Core 1 for network
    multicore_launch_core1(core1_network_entry);

    // Core 0 continues with timing
    core0_timing_main();
}
```

### 1.2 Files to modify:
- `grandmaster/main_dual_core.c` (new)
- `grandmaster/CMakeLists.txt` (add new executable)
- `grandmaster/discipline_v3.c` (add thread-safety notes)
- `grandmaster/ptp_grandmaster.c` (move to core 1)

## Phase 2: Core 0 Implementation (Timing)

### 2.1 Core 0 Responsibilities:
```c
void core0_timing_main(void) {
    uint64_t last_stats_time = 0;

    while (true) {
        // Periodic stats (every 10 seconds)
        uint64_t now = time_us_64();
        if (now - last_stats_time >= 10000000) {
            print_timing_stats();
            last_stats_time = now;
        }

        // Minimal sleep to yield CPU
        sleep_ms(100);
    }
}
```

### 2.2 What runs on Core 0:
- ✅ GPS PPS IRQ handler (already in SRAM)
- ✅ PIO state machines (hardware, no CPU)
- ✅ `get_gps_time_ns()` calls from Core 0 (if needed)
- ✅ Stats printing (low priority)

## Phase 3: Core 1 Implementation (Network)

### 3.1 Core 1 Entry Point:
```c
void core1_network_entry(void) {
    // Wait for Core 0 to initialize timing
    while (!core0_ready) {
        tight_loop_contents();
    }

    // Initialize network on Core 1
    w5500_init();
    ptp_grandmaster_init();

    // Network processing loop
    while (true) {
        w5500_process_packets();
        ptp_process();
    }
}
```

### 3.2 What runs on Core 1:
- ✅ W5500 SPI transactions
- ✅ W5500 IRQ handler
- ✅ PTP packet processing
- ✅ PTP state machine
- ✅ Sync/Follow_Up/Delay_Req packet generation
- ✅ Timestamp capture using `get_gps_time_ns()`

## Phase 4: Thread Safety & Synchronization

### 4.1 Shared State (discipline_v3.c):
```c
// Already volatile - safe for dual-core reads
volatile uint64_t gps_ns_counter;           // Updated only in Core 0 IRQ
volatile uint64_t last_pps_system_us;       // Updated only in Core 0 IRQ
volatile uint32_t measured_ticks_last_second; // Updated only in Core 0 IRQ

// Core 1 only READS these via get_gps_time_ns()
```

### 4.2 Synchronization Flag:
```c
// In shared_state.h
volatile bool core0_ready = false;  // Set by Core 0 after discipline_init_v3()
```

### 4.3 Critical Section Analysis:
**NO LOCKS NEEDED** because:
- Core 0: WRITES timing variables (only in IRQ, 1 Hz)
- Core 1: READS timing variables (via get_gps_time_ns())
- Worst case: Core 1 reads during Core 0 IRQ → off by 1 GPS PPS cycle (<50ns transient)
- ARM Cortex-M0+ guarantees atomic 32/64-bit aligned reads

## Phase 5: PTP Packet Timestamping

### 5.1 TX Timestamp (Sync packet):
```c
// In W5500 TX complete callback (Core 1):
void ptp_sync_tx_complete(void) {
    uint64_t tx_timestamp_ns = get_gps_time_ns();

    // Format for Follow_Up packet
    ptp_timestamp.seconds = tx_timestamp_ns / 1000000000ULL;
    ptp_timestamp.nanoseconds = tx_timestamp_ns % 1000000000ULL;
}
```

### 5.2 RX Timestamp (Delay_Req from slave):
```c
// In W5500 RX callback (Core 1):
void ptp_delay_req_received(void) {
    uint64_t rx_timestamp_ns = get_gps_time_ns();

    // Send Delay_Resp with this timestamp
    ptp_timestamp.seconds = rx_timestamp_ns / 1000000000ULL;
    ptp_timestamp.nanoseconds = rx_timestamp_ns % 1000000000ULL;
}
```

## Phase 6: CMakeLists.txt Changes

```cmake
# New dual-core grandmaster executable
add_executable(grandmaster_dual_core
    main_dual_core.c
    gps.c
    discipline_v3.c
    ptp_grandmaster.c
)

target_include_directories(grandmaster_dual_core PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}
)

target_compile_definitions(grandmaster_dual_core PRIVATE
    BUILD_GRANDMASTER
    DUAL_CORE_BUILD
)

target_link_libraries(grandmaster_dual_core
    pico_stdlib
    pico_multicore          # ← CRITICAL: Add multicore support
    hardware_pio
    hardware_uart
    hardware_timer
    hardware_irq
    hardware_clocks
    hardware_gpio
    common
    network
)

pico_generate_pio_header(grandmaster_dual_core ${CMAKE_CURRENT_LIST_DIR}/pio/gps_discipline_v2.pio)
pico_generate_pio_header(grandmaster_dual_core ${CMAKE_SOURCE_DIR}/pio/pps_capture.pio)
pico_enable_stdio_usb(grandmaster_dual_core 1)
pico_add_extra_outputs(grandmaster_dual_core)
```

## Phase 7: Testing & Validation

### 7.1 Unit Tests:
1. **Core 0 only**: Flash and verify timing stats (baseline)
2. **Core 1 launch**: Verify Core 1 starts and prints debug message
3. **Timestamp reads**: Core 1 reads get_gps_time_ns() every 100ms, verify no crashes
4. **Stress test**: Core 1 reads get_gps_time_ns() at 10 kHz, verify timing accuracy maintained

### 7.2 Integration Tests:
1. **W5500 init**: Verify network stack initializes on Core 1
2. **PTP packets**: Send Sync packets, verify timestamps are GPS-locked
3. **Slave sync**: Connect real PTP slave, verify synchronization
4. **Long-term stability**: Run for 24 hours, verify no timing drift

### 7.3 Performance Metrics:
```
Core 0 stats (every 10s):
PPS:157 GPS:157s crystal_err:+30564ns (+30.564ppm) interp_err:+435ns lock:YES

Core 1 stats (every 10s):
PTP: Sync_sent:157 Delay_Req_rx:156 Offset_mean:123ns Offset_std:45ns
```

## Phase 8: Migration Strategy

### 8.1 Incremental approach:
1. ✅ **Step 1**: Create main_dual_core.c, launch Core 1 with empty loop
2. ✅ **Step 2**: Move stats printing to Core 1 (verify no crashes)
3. ✅ **Step 3**: Core 1 calls get_gps_time_ns() and prints (verify thread safety)
4. ✅ **Step 4**: Initialize W5500 on Core 1
5. ✅ **Step 5**: Move PTP protocol to Core 1
6. ✅ **Step 6**: Full PTP grandmaster functionality

### 8.2 Fallback:
Keep `grandmaster_minimal_test` (single-core) as fallback for debugging

## Expected Benefits

✅ **Core 0**: Minimal load, deterministic GPS PPS handling
✅ **Core 1**: Heavy network processing doesn't affect timing
✅ **Timestamp accuracy**: Still ~200-400ns (no degradation)
✅ **PTP performance**: Can handle higher packet rates
✅ **Scalability**: Easy to add features to Core 1 without timing impact

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Core 1 reads during Core 0 IRQ | Transient <50ns error | Acceptable for PTP |
| UART stdio conflicts | Printf corruption | Use Core 0 for all printf, or use mutex |
| Flash cache thrashing | Both cores miss cache | Both IRQs in SRAM (already done) |
| Multicore init issues | Hang at boot | Test incrementally, add debug prints |

## Implementation Status

- [ ] Phase 1: Code Organization
- [ ] Phase 2: Core 0 Implementation
- [ ] Phase 3: Core 1 Implementation
- [ ] Phase 4: Thread Safety
- [ ] Phase 5: PTP Timestamping
- [ ] Phase 6: CMakeLists.txt
- [ ] Phase 7: Testing
- [ ] Phase 8: Migration

## Notes

- Single-core minimal test (`grandmaster_minimal_test`) achieved ±200-400ns interpolation accuracy
- V3 discipline uses GPIO pin signaling, no continuous DMA
- All timing-critical functions already in SRAM (`__time_critical_func`)
- `get_gps_time_ns()` is thread-safe for reads from Core 1

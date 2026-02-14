# LTSP v0.1 — Implementation Plan

## Architectural Decision (Resolved)

**GPS nanoseconds are the common timestamp domain** (Option B from comparison.md §5).
All PDU timestamp fields (Prev_Tx_Timestamp, Model_Epoch, Last_1PPS_Count, a0, Sigma,
GM_Local_Processing_Mean) are in nanoseconds. Only Last_1PPS_Interval remains in PIO
ticks (single-SM measurement). The RFC and implementation guide have been updated to
reflect this decision and the actual hardware pin assignments, SPI speed, and PIO
tick rate (12ns, 83.33MHz).

---

## Work Items (Dependency Order)

### Phase 1: Common Library — No Hardware Dependency

These modules compile with host gcc for testing and with pico-sdk for firmware.
No PIO, no W5500, no GPIO.

#### 1.1. LTSP PDU Pack/Unpack
- **Files:** `common/ltsp_pdu.h`, `common/ltsp_pdu.c`
- **Action:** New
- **Complexity:** Moderate
- **Description:**
  - Define `ltsp_pdu_t` struct matching RFC §5 (56 bytes)
  - `ltsp_pdu_pack()` — serialize struct to wire format (big-endian)
  - `ltsp_pdu_unpack()` — deserialize wire bytes to struct (little-endian host)
  - Explicit byte manipulation for endianness (no ntohl/htons — bare metal)
  - Field types: uint8_t version/flags, uint16_t sequence, int64_t prev_tx/epoch/a0/1pps_count, float a1/sigma, uint32_t 1pps_interval/gm_local_mean
  - Prev_Tx_Timestamp, Model_Epoch, Last_1PPS_Count, and a0 are int64_t nanoseconds. Last_1PPS_Interval stays uint32_t PIO ticks. GM_Local_Processing_Mean is uint32_t nanoseconds.
- **Dependencies:** None
- **Tests:** `test/test_pdu.c` — round-trip, byte-order verification, field boundary checks

#### 1.2. Sequence Number Handler
- **Files:** `common/ltsp_sequence.h`, `common/ltsp_sequence.c`
- **Action:** New
- **Complexity:** Simple
- **Description:**
  - `ltsp_seq_init()` — initialize state
  - `ltsp_seq_validate()` — check incoming sequence against expected
  - Returns: ACCEPT, GAP (1-3 missing), RESTART (gap > 3), DUPLICATE
  - Handles 16-bit wraparound (65535 → 0)
  - Tracks loss count
- **Dependencies:** None
- **Tests:** `test/test_sequence.c` — normal, gap, wraparound, backward, restart

#### 1.3. Minimum Filter
- **Files:** `common/ltsp_min_filter.h`, `common/ltsp_min_filter.c`
- **Action:** New
- **Complexity:** Simple
- **Description:**
  - Circular buffer of W samples (default 120)
  - Track current minimum and age
  - Full scan when minimum ages out
  - Path jump detection (threshold 10µs = 10000ns)
  - `ltsp_min_filter_init()`, `ltsp_min_filter_update()`, `ltsp_min_filter_get_min()`
  - `ltsp_min_filter_reset()` — for path reroute or protocol restart
- **Dependencies:** None
- **Tests:** `test/test_min_filter.c` — known sequence, expiry, path jump, reset

#### 1.4. GPS Regression (Least Squares)
- **Files:** `common/ltsp_regression.h`, `common/ltsp_regression.c`
- **Action:** New
- **Complexity:** Moderate
- **Description:**
  - Circular buffer of REGRESSION_WINDOW (60) samples: {t_k, e_k}
  - Running sums: S_t, S_e, S_tt, S_te, S_ee (double precision)
  - `ltsp_regression_init()`, `ltsp_regression_add_sample()`
  - `ltsp_regression_compute()` — returns a0, a1, Sigma
  - Re-reference a0 to arbitrary T_epoch
  - Periodic recompute from scratch every REGRESSION_WINDOW cycles
  - **Integration:** Called from GM's 1PPS handler with phase error measurements
  - **Input domain:** PIO ticks (single SM, no cross-domain issue)
- **Dependencies:** None
- **Tests:** `test/test_regression.c` — known drift, zero drift, noise, convergence, re-reference

#### 1.5. 64-bit PIO Timestamp Extension
- **Files:** `common/ltsp_pio_timestamp.h`, `common/ltsp_pio_timestamp.c`
- **Action:** New
- **Complexity:** Simple
- **Description:**
  - `ltsp_pio_ts_t` — struct with upper32 + lower32
  - `ltsp_pio_ts_extend()` — given new 32-bit value and previous, detect wrap, update upper half
  - `ltsp_pio_ts_to_u64()` — combine to 64-bit
  - `ltsp_pio_ts_diff()` — compute signed difference between two 64-bit timestamps
  - Counter counts DOWN: wrap detected when new > previous
  - At 83.33MHz, wraps every ~51.5s (0xFFFFFFFF * 12ns). At 1Hz measurement, at most 1 wrap between samples.
  - Used on Receiver to maintain 64-bit T_pio_rx for deferred processing (must correlate T_pio_rx[N-1] with Prev_Tx_Timestamp in next packet).
- **Dependencies:** None
- **Tests:** `test/test_pio_timestamp.c` — normal, wrap, multi-wrap, diff

### Phase 2: Build System

#### 2.1. CMake Configuration
- **Files:** `CMakeLists.txt` (modify top-level), `ltsp/CMakeLists.txt` (new), `ltsp/common/CMakeLists.txt`, `ltsp/gm/CMakeLists.txt`, `ltsp/receiver/CMakeLists.txt`, `ltsp/test/CMakeLists.txt`
- **Action:** New directory structure alongside existing code
- **Complexity:** Moderate
- **Description:**
  - Create `ltsp/` directory per implementation guide §2.2
  - Common library compiles for both pico-sdk and host gcc
  - GM target: `ltsp_gm` (.uf2)
  - Receiver target: `ltsp_receiver` (.uf2)
  - Test target: `ltsp_test` (host executable, no pico-sdk)
  - Reuse existing PIO programs via include paths (don't duplicate)
  - Reuse `w5500_simple.c`, `eth_simple.c` via include paths
- **Dependencies:** 1.1-1.5 (needs source files to exist)

#### 2.2. Host-Side Test Harness
- **Files:** `ltsp/test/test_main.c`, `ltsp/test/CMakeLists.txt`
- **Action:** New
- **Complexity:** Simple
- **Description:**
  - Minimal test runner (no framework dependency)
  - Compile with `gcc -o ltsp_test test_main.c ../common/*.c -lm`
  - Run assertions, report pass/fail
  - Include all test files from Phase 1
- **Dependencies:** 1.1-1.5, 2.1

### Phase 3: GM LTSP Frame Construction

#### 3.1. LTSP Ethernet Frame Builder
- **Files:** Modify `grandmaster/eth_simple.c/.h`
- **Action:** Adapt
- **Complexity:** Simple
- **Description:**
  - Add `eth_build_ltsp(uint8_t *frame, const uint8_t *src_mac, const uint8_t *payload, uint16_t payload_len)`
  - Dest MAC: FF:FF:FF:FF:FF:FF (broadcast)
  - EtherType: 0x88B5
  - No IP/UDP headers — just Ethernet + payload
  - Total: 14 (Eth header) + 56 (PDU) = 70 bytes
  - W5500 handles FCS automatically
- **Dependencies:** 1.1

#### 3.2. GM Regression Integration
- **Files:** Modify `grandmaster/discipline_v3.c`
- **Action:** Adapt
- **Complexity:** Moderate
- **Description:**
  - Add `ltsp_regression_t` instance to discipline state
  - In PPS IRQ handler: after computing phase_error_ticks, call `ltsp_regression_add_sample(counter_at_pps, phase_error_ticks)`
  - In main loop (not IRQ): call `ltsp_regression_compute()` before PDU construction
  - Export a0, a1, Sigma via new getter functions
  - Keep existing single-sample crystal error for `get_gps_time_ns()` (it works)
- **Dependencies:** 1.4

#### 3.3. GM Local Processing Delay Tracking
- **Files:** Modify `grandmaster/hw_timestamp.c/.h`
- **Action:** Adapt
- **Complexity:** Simple
- **Description:**
  - Add running mean of TX latency (already computed in `hw_timestamp_find_tx()`)
  - `hw_timestamp_get_tx_latency_mean()` — returns uint32_t in PIO ticks
  - EMA with α=0.1, stored as double, returned as uint32_t
  - This becomes the `GM_Local_Processing_Mean` PDU field
- **Dependencies:** None (builds on existing code)

#### 3.4. GM Deferred Timestamp Storage
- **Files:** New state in GM main or `hw_timestamp.c`
- **Action:** Adapt
- **Complexity:** Simple
- **Description:**
  - After TX of packet N, store the HW timestamp as `prev_tx_timestamp`
  - Convert to GPS nanoseconds (Option B) or keep as 64-bit PIO ticks
  - On next packet construction, embed this value in Prev_Tx_Timestamp field
  - First packet: Prev_Tx_Timestamp = 0
  - If HW timestamp capture fails: use EMA fallback (already exists) or set to 0
- **Dependencies:** 3.3

#### 3.5. GM Main Loop (LTSP)
- **Files:** `ltsp/gm/main.c` (new, heavily based on `grandmaster/main_w5500_dual_core.c`)
- **Action:** New (adapted from existing)
- **Complexity:** Complex
- **Description:**
  - Core 0: GPS discipline (reuse `discipline_v3.c` with regression from 3.2)
  - Core 1: LTSP transmission loop
    1. Every 1 second (triggered by 1PPS flag or timer):
    2. Read regression model (a0, a1, Sigma) — computed by Core 0
    3. Construct LTSP PDU:
       - Version = 1, Flags = 0 (no AUTH, no HOLDOVER)
       - Sequence = N (increment)
       - Prev_Tx_Timestamp = stored from packet N-1 (GPS ns)
       - Model_Epoch = current GPS time (`get_gps_time_ns()`)
       - a0, a1, Sigma from regression
       - Last_1PPS_Count = GPS ns of most recent 1PPS
       - Last_1PPS_Interval = measured ticks (PIO ticks, uint32)
       - GM_Local_Processing_Mean = from hw_timestamp
    4. Pack PDU to wire bytes
    5. Build Ethernet frame (EtherType 0x88B5, broadcast)
    6. Record T_sys before SPI send
    7. Send via `w5500_send_frame()`
    8. Capture HW timestamp for TX (poll INTn FIFO)
    9. Convert to GPS ns, store as prev_tx_timestamp for next packet
    10. Update GM local processing stats
  - Keep: W5500 init, ARP/ICMP handling (for network debugging), HW timestamp polling
  - Remove: PTP Sync/FollowUp/DelayReq/DelayResp handling
  - USB serial diagnostics: 1PPS count, crystal error, regression params, TX timestamp, local processing delay
- **Dependencies:** 1.1, 1.4, 1.5, 3.1-3.4

### Phase 4: Receiver LTSP Processing

#### 4.1. LTSP Frame Reception and Parsing
- **Files:** `ltsp/receiver/main.c` (new, based on `slave/main_w5500_slave.c`)
- **Action:** New (adapted from existing)
- **Complexity:** Moderate
- **Description:**
  - Core 1: Network reception
    1. Poll W5500 for frames
    2. Filter by EtherType 0x88B5 (ignore others, but still handle ARP/ICMP)
    3. Unpack LTSP PDU
    4. Validate version (must be 1)
    5. Validate sequence (using sequence handler from 1.2)
    6. Capture T_pio_rx from HW timestamp (INTn correlation)
    7. Capture T_sys_spi_rx from System Timer
    8. Store {seq, T_pio_rx, T_sys_spi_rx, PDU fields} for Core 0
  - Core 0: Deferred processing
    1. When new packet arrives with valid Prev_Tx_Timestamp:
    2. Retrieve stored T_pio_rx[N-1] from previous packet
    3. Perform error decomposition (Phase 4.2)
    4. Update minimum filter (Phase 4.3)
    5. Compute and report offset
- **Dependencies:** 1.1, 1.2, 1.5, 3.1

#### 4.2. Error Decomposition
- **Files:** `ltsp/receiver/error_decomp.h`, `ltsp/receiver/error_decomp.c`
- **Action:** New
- **Complexity:** Moderate
- **Description:**
  - `error_decomp_compute()`:
    1. T_gps_tx[N-1] is directly from PDU Prev_Tx_Timestamp (GPS ns).
    2. T_rx_ns[N-1] = T_pio_rx[N-1] * 12 (convert receiver PIO ticks to ns).
    3. D_total = T_rx_ns[N-1] - T_gps_tx[N-1] (includes receiver crystal error).
    4. D_gm_local = GM_Local_Processing_Mean (from PDU, already nanoseconds).
    5. D_rx_local = T_sys_spi_rx - cross_cal(T_pio_rx) (measured locally, ns).
    6. D_wire = D_total - D_gm_local - D_rx_local
  - During initial acquisition (no receiver clock model): use D_total directly with minimum filter
- **Dependencies:** 1.5

#### 4.3. Receiver Offset Computation and CSV Reporting
- **Files:** Part of `ltsp/receiver/main.c`
- **Action:** New
- **Complexity:** Moderate
- **Description:**
  - Apply minimum filter to D_wire (or D_total during acquisition)
  - Compute clock offset: offset = Wire_Delay_Est - D_wire_min_floor
  - Actually, offset = T_pio_rx - (T_gps_tx + Wire_Delay_Est + D_gm_local + D_rx_local)
  - Report CSV per implementation guide §2.8:
    `seq,t_pio_rx,t_pio_tx,d_total,d_wire_est,d_gm_local,d_rx_local,offset_ns,sigma,a0,a1,1pps_interval`
  - One line per processed deferred packet (not every received packet)
  - Values in PIO ticks (multiply by 12 for nanoseconds)
- **Dependencies:** 1.3, 4.1, 4.2

### Phase 5: Integration and Validation

#### 5.1. End-to-End Integration Test
- **Action:** Build both GM and receiver, flash, connect via crossover
- **Complexity:** Complex (debugging)
- **Description:**
  - Verify LTSP frames are transmitted and received
  - Verify sequence numbers increment correctly
  - Verify deferred timestamps are populated after first packet
  - Verify error decomposition produces reasonable values
  - Verify minimum filter converges
  - Verify CSV output contains valid data
  - Run for 300+ seconds (per implementation guide §3.2)
- **Pass criteria:**
  - No sequence gaps on crossover
  - D_wire stable (constant on crossover)
  - GM local processing delay consistent
  - 1PPS interval consistent with crystal error
  - Offset mean < 1µs after acquisition
  - Offset stddev within 2x Sigma
- **Dependencies:** All above

---

## File/Directory Plan

```
ltsp/                           (NEW directory, alongside existing grandmaster/ and slave/)
├── CMakeLists.txt              (NEW — top-level LTSP build)
├── common/
│   ├── CMakeLists.txt          (NEW)
│   ├── ltsp_pdu.h              (NEW — PDU structure and pack/unpack)
│   ├── ltsp_pdu.c              (NEW)
│   ├── ltsp_sequence.h         (NEW — sequence validation)
│   ├── ltsp_sequence.c         (NEW)
│   ├── ltsp_min_filter.h       (NEW — minimum delay filter)
│   ├── ltsp_min_filter.c       (NEW)
│   ├── ltsp_regression.h       (NEW — OLS regression)
│   ├── ltsp_regression.c       (NEW)
│   ├── ltsp_pio_timestamp.h    (NEW — 64-bit PIO timestamp extension)
│   └── ltsp_pio_timestamp.c    (NEW)
├── gm/
│   ├── CMakeLists.txt          (NEW)
│   └── main.c                  (NEW — adapted from grandmaster/main_w5500_dual_core.c)
├── receiver/
│   ├── CMakeLists.txt          (NEW)
│   ├── main.c                  (NEW — adapted from slave/main_w5500_slave.c)
│   ├── error_decomp.h          (NEW)
│   └── error_decomp.c          (NEW)
└── test/
    ├── CMakeLists.txt          (NEW — host gcc, no pico-sdk)
    ├── test_main.c             (NEW — test runner)
    ├── test_pdu.c              (NEW)
    ├── test_sequence.c         (NEW)
    ├── test_min_filter.c       (NEW)
    ├── test_regression.c       (NEW)
    └── test_pio_timestamp.c    (NEW)

Modified existing files:
├── grandmaster/
│   ├── eth_simple.c            (ADD eth_build_ltsp function)
│   ├── eth_simple.h            (ADD declaration)
│   ├── discipline_v3.c         (ADD regression integration)
│   └── hw_timestamp.c          (ADD TX latency running mean)
└── CMakeLists.txt              (ADD ltsp subdirectory)
```

---

## Complexity Estimates

| Item | Complexity | Estimated New Lines | Risk |
|------|-----------|-------------------|------|
| 1.1 PDU pack/unpack | Moderate | ~250 | Low — straightforward byte manipulation |
| 1.2 Sequence handler | Simple | ~80 | Low |
| 1.3 Minimum filter | Simple | ~100 | Low |
| 1.4 GPS regression | Moderate | ~200 | Medium — numerical stability, must verify against known data |
| 1.5 64-bit timestamp | Simple | ~60 | Low |
| 2.1 CMake config | Moderate | ~150 | Medium — cross-compilation paths, pico-sdk integration |
| 2.2 Test harness | Simple | ~80 | Low |
| 3.1 LTSP frame builder | Simple | ~30 | Low |
| 3.2 GM regression integration | Moderate | ~50 | Low — just wiring |
| 3.3 GM local proc. delay | Simple | ~30 | Low |
| 3.4 Deferred timestamp | Simple | ~20 | Low |
| 3.5 GM main loop | Complex | ~400 | Medium — integration of many components |
| 4.1 Receiver frame handling | Moderate | ~300 | Medium — deferred processing logic |
| 4.2 Error decomposition | Moderate | ~150 | Medium — cross-clock domain math |
| 4.3 CSV reporting | Moderate | ~100 | Low |
| 5.1 Integration test | Complex | — | High — real hardware, debugging |
| **Total** | | **~2000** | |

---

## RFC and Implementation Guide Updates (Applied)

The following changes have been applied to both documents:

1. PIO_TICK_HZ = 83.33MHz, PIO_TICK_NS = 12 (was 250MHz / 4ns)
2. SM allocation documents two PIO blocks on GM, cross-block bridging via GPS-ns
3. All PDU timestamp fields specified as GPS nanoseconds (except Last_1PPS_Interval)
4. GPIO assignments match existing hardware (GPS PPS = GPIO 2, W5500 INT = GPIO 21, etc.)
5. SPI speed = 40MHz (was 33MHz)
6. PIO appendix replaced with actual tested programs from existing codebase

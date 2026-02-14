# LTSP v0.1 — Existing Code vs RFC Requirements Comparison

## Executive Summary

The existing codebase implements a full IEEE 1588 PTP system with GPS-disciplined
grandmaster and PI-servo slave. Approximately 60% of the LTSP infrastructure already
exists in working, tested form. The major gaps are: LTSP PDU format, deferred
timestamp mechanism, least-squares regression on the GM, minimum filter, error
decomposition model, and MACRAW broadcast framing with EtherType 0x88B5.

Several RFC assumptions conflict with real hardware experience. These are documented
in Section 3 below.

---

## 1. Requirements Mapping

| # | Requirement (RFC Section) | Existing Code | Status | Notes |
|---|---------------------------|---------------|--------|-------|
| 1 | **PIO 1PPS capture (§3.1, §10)** | `grandmaster/pio/gps_discipline_v2.pio` SM0+SM1, `grandmaster/discipline_v3.c` | **Done** | SM1 detects PPS rising edge on GPIO 2, pulses trigger pin to SM0 which snapshots X counter. 250MHz clock, 4ns/tick counter, ~12ns detection. IRQ handler extends, stores, computes. |
| 2 | **PIO INTn capture — GM TX (§3.1, §4.2)** | `grandmaster/pio/int_timestamp.pio`, `grandmaster/hw_timestamp.c` | **Done** | PIO1 SM1 detects W5500 INT falling edge, pushes marker. Software reads SM0 counter atomically. Circular buffer of 16 timestamps. Direction-filtered correlation. |
| 3 | **PIO INTn capture — RX (§3.1, §4.2)** | `slave/pio/int_timestamp.pio`, `slave/ptp_discipline.c:308-326` | **Done** | Same PIO program as GM. Identical correlation logic. |
| 4 | **PIO free-running counter (§3.1)** | `grandmaster/pio/counter_simple.pio`, `slave/pio/counter_simple.pio` | **Done** | 3-cycle loop at 250MHz = 83.33MHz effective = 12ns/tick. Counts DOWN. **Differs from RFC** (see §3). |
| 5 | **PIO counter 64-bit extension (§3.1)** | Not implemented | **New** | Existing code uses 32-bit counters only. Wrap handled per-comparison but no persistent 64-bit upper half. RFC requires 64-bit synthetic timestamps for PDU fields. |
| 6 | **System Timer capture (§3.2)** | `grandmaster/discipline_v3.c:141`, `slave/ptp_discipline.c` | **Done** | `time_us_64()` captured at IRQ entry and at SPI operations. |
| 7 | **Cross-domain calibration (§3.3)** | `grandmaster/discipline_v3.c:141,205-207` | **Partial** | System timer captured at PPS. Used for interpolation. Not explicitly stored as calibration pair for PIO↔sys conversion. Needs formalization for GM local processing delay. |
| 8 | **GPS crystal calibration (§3.4)** | `grandmaster/discipline_v3.c:196-201` | **Done** | Measures ticks between 1PPS, computes phase error and ppb. Single-sample (not regression). |
| 9 | **W5500 MACRAW mode (§4.1)** | `grandmaster/w5500_simple.c` | **Done** | Socket 0 MACRAW, SPI at 40MHz. Frame send/receive with pointer management. |
| 10 | **Ethernet frame construction (§5)** | `grandmaster/eth_simple.c` | **Adapt** | Builds complete Ethernet frames (MAC+EtherType+payload). Currently builds IPv4/UDP frames. Need to add raw EtherType 0x88B5 frame builder (simpler than UDP). |
| 11 | **LTSP PDU format (§5)** | — | **New** | 56-byte PDU with version, flags, sequence, deferred timestamp, model epoch, a0, a1, Sigma, 1PPS count/interval, GM local processing mean. Needs pack/unpack with big-endian byte order. |
| 12 | **Deferred timestamp mechanism (§6.1)** | — | **New** | Store T_pio_tx from packet N, embed in packet N+1. Core concept of LTSP. GM already captures INTn for TX; needs to store and defer. |
| 13 | **GM transmission loop (§6.1)** | `grandmaster/main_w5500_dual_core.c:103-209` | **Adapt** | Existing sends PTP Sync+FollowUp every 1s. Need to replace with LTSP PDU construction. The 1Hz cadence and HW timestamp capture are reusable. |
| 14 | **Least-squares regression (§8.2)** | — | **New** | GM currently stores single-sample crystal error. RFC requires circular buffer of 60 1PPS samples and OLS regression for a0, a1, Sigma. |
| 15 | **Receiver frame reception (§6.2)** | `slave/main_w5500_slave.c`, `slave/ptp_slave_w5500.c` | **Adapt** | Existing receives PTP frames via MACRAW. Need to: change frame filter to EtherType 0x88B5, parse LTSP PDU instead of PTP, implement deferred processing. |
| 16 | **Sequence number handling (§6.3)** | — | **New** | 16-bit sequence with gap detection, wraparound, protocol restart on large gaps. |
| 17 | **Error decomposition (§7)** | — | **New** | D_total = D_gm_local + D_wire + D_rx_local. Uses deferred timestamps and GM's reported local processing mean. |
| 18 | **Cross-clock delay calculation (§7.2)** | — | **New** | Convert GM PIO ticks to GPS time using a0/a1, then to receiver time domain. |
| 19 | **Minimum filter (§8.1)** | — | **New** | Circular buffer of 120 wire delay samples. Track minimum with age-based expiry. Path jump detection. |
| 20 | **Holdover state machine (§9)** | — | **Deferred** | v0.2 per implementation guide. |
| 21 | **Receiver clock discipline (§9.1)** | `slave/ptp_discipline.c:467-499` | **Deferred** | PI servo exists but v0.1 only reports offsets, no clock adjustment. |
| 22 | **GM local processing delay (§7)** | `grandmaster/hw_timestamp.c` | **Adapt** | GM already measures TX latency (SPI init to INTn). Need to: maintain running mean, include in PDU. |
| 23 | **RX local processing estimation (§7)** | `slave/ptp_discipline.c:246-298` | **Adapt** | Slave already measures RX latency (INTn to SPI read complete). Need to formalize for error decomposition. |
| 24 | **Authentication (§11)** | — | **Deferred** | v0.2 per implementation guide. |
| 25 | **USB serial CSV reporting** | `slave/main_w5500_slave.c` | **Adapt** | Existing reports PTP stats. Need new CSV format per implementation guide §2.8. |
| 26 | **Host-side unit tests** | — | **New** | PDU pack/unpack, minimum filter, regression, sequence handling. No pico-sdk dependency. |
| 27 | **Dual-core architecture** | `grandmaster/main_w5500_dual_core.c`, `slave/main_w5500_slave.c` | **Done** | Core 0: timing-critical. Core 1: network. Volatile shared state. |
| 28 | **1PPS output (Receiver)** | `slave/pps_scheduler.c`, `slave/pio/scheduled_1pps.pio` | **Deferred** | v0.2 per implementation guide. v0.1 is report-only. |

---

## 2. Reuse Assessment

### 2a. Code Reusable Directly

| File | What It Provides |
|------|------------------|
| `grandmaster/pio/gps_discipline_v2.pio` | GPS PPS capture — working, tested, better than RFC's illustrative example |
| `grandmaster/pio/counter_simple.pio` | Free-running counter — used by both GM and slave for HW timestamps |
| `grandmaster/pio/int_timestamp.pio` | W5500 INTn edge capture — working on both GM and slave |
| `grandmaster/w5500_simple.c/.h` | W5500 MACRAW driver — complete, working at 40MHz SPI |
| `grandmaster/discipline_v3.c` | GPS 1PPS handling, crystal calibration, `get_gps_time_ns()` |
| `grandmaster/hw_timestamp.c/.h` | HW timestamp correlation with circular buffer |
| `grandmaster/shared_state.h` | Inter-core communication pattern |
| `slave/pio/counter_simple.pio` | Identical to GM version |
| `slave/pio/int_timestamp.pio` | Identical to GM version |

### 2b. Code Needing Adaptation

| File | Current Use | Required Changes |
|------|-------------|------------------|
| `grandmaster/eth_simple.c/.h` | Builds IPv4/UDP/ARP/ICMP frames | Add `eth_build_ltsp()` for raw EtherType 0x88B5 broadcast frames. Much simpler than existing UDP builder. |
| `grandmaster/main_w5500_dual_core.c` | PTP GM main loop | Replace PTP Sync/FollowUp with LTSP PDU construction. Keep dual-core architecture, 1Hz cadence, HW timestamp capture. |
| `slave/main_w5500_slave.c` | PTP slave main loop | Replace PTP message handling with LTSP PDU reception and deferred processing. Keep dual-core architecture. |
| `grandmaster/discipline_v3.c` | Single-sample crystal error | Add circular buffer of 1PPS measurements and OLS regression. Keep existing PPS capture and IRQ handler. |
| `grandmaster/hw_timestamp.c` | TX/RX latency correlation | Add running mean of GM local processing delay for PDU field. |

### 2c. New Code Required

| Module | Location (per impl guide) | Complexity |
|--------|---------------------------|------------|
| `ltsp_pdu.c/.h` | `common/` | Moderate — 56-byte PDU pack/unpack with big-endian byte manipulation |
| `gps_regression.c/.h` | `common/` or `gm/` | Moderate — OLS with circular buffer, running sums, Sigma |
| `min_filter.c/.h` | `common/` | Simple — circular buffer with min tracking and age expiry |
| `error_decomp.c/.h` | `receiver/` | Moderate — cross-clock conversion using a0/a1, component separation |
| `sequence.c/.h` | `common/` | Simple — gap detection, wraparound, protocol restart |
| `pio_timestamp_64.c/.h` | `common/` | Simple — 32-bit to 64-bit extension with wrap tracking |
| Host-side tests | `test/` | Moderate — test harnesses for above modules |
| `gm/main.c` | `gm/` | Complex — new GM main loop (adapt from existing) |
| `receiver/main.c` | `receiver/` | Complex — new receiver main loop (adapt from existing) |

---

## 3. RFC Conflicts with Real Hardware

### 3.1. PIO Counter Resolution

**RFC states:** PIO_CLOCK_HZ = 250MHz, PIO_TICK_NS = 4ns, 3-instruction loop = 12ns detection resolution.

**Reality:** The existing code has TWO counter types with different characteristics:

1. **GM 1PPS counter** (`counter_with_pin_trigger`): Runs at 250MHz (4ns/tick). The 3-instruction loop means ~12ns between edge checks, but the counter itself increments every 4ns. The captured X value has 4ns resolution. `EXPECTED_TICKS_PER_SECOND = 83,333,333` in the code, which is 250M/3 — this means the counter effectively ticks at 83.33MHz despite the 250MHz PIO clock. **The RFC's PIO_TICK_NS = 4ns is wrong for this counter; it's actually 12ns per X decrement.**

2. **HW timestamp counter** (`counter_simple`): Explicitly 3-cycle loop with `[2]` delay. 12ns/tick. `NS_PER_TICK = 12` in `hw_timestamp.c`.

**Implication for LTSP:** All PDU timestamp fields should use 12ns tick resolution (83.33MHz effective), not 4ns. The RFC's `PIO_CLOCK_HZ = 250e6` and `PIO_TICK_NS = 4` should be revised to `PIO_TICK_HZ = 83.33e6` and `PIO_TICK_NS = 12`. Alternatively, if the counter programs are rewritten to be 1-instruction loops (just `jmp x--`), 4ns resolution is achievable but would change all the existing tested code.

**Recommendation:** Use 12ns/tick (83.33MHz) as-is. The existing code works. Update the RFC constants. The practical impact is minimal — 12ns is still far below the target accuracy.

### 3.2. PIO Counter Architecture on GM

**RFC states (§4.2):** GM needs SM0 (1PPS), SM1 (INTn), SM2 (counter). All share a common counter.

**Reality:** The GM uses TWO PIO blocks with INDEPENDENT counters:
- **PIO0:** SM0 = counter_with_pin_trigger (1PPS), SM1 = pps_edge_with_pin_signal
- **PIO1:** SM0 = counter_simple (HW timestamp), SM1 = int_timestamp (INTn capture)

The 1PPS counter and the INTn counter are in different PIO blocks with independent X registers. They cannot be directly subtracted. The existing code converts via clock models (GPS time interpolation), not by cross-SM comparison.

**Implication for LTSP:** The RFC's assumption that T_pio_1pps and T_pio_tx are in the same counter domain is incorrect for this hardware. The GM must convert between domains. The existing `get_gps_time_ns()` function already solves this by converting everything to GPS nanoseconds via system timer interpolation.

**Recommendation:** For the GM, use `get_gps_time_ns()` as the common time base. The 1PPS counter provides crystal calibration, the HW timestamp counter provides TX timing, and GPS time provides the bridge. The PDU's `T_epoch`, `a0`, `Last_1PPS_Count` should all be expressed in GPS nanoseconds (or a common counter), not in raw PIO ticks from different SMs.

**Alternative:** Move INTn capture to PIO0 (same block as 1PPS counter) so they share the same X register. This would require PIO0 SM2 for INTn and SM3 is not available (only 4 SMs). Actually PIO0 SM0 and SM1 are used by GPS discipline. SM2 could run `int_timestamp`. But `counter_simple` would need to share the X register with `counter_with_pin_trigger`, which isn't how they work — each SM has its own X.

**Practical Resolution:** The cross-SM comparison issue is inherent to the RP2040. The existing approach of converting via `get_gps_time_ns()` is sound and should be used by LTSP. The RFC's T_pio_tx should be a GPS-nanosecond timestamp derived from HW capture + conversion, not a raw PIO counter value.

### 3.3. W5500 INTn Behavior

**RFC assumes:** INTn asserts (falls) on TX complete and RX arrival. Single edge per event.

**Reality:** The existing code reveals:
- INTn is active-low, level-triggered (stays low until interrupt register is cleared)
- Multiple events can overlap (RX arriving while TX completing)
- The PIO `int_timestamp` program detects HIGH-to-LOW transitions, so it naturally handles only edges
- The circular buffer approach (`hw_timestamp.c`) with direction filtering is necessary because a single INTn assertion can correspond to either RX or TX
- Spurious/duplicate detections are handled by age-based expiry (100ms max)

**Implication:** The deferred timestamp mechanism must be robust to INTn ambiguity. The existing direction-filtered correlation is a good solution.

### 3.4. SPI Speed

**Implementation guide states:** SPI0 at 33MHz.

**Reality:** Existing code uses 40MHz SPI (`w5500_simple.c`). This is faster and working. Use 40MHz.

### 3.5. GPIO Pin Assignments

**Implementation guide states:** SPI0_MISO = GPIO 16, W5500_INT = GPIO 20, GPS_1PPS = GPIO 21.

**Reality (from existing code):**
- SPI0_MISO = GPIO 16 (matches)
- SPI0_CS = GPIO 17 (matches)
- SPI0_SCK = GPIO 18 (matches)
- SPI0_MOSI = GPIO 19 (matches)
- W5500_RST = GPIO 20 (impl guide says INT here!)
- W5500_INT = GPIO 21 (impl guide says GPS_1PPS here!)
- GPS_1PPS = GPIO 2 (NOT GPIO 21!)
- Internal trigger = GPIO 22 (not in impl guide)

**Recommendation:** Use existing pin assignments. They are tested and working. The implementation guide's pin assignments are wrong for the existing hardware.

### 3.6. Counter Wrapping Convention

**RFC states:** X counts DOWN. Implementations must account for this.

**Reality:** Confirmed. The existing code handles this correctly:
- `discipline_v3.c:184-189`: Explicit up/down comparison for elapsed ticks
- `hw_timestamp.c:139-151`: `counter_before > counter_after` = normal, else wraparound
- Counter values are inverted: higher X = earlier time (counts toward 0)

---

## 4. Time Domain Summary (Existing Code)

| Domain | Clock Rate | Resolution | Location | Used For |
|--------|-----------|------------|----------|----------|
| PIO0 X counter (GM) | 250MHz / 3-cycle loop | 12ns | `gps_discipline_v2.pio` SM0 | GPS 1PPS interval measurement |
| PIO1 X counter (GM) | 250MHz / 3-cycle loop | 12ns | `counter_simple.pio` SM0 | W5500 INTn TX/RX HW timestamps |
| PIO0 X counter (Slave) | 250MHz / 3-cycle loop | 12ns | `counter_simple.pio` SM0 | W5500 INTn RX HW timestamps |
| System Timer | 1MHz | 1µs | `time_us_64()` | GPS interpolation, SPI timing |
| GPS Nanoseconds | Derived | 1ns (calculated) | `get_gps_time_ns()` | Absolute time reference |
| PTP Clock (Slave) | Derived | 1ns (calculated) | `get_ptp_time_ns()` | Master time estimate |

---

## 5. Critical Architectural Decision

The RFC envisions all timestamps in a single PIO counter domain per device. The
existing hardware uses separate PIO blocks for 1PPS and INTn on the GM, making
direct PIO-tick comparison impossible.

**Options:**

**A. Restructure PIO to share counter (risky)**
Move INTn capture into PIO0 alongside 1PPS. Would require a new PIO program
combining counter, 1PPS trigger, and INTn detection in 3 SMs on one PIO block.
Risk: may break working GPS discipline code.

**B. Use GPS nanoseconds as common domain (recommended)**
Keep existing PIO architecture. Convert all timestamps to GPS nanoseconds using
`get_gps_time_ns()` and the HW timestamp correlation. The PDU carries GPS-domain
timestamps. The receiver uses these directly.

This means PDU fields like `Prev_Tx_Timestamp`, `Model_Epoch`, `Last_1PPS_Count`
are in GPS nanoseconds (int64_t), not raw PIO ticks. The `a0` field becomes the
GPS-to-PIO offset, and `Last_1PPS_Interval` stays in PIO ticks (single-SM
measurement, no cross-SM issue).

**Recommendation:** Option B. It's simpler, reuses all existing code, and provides
the receiver with directly usable timestamps. The only loss is that GPS nanosecond
timestamps have ~1µs jitter (from System Timer interpolation in `get_gps_time_ns()`),
but this is already the practical precision floor for the GM's absolute time
reference.

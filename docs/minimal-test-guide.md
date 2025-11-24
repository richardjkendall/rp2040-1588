# Minimal Test Guide - GPS Discipline Precision

## Purpose

Test the PIO + DMA GPS discipline architecture in isolation, without printf/network/NMEA overhead, to determine:
1. Does the 250 MHz counter + DMA fundamentally work?
2. Is the 100 PPS output GPS-disciplined with good precision?
3. Is DMA bus usage tolerable without printf/network?
4. Does the system run stably or crash?

## Binary Location

`build/grandmaster/grandmaster_minimal_test.uf2`

## What's Different from Full Version

**REMOVED (to eliminate bus contention):**
- All printf in main loop
- Network polling (`network_poll()`)
- GPS NMEA parsing (`gps_process()`)
- PTP processing
- Status updates

**KEPT (essential for test):**
- PIO0 SM0: 250 MHz counter
- PIO0 SM1: GPS PPS capture
- DMA: Counter → PPS capture (continuous at 250 MHz)
- GPS discipline IRQ handler
- 100 PPS output generation
- Startup printf (shows system initialized, then goes silent)

## GPIO Test Points

Connect oscilloscope to these pins:

### GPIO 3 (LED_OUTPUT_PIN) - 100 PPS Output
**What to measure:**
- Frequency: Should be 100 Hz (10.000ms period)
- Precision: Measure jitter over 100+ cycles
- Stability: Does period drift over time?
- Lock behavior: Does precision improve after GPS lock?

**Expected:**
- Before GPS lock: 100 Hz but may drift
- After GPS lock: 100.000 Hz ± sub-microsecond jitter

### GPIO 4 (DEBUG_PPS_PIN) - GPS PPS IRQ Toggle
**What to measure:**
- Frequency: Should be 0.5 Hz (toggles every GPS PPS)
- Period: 2 seconds between toggles
- Confirms: PPS IRQ handler is firing

**Expected:**
- Square wave at 0.5 Hz
- Should appear within 10 seconds of startup (GPS cold start)
- Stops toggling if GPS loses lock

### GPIO 2 (GPS_PPS_PIN) - GPS Module Output
**What to measure:**
- Frequency: 1 Hz (pulses every second)
- Pulse width: ~100ms typical for GPS modules
- Timing: Compare with GPIO 4 (should be synchronized)

**Expected:**
- 1 Hz pulses from GPS module
- GPIO 4 should toggle on rising edge

### GPIO 5 (DEBUG_LOCK_PIN) - Discipline Lock Status
**What to observe:**
- Low (0V): Not locked to GPS
- High (3.3V): Discipline locked

**Expected timeline:**
1. Startup: Low (no GPS fix yet)
2. After GPS acquires fix (10-60 seconds): Still low (discipline converging)
3. After 10+ good PPS measurements: Goes high (locked)
4. Stays high indefinitely (unless GPS loses fix)

## Serial Output (Startup Only)

Connect to USB serial at 115200 baud. You should see:

```
=== MINIMAL TEST - GPS Discipline ===
System clock: 250 MHz

Debug GPIOs initialized:
  GPIO 3: 100 PPS output
  GPIO 4: GPS PPS IRQ toggle
  GPIO 5: Lock status

GPS ready (TX=0 RX=1 PPS=2)
Initializing GPS discipline (PIO + DMA architecture)...
  PIO programs loaded (counter @ X, pps @ Y)
  Counter SM initialized (250 MHz)
  PPS capture SM initialized (GPIO 2)
  DMA configured (chan 0, SM0 → SM1)
  IRQ handler installed
GPS discipline ready (4ns resolution, software correction)

*** ENTERING MINIMAL TEST LOOP ***
*** NO MORE PRINTF - WATCH GPIO SIGNALS ***
```

After this, **all printf stops**. System continues running but is silent.

## Test Procedure

### Setup
1. Flash `grandmaster_minimal_test.uf2` to Pico
2. Connect GPS module:
   - GPS TX → GPIO 1 (Pico RX)
   - GPS RX → GPIO 0 (Pico TX)
   - GPS PPS → GPIO 2
   - GPS VCC/GND → Pico power
3. Connect scope probes:
   - CH1: GPIO 3 (100 PPS)
   - CH2: GPIO 4 (PPS IRQ toggle)
   - CH3: GPIO 5 (Lock status)
   - Optional CH4: GPIO 2 (GPS PPS input)
4. Place GPS antenna with clear sky view
5. Connect USB for serial (optional - just shows startup)

### Observation Timeline

**T=0s (Power On):**
- Serial shows initialization messages
- All GPIOs start low
- No 100 PPS yet (needs first GPS PPS)

**T=10-60s (GPS Cold Start):**
- GPS acquires satellites
- GPIO 2: GPS PPS starts (1 Hz pulses)
- GPIO 4: Starts toggling (0.5 Hz) - confirms IRQ working
- GPIO 3: 100 PPS starts (10ms period)
- GPIO 5: Still low (not locked yet)

**T=60-120s (Discipline Convergence):**
- GPIO 3: 100 PPS running continuously
- Measure jitter on scope - should be improving
- GPIO 5: May go high (locked) after 10+ good PPS measurements

**T=120s+ (Steady State):**
- GPIO 3: 100 PPS locked to GPS (sub-microsecond jitter)
- GPIO 4: Toggling steadily at 0.5 Hz
- GPIO 5: High (locked)
- System should run indefinitely without crashing

### Measurements to Record

**100 PPS Precision (GPIO 3):**
1. Set scope to measure period statistics over 100 samples
2. Record: Mean, Min, Max, Std Dev
3. Expected if locked: 10.000000ms ± <1μs

**System Stability:**
1. Let run for 10+ minutes
2. Does it crash? (GPIO activity stops)
3. Does it stay locked? (GPIO 5 stays high)
4. Does 100 PPS drift? (period changes over time)

**DMA Bus Impact:**
1. Does the system run at all? (If DMA monopolizes bus, it may not)
2. Is GPIO toggling smooth or glitchy?

## Success Criteria

✅ **PASS** if:
- System runs for 10+ minutes without crashing
- GPIO 4 toggles at 0.5 Hz (PPS IRQ working)
- GPIO 3 outputs 100 PPS with <10μs jitter when locked
- GPIO 5 goes high within 2 minutes (locks to GPS)

❌ **FAIL** if:
- System crashes/hangs within 10 minutes
- No GPIO activity (DMA completely starving CPU)
- 100 PPS jitter >100μs (discipline not working)
- Never locks (GPIO 5 stays low forever)

## Interpretation

### If Test PASSES
- Architecture fundamentally works
- Precision is good (4ns counter resolution paying off)
- DMA at 250 MHz is tolerable without printf/network
- **Next step:** Optimize printf/network or reduce DMA rate

### If Test FAILS (System Hangs)
- DMA at 250 MHz monopolizes bus even without printf
- **Next step:** Implement Option 2 (PIO IRQ flag sync) to reduce DMA rate
- Or: Push counter every 100 cycles instead of every cycle

### If Test FAILS (Poor Precision)
- Discipline algorithm needs tuning (Kp/Ki gains)
- Or: Counter wraparound handling bug
- Or: FIFO snapshot not working as designed

### If Test FAILS (Never Locks)
- GPS not getting valid fix (check antenna placement)
- Or: PPS capture not working (check GPIO 4 - should toggle)
- Or: Lock thresholds too tight (need relaxing)

## Debug Tips

**No GPIO activity at all:**
- Check serial output - did initialization succeed?
- Check power - is Pico running?
- Check GPS connection - is GPS powered?

**GPIO 4 not toggling:**
- GPS PPS not arriving (check GPS module status LED)
- PIO IRQ not firing (check PIO program loaded correctly)
- GPIO 2 input not connected

**GPIO 3 (100 PPS) not running:**
- Depends on first GPS PPS (needs GPIO 4 working first)
- Check `output_100pps_enabled` flag in discipline_v2.c

**GPIO 5 never goes high:**
- May need longer convergence time (try 5+ minutes)
- Check lock thresholds in discipline_v2.c (1μs phase, 100 PPB freq)
- GPS fix may be marginal (low satellite count)

## Next Steps After Test

**If successful, next improvements:**
1. Add back GPS NMEA parsing (should be low overhead)
2. Add back network polling (may reveal bus contention)
3. If network causes issues: Reduce DMA rate (Option 2)
4. Add back printf with rate limiting (max 1 Hz)
5. Re-enable full PTP grandmaster functionality

**If unsuccessful:**
- Implement PIO IRQ flag synchronization (Option 2)
- Reduces DMA rate from 250M/s to ~1M/s (250x less bus usage)
- Maintains <40ns PPS capture precision

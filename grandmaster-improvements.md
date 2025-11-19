# Grandmaster Implementation Improvements

This document outlines three key improvements for the PTP grandmaster firmware to enhance its accuracy, stability, and correctness.

## 1. Reduce Timestamp Latency with `__time_critical_func`

**Goal:** Reduce the latency between the physical PPS signal edge and the software timestamp capture.

**Problem:** The `gps_pps_irq_handler` function is executed from flash memory, which has inherent latency. This contributes to the overall phase offset.

**Solution:** Use the `__time_critical_func` attribute from the Pico SDK to instruct the compiler to place the function in the faster SRAM.

### Instructions

In `grandmaster/gps.c`, add the `__time_critical_func` attribute to the function definition of `gps_pps_irq_handler`.

**File:** `grandmaster/gps.c`

```c
// PIO IRQ handler - called on GPS PPS rising edge
static void __time_critical_func(gps_pps_irq_handler)() {
    // Check if our PIO triggered the interrupt
    if (pio_interrupt_get(gps_state.pio, 0)) {
        // Clear the interrupt
        pio_interrupt_clear(gps_state.pio, 0);

        // Capture timestamp immediately
        gps_state.last_pps_timestamp_us = time_us_64();
        gps_state.pps_available = true;
    }
}
```

---

## 2. Prevent Data Tearing with Spin-Locks

**Goal:** Ensure data shared between Core 0 and Core 1 is read and written atomically, preventing corruption.

**Problem:** The `core1_stats_t` struct is written by Core 1 and read by Core 0. Reading a 64-bit value like `continuous_time_ns` is not an atomic operation on a 32-bit processor. Core 0 could read a partially-updated, corrupt value if the Core 1 interrupt occurs in the middle of the read.

**Solution:** Use a spin-lock to protect both read and write accesses to the shared data structure.

### Instructions

**Step 2a: Add Spin-Lock to Shared State**

In `grandmaster/shared_state.h`, include the spin-lock header and add a `spin_lock_t` instance to the `core1_stats_t` struct.

**File:** `grandmaster/shared_state.h`

```c
#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/sync.h" // <-- Add this include

typedef struct {
    // Spin-lock for protecting access
    spin_lock_t *lock; // <-- Add this member

    // Discipline statistics
    volatile uint32_t pps_count;
    // ... (rest of the struct members) ...
    volatile uint64_t last_update_us;
} core1_stats_t;

// ...
#endif
```

**Step 2b: Initialize the Spin-Lock**

In `grandmaster/discipline_core.c`, initialize the spin-lock before launching Core 1.

**File:** `grandmaster/discipline_core.c` (inside `discipline_init` or a similar setup function)

```c
// In your initialization function before multicore_launch_core1(...)
static spin_lock_t *shared_data_lock;

void discipline_init(/*...args...*/) {
    // ...
    // Claim a free spinlock
    int lock_num = spin_lock_claim_unused(true);
    shared_data_lock = spin_lock_init((uint)lock_num);
    core1_stats.lock = shared_data_lock;
    // ...
}
```

**Step 2c: Protect Writes on Core 1**

In `grandmaster/discipline_core.c`, wrap the updates to `core1_stats` with the spin-lock.

**File:** `grandmaster/discipline_core.c` (inside `core1_entry`)

```c
// In the main loop of core1_entry()
void core1_entry() {
    // ...
    while (true) {
        // ... (do calculations)

        // --- Protect shared state update ---
        uint32_t lock = spin_lock_blocking(core1_stats.lock);

        core1_stats.disciplined_time_ns = disciplined_clock.nanoseconds;
        core1_stats.continuous_time_ns = (ptp_seconds * 1000000000ULL) + disciplined_clock.nanoseconds;
        core1_stats.last_update_us = time_us_64();
        // ... (update other stats)

        spin_unlock(core1_stats.lock, lock);
        // --- End of protected section ---
    }
}
```

**Step 2d: Protect Reads on Core 0**

In `grandmaster/ptp_grandmaster.c` (and `main.c` for printing), wrap reads with the same lock.

**File:** `grandmaster/ptp_grandmaster.c` (inside `send_sync_and_followup`)

```c
static void send_sync_and_followup(void) {
    uint64_t timestamp_ns;
    uint64_t precise_timestamp_ns;

    // --- Protect shared state read ---
    uint32_t lock = spin_lock_blocking(core1_stats.lock);
    timestamp_ns = core1_stats.continuous_time_ns;
    spin_unlock(core1_stats.lock, lock);
    // --- End of protected section ---

    // ... build and send sync message ...

    // --- Protect shared state read ---
    lock = spin_lock_blocking(core1_stats.lock);
    precise_timestamp_ns = core1_stats.continuous_time_ns;
    spin_unlock(core1_stats.lock, lock);
    // --- End of protected section ---

    // ... build and send followup message ...
}
```

---

## 3. Improve PTP Message Scheduling

**Goal:** Schedule the transmission of PTP `Sync` messages based on the disciplined clock, not a free-running timer.

**Problem:** `ptp_grandmaster_process()` uses `to_ms_since_boot()`, which is based on the RP2040's internal oscillator. This timer drifts relative to the GPS-disciplined clock, meaning `Sync` messages are not sent at a true 1-second interval.

**Solution:** Trigger `Sync` message transmission from the timing-critical core (Core 1) when the disciplined clock passes a one-second boundary.

### Instructions

This is a more architectural change. One effective way is to use a flag in the shared state.

**Step 3a: Add a Flag to Shared State**

Add a boolean flag to the `core1_stats_t` struct.

**File:** `grandmaster/shared_state.h`

```c
typedef struct {
    spin_lock_t *lock;
    volatile bool trigger_ptp_sync; // <-- Add this flag
    // ...
} core1_stats_t;
```

**Step 3b: Trigger the Flag on Core 1**

In `grandmaster/discipline_core.c`, when the disciplined clock rolls over the 1-second mark, set the flag.

**File:** `grandmaster/discipline_core.c`

```c
// In core1_entry, after the disciplined time is updated
// This logic assumes you know when the second has rolled over.
// A common way is to check the nanosecond part of the time.

// Example:
uint64_t old_ns = core1_stats.disciplined_time_ns;
// ... update time ...
uint64_t new_ns = disciplined_clock.nanoseconds;

if (old_ns > 900000000 && new_ns < 100000000) { // Rolled over the second
    uint32_t lock = spin_lock_blocking(core1_stats.lock);
    core1_stats.trigger_ptp_sync = true;
    spin_unlock(core1_stats.lock, lock);
}
```

**Step 3c: Act on the Flag on Core 0**

In `grandmaster/main.c`, change the main loop to check for the flag instead of using a timer.

**File:** `grandmaster/main.c` (in `main()`'s `while(true)` loop)

```c
int main() {
    // ... setup ...
    while (true) {
        // ... other tasks ...

        // Check if Core 1 has requested a PTP sync message
        bool should_send_sync = false;
        uint32_t lock = spin_lock_blocking(core1_stats.lock);
        if (core1_stats.trigger_ptp_sync) {
            should_send_sync = true;
            core1_stats.trigger_ptp_sync = false; // Clear the flag
        }
        spin_unlock(core1_stats.lock, lock);

        if (should_send_sync) {
            #if ENABLE_ANNOUNCE_MESSAGES
            send_announce_message();
            #endif
            send_sync_and_followup(); // This function should now be exposed from ptp_grandmaster.c
        }

        // The ptp_grandmaster_process() function should be refactored.
        // The message sending logic moves here, while other periodic tasks
        // like cleaning up stale slaves can remain in ptp_grandmaster_process().
        ptp_grandmaster_process(); // This will now just handle non-message tasks
    }
}
```
*Note: This change requires refactoring. The call to `send_sync_and_followup()` needs to be moved from `ptp_grandmaster_process()` to the main loop as shown, and the function might need to be exposed via its header if it isn't already.*

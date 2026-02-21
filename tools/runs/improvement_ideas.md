# LTSP Receiver — Improvement Ideas

**Baseline (2026-02-20):** Phase mean -1.4 µs, sigma 6.5 µs, P-P 35 µs, drift ~0.
Network jitter sigma ~31 µs, alpha=0.05 IIR filter. Two 30-min runs confirm repeatability.

---

## 1. ~~Outlier rejection before IIR filter~~ — FAILED

**Status:** Tested 2026-02-20. Three approaches tried, all failed. **Do not revisit.**

**What we tried:**
- **Coasting** (skip IIR update when offset_ns > threshold): During convergence, offset_ns
  is large because d_min hasn't settled — all packets rejected, filter never converges.
  Adding a settling guard still rejected 78% of packets.
- **Error clamping at 2σ** (cap `gps_now - predicted` at ±62 µs): The IIR error signal
  inherently has ~31 µs sigma spread. Clamping at 2σ clips 48% of normal packets,
  biasing the estimate and preventing convergence.
- **Error clamping at 4σ** (±125 µs): Still clipped 78% on cold boot. After settling,
  worked briefly (~6.8 µs sigma) but suffered catastrophic failure when d_min got a
  -16.6 second spike, corrupting offset_ns for all subsequent packets.

**Key lesson:** The alpha=0.05 IIR already handles jitter correctly — each sample
contributes only 5%. Outlier rejection at the filter input is counterproductive.

---

## 2. ~~Lower alpha (0.02)~~ — TESTED, IMPROVED

**Status:** Tested 2026-02-20. Confirmed improvement, superseded by #3.

**Result:** Settled sigma 5.4 µs (vs 6.5 µs baseline). P-P 35.6 µs.
Short-term windows (60s) reached 3.2 µs sigma. Theory predicted ~3 µs but
low-frequency wander (~5 µs over 2-min windows) limits the long-term average.
Run: `tools/runs/20260220_200753`

**Conclusion:** Lower alpha helps but hits a floor around 4–5 µs due to slow wander.
Convergence too slow for cold boot (~300s). Adaptive alpha (#3) is strictly better.

---

## 3. ~~Adaptive alpha~~ — TESTED, CURRENT BEST

**Status:** Tested 2026-02-20. **Currently deployed.** Best performance achieved.

**Implementation:** Exponential decay from alpha=0.2 to alpha=0.02, tau=30 packets.
```c
#define PPS_ALPHA_INIT   0.2
#define PPS_ALPHA_FINAL  0.02
#define PPS_ALPHA_TAU    30.0
alpha = PPS_ALPHA_FINAL + (PPS_ALPHA_INIT - PPS_ALPHA_FINAL) * exp(-count / TAU);
```

**Result (30-min, fresh boot):** Run `tools/runs/20260220_204329`
- Settled (t>300s): sigma 4.6 µs, P-P 26.5 µs, mean -2.5 µs, drift +0.002 µs/s
- Short-term windows: 2.8–5.1 µs sigma (2-min windows)
- Zero packet loss, zero scope glitches
- Convergence: ~300s from boot (limited by regression needing ~65 packets, not alpha)

| Metric       | Adaptive (0.2→0.02) | Fixed 0.02 | Baseline (0.05) |
|-------------|---------------------|------------|-----------------|
| Sigma       | **4.6 µs**          | 5.4 µs     | 6.5 µs          |
| P-P         | **26.5 µs**         | 35.6 µs    | 35 µs           |
| Convergence | ~300s               | ~400s      | ~60s            |

**Remaining limitation:** Low-frequency wander (~5 µs over 2-min windows) sets
the floor. Source unknown — likely temperature, GPS jitter, or W5500 latency variation.

---

## 4. Median pre-filter — COULD TRY

Keep a sliding window of e.g. 5 `gps_now` samples and take the median
before feeding to the IIR filter. Robust to outliers without needing
an explicit threshold.

**Why it might help beyond #3:** Could reduce the low-frequency wander that
limits sigma to ~4.6 µs. Median rejects occasional large excursions that
the IIR smooths but doesn't eliminate.

**Tradeoff:** Adds 5-sample latency to filter response. More memory/code.
At 1 Hz packet rate, 5-sample latency = 5 seconds.

---

## 5. Higher packet rate — FUTURE

Currently 1 Hz. At 10 Hz, the filter gets 10x more samples per unit time.
Could use lower alpha without convergence penalty, or converge faster at
current alpha. Would also reduce wander by averaging more samples per second.

**Tradeoff:** More SPI/Ethernet traffic, more CPU on both GM and RX.
GM needs GPS-disciplined timing at higher rate. Significant GM firmware changes.

**Defer until:** Alpha and filter improvements are exhausted.

---

## 6. ~~Two-way delay measurement~~ — NOT PURSUING

User decided not to pursue this approach.

# Building a GPS-Disciplined PTP/IEEE 1588v2 System with the Raspberry Pi Pico

## Introduction

- Brief overview: What is PTP/IEEE 1588 and why it matters
- The challenge: Precision timing typically requires expensive hardware timestamping
- The experiment: Can a $4 microcontroller achieve useful PTP performance?
- Spoiler: Yes, with some clever engineering

## Project Goals

- Demonstrate RP2040 capabilities for precision timing applications
- Build an experimental platform to compare different network technologies
- Create an independent GPS-calibrated measurement system for validation
- Explore the limits of software-only timestamping on embedded systems
- Document what's possible without dedicated hardware timestamping PHYs

## System Architecture

### Three-Device Setup

1. **Grandmaster (GM)**
   - GPS-disciplined clock (NEO-6M GPS module)
   - W5500 Ethernet interface
   - Generates GPS-synchronized 1PPS output
   - Serves as PTP time source

2. **Slave**
   - W5500 Ethernet interface
   - PTP discipline loop for crystal correction
   - Generates PTP-synchronized 1PPS output
   - Tracks Grandmaster time

3. **Independent Measurement Device**
   - GPS-disciplined for crystal calibration
   - Measures phase offset between GM and Slave 1PPS signals
   - Provides ground truth verification
   - WiFi telemetry capability (optional)

### Why Independent Measurement Matters

- PTP implementations can lie to themselves
- Self-reported accuracy vs actual accuracy
- GPS-based measurement provides objective validation
- Can measure path delay asymmetry

## Technical Deep Dive

### GPS Discipline System

- Using RP2040 PIO for precise edge capture
- Crystal characterization and frequency correction
- Achieving sub-ppm accuracy with consumer GPS modules
- Scale factor calculation for tick-to-nanosecond conversion

### PTP Implementation on W5500

- Software timestamping challenges
- Interrupt latency considerations
- Ethernet frame processing overhead
- Why we don't have hardware timestamps (and that's okay)

### Phase Offset Measurement

- Bidirectional measurement technique
- PIO-based edge-to-edge timing
- Handling pulse overlap and edge detection
- Why pulse width matters (even when measuring rising edges)

### PIO Magic: The Secret Sauce

- State machines for 1PPS generation
- Edge capture and timestamping
- Tick counting between edges
- Clock dividers and timing resolution (~36ns per measurement tick)

## Implementation Challenges & Solutions

### Challenge 1: Edge Detection with Polling

**Problem**: `jmp pin` checks pin state, not edges
- Wide pulses (10ms) caused overlapping signals
- PIO saw "pin already HIGH" instead of new edge
- Resulted in incorrect measurements

**Solution**: Reduced pulse width to 10µs
- Guarantees non-overlapping for offsets > 10µs
- Still visible on oscilloscope
- Clean edge detection

### Challenge 2: Loop Overhead in Tick Counting

**Problem**: Initial conversion assumed 12ns per tick
- Actually 36ns due to 3-cycle loop overhead
- Measurements were off by 3x

**Solution**: Accounted for actual PIO instruction timing
- Measured loop cycles: `jmp x--` + `jmp pin` + `jmp loop` = 3 cycles
- Corrected conversion factor: ticks × 36ns

### Challenge 3: Invalid Measurements Polluting Statistics

**Problem**: Occasional edge detection failures
- Near-zero measurements when edge missed
- Near-1-second measurements from wrong interval
- Statistics showed huge standard deviation

**Solution**: Measurement validation
- Filter: only accept 1µs to 10ms offsets
- Valid measurements: consistent ~130µs
- Clean statistics: ±3µs variation

## Performance Results

### Achieved Performance

- **Phase offset**: 133µs (Slave leading Grandmaster)
- **Variation**: ±5µs (127-138µs range)
- **Long-term stability**: Sub-microsecond EMA standard deviation
- **GPS crystal discipline**: ±0.000 ppm
- **Consistency**: 100% measurements show same direction (Slave first)

### Context: How Good Is This?

| Implementation Type | Typical Accuracy |
|---------------------|------------------|
| Hardware-timestamped PTP | 10-100ns |
| Software PTP on fast CPUs | 1-10µs |
| **RP2040 + W5500 (this project)** | **~130µs** |
| NTP over internet | 10-100ms |

### What the Numbers Mean

- **Static offset (133µs)**: Likely systematic delays (interrupt latency, processing)
- **Low jitter (±5µs)**: Excellent for software timestamping
- **Sub-µs EMA noise**: Shows crystal discipline is working beautifully
- **Consistent direction**: No mode hopping or instability

## Lessons Learned

### What Worked Well

1. **PIO is incredibly powerful** for precise timing tasks
2. **GPS discipline** provides excellent crystal calibration
3. **Independent measurement** is invaluable for validation
4. **RP2040 dual-core** architecture allows separation of concerns

### What Could Be Better

1. **Software timestamping** has fundamental limitations
2. **W5500 interrupt latency** adds overhead
3. **Static calibration offset** suggests asymmetric processing delays
4. **Need hardware timestamps** for sub-microsecond PTP accuracy

### Insights

- The 133µs offset appears to be a static calibration error
- Could potentially compensate in software if consistent
- Most impressive: sub-microsecond jitter shows the system is stable
- GPS-based measurement device is the real MVP

## Future Experiments

### Network Technology Comparison

- ✅ WiFi (completed - initial implementation)
- ✅ Ethernet (W5500 - current results)
- 🔜 GPON (planned)
- 🔜 Direct fiber connection
- 🔜 Multi-hop switched networks

### Potential Improvements

1. **Path delay characterization**
   - Separate measurement of GM→Slave vs Slave→GM delays
   - Asymmetry compensation

2. **Interrupt optimization**
   - Faster timestamp capture closer to MAC
   - Reduced software processing overhead

3. **PIO-based timestamping**
   - Could PIO timestamp at MII/RMII interface?
   - Bypass software interrupt latency

4. **Hardware timestamping PHY**
   - Compare RP2040 with proper timestamping hardware
   - DP83640, LAN8814, or similar

5. **Two-step PTP**
   - Might reduce some software delays
   - More accurate timestamp capture

### GPON-Specific Tests

- Characterize upstream/downstream asymmetry
- Measure impact of dynamic bandwidth allocation
- Test under varying network load
- Compare with Ethernet baseline

## Code Architecture

### Repository Structure
```
pico-gps-1588/
├── grandmaster/          # GPS-disciplined GM implementation
├── slave/                # PTP slave with discipline loop
├── measurement/          # Independent measurement device
├── common/               # Shared discipline and PTP code
└── pio/                  # PIO state machine programs
```

### Key Technologies

- **Language**: C
- **SDK**: Pico SDK
- **Hardware**: RP2040, W5500, NEO-6M GPS
- **Timing**: PIO state machines
- **Network**: Custom PTP stack for W5500

## Bill of Materials

| Component | Purpose | Cost |
|-----------|---------|------|
| Raspberry Pi Pico | Microcontroller (x3) | ~$12 |
| W5500-EVB-Pico | Ethernet interface (x2) | ~$20 |
| Pico W | WiFi measurement device | ~$6 |
| NEO-6M GPS Module | Time reference (x1) | ~$10 |
| **Total** | | **~$48** |

*Compared to commercial PTP grandmaster: $5,000-$50,000*

## Conclusions

### What This Project Demonstrates

1. **RP2040 is capable** of precision timing applications
2. **Software timestamping** can achieve microsecond-level performance
3. **GPS discipline** works excellently on embedded systems
4. **Independent measurement** is crucial for validation
5. **$50 experiment** can reveal network timing characteristics

### Realistic Assessment

- **Not a replacement** for hardware-timestamped PTP
- **Excellent research/education platform**
- **Good enough** for many embedded timing applications
- **Outstanding** for network characterization experiments

### The Real Achievement

Not achieving nanosecond accuracy (we didn't), but:
- Demonstrating what's possible with clever engineering
- Creating an independent measurement methodology
- Building a platform for network timing research
- Documenting the journey and learnings

## Resources

- [Project Repository](link-to-repo)
- [IEEE 1588 Standard](https://standards.ieee.org/standard/1588-2019.html)
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [W5500 Datasheet](https://www.wiznet.io/product-item/w5500/)

## Acknowledgments

- Thanks to the Raspberry Pi Foundation for the excellent documentation
- PTP implementation inspired by various open-source projects
- GPS discipline techniques from amateur radio timing community

---

*This project is open source and available on GitHub. Contributions, suggestions, and experiments welcome!*

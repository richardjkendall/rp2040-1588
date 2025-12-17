package main

import (
	"math"
	"sync"
	"time"
)

// Measurement from Pico W
type Measurement struct {
	Seq            uint64  `json:"seq"`
	TimestampUs    uint64  `json:"timestamp_us"`
	PhaseNs        float64 `json:"phase_ns"`
	GmToSlaveNs    float64 `json:"gm_to_slave_ns"`
	SlaveToGmNs    float64 `json:"slave_to_gm_ns"`
	ScaleFactor    float64 `json:"scale_factor"`
	GmFirst        bool    `json:"gm_first"`
	CrystalErrorNs int32   `json:"crystal_error_ns"`
}

// TimestampedMeasurement includes server receive time
type TimestampedMeasurement struct {
	Measurement
	ReceivedAt time.Time
}

// Batch from Pico W
type Batch struct {
	Device       string        `json:"device"`
	BatchSeq     uint32        `json:"batch_seq"`
	Count        int           `json:"count"`
	Measurements []Measurement `json:"measurements"`
	Stats        BufferStats   `json:"stats"`
}

type BufferStats struct {
	BufferAvailable uint32 `json:"buffer_available"`
	BufferDropped   uint32 `json:"buffer_dropped"`
}

// WindowedStats holds statistics for a time window
type WindowedStats struct {
	Count  uint64
	Mean   float64
	StdDev float64
	Min    float64
	Max    float64
}

// Statistics we calculate
type Statistics struct {
	mu sync.RWMutex

	// All-time statistics
	SampleCount     uint64
	Mean            float64
	M2              float64 // For Welford's algorithm
	Min             float64
	Max             float64
	LastCrystalErr  int32
	LastScaleFactor float64
	BatchesReceived uint32
	LastUpdate      time.Time
	BufferDropped   uint32
	GmFirstCount    uint64
	SlaveFirstCount uint64
	OutliersFiltered uint64 // Count of filtered outlier measurements

	// Rolling window measurements (keep last 30 minutes)
	measurements []TimestampedMeasurement

	// Broadcast channel for updates
	updates chan *StatsSnapshot
}

// MTIEPoint represents a single MTIE data point
type MTIEPoint struct {
	Tau  float64 `json:"tau"`  // Observation interval in seconds
	MTIE float64 `json:"mtie"` // Maximum time interval error in nanoseconds
}

// AllanPoint represents a single Allan deviation data point
type AllanPoint struct {
	Tau   float64 `json:"tau"`   // Averaging time in seconds
	ADev  float64 `json:"adev"`  // Allan deviation in nanoseconds
	Count int     `json:"count"` // Number of samples used
}

// TIEPoint represents a time-series data point for TIE plotting
type TIEPoint struct {
	Time    float64 `json:"time"`    // Time in seconds since first measurement
	PhaseNs float64 `json:"phase_ns"` // Phase offset in nanoseconds
}

// HistogramData contains histogram bins and metadata
type HistogramData struct {
	Bins    []int   `json:"bins"`     // Histogram bin counts
	MinVal  float64 `json:"min_val"`  // Minimum value (start of first bin)
	BinSize float64 `json:"bin_size"` // Size of each bin in nanoseconds
}

type StatsSnapshot struct {
	// All-time statistics
	SampleCount      uint64    `json:"sample_count"`
	Mean             float64   `json:"mean"`
	StdDev           float64   `json:"stddev"`
	Min              float64   `json:"min"`
	Max              float64   `json:"max"`
	LastCrystalErr   int32     `json:"last_crystal_err"`
	LastScaleFactor  float64   `json:"last_scale_factor"`
	BatchesReceived  uint32    `json:"batches_received"`
	LastUpdate       time.Time `json:"last_update"`
	BufferDropped    uint32    `json:"buffer_dropped"`
	GmFirstCount     uint64    `json:"gm_first_count"`
	SlaveFirstCount  uint64    `json:"slave_first_count"`
	OutliersFiltered uint64    `json:"outliers_filtered"`

	// Windowed statistics
	Window5Min  WindowedStats `json:"window_5min"`
	Window15Min WindowedStats `json:"window_15min"`
	Window30Min WindowedStats `json:"window_30min"`

	// Chart data
	MTIEData  []MTIEPoint    `json:"mtie_data"`  // MTIE curve
	AllanData []AllanPoint   `json:"allan_data"` // Allan deviation curve
	TIEData   []TIEPoint     `json:"tie_data"`   // Time series for TIE plot
	Histogram *HistogramData `json:"histogram"`  // Histogram with metadata
}

func NewStatistics() *Statistics {
	return &Statistics{
		updates:      make(chan *StatsSnapshot, 10),
		measurements: make([]TimestampedMeasurement, 0, 2000), // Preallocate for ~30min @ 1Hz
	}
}

// isOutlier detects extreme outliers using median absolute deviation (MAD)
// Returns true if the value is an extreme outlier (> 10 * MAD from median)
func (s *Statistics) isOutlier(value float64) bool {
	if len(s.measurements) < 10 {
		return false // Not enough data to determine outliers
	}

	// Use last 100 measurements (or all if fewer) for median calculation
	sampleSize := len(s.measurements)
	if sampleSize > 100 {
		sampleSize = 100
	}
	startIdx := len(s.measurements) - sampleSize

	// Extract phase values for median calculation
	values := make([]float64, sampleSize)
	for i := 0; i < sampleSize; i++ {
		values[i] = s.measurements[startIdx+i].PhaseNs
	}

	// Calculate median using sorting (simple approach)
	sortedVals := make([]float64, len(values))
	copy(sortedVals, values)

	// Simple insertion sort for small arrays
	for i := 1; i < len(sortedVals); i++ {
		key := sortedVals[i]
		j := i - 1
		for j >= 0 && sortedVals[j] > key {
			sortedVals[j+1] = sortedVals[j]
			j--
		}
		sortedVals[j+1] = key
	}

	median := sortedVals[len(sortedVals)/2]

	// Calculate MAD (Median Absolute Deviation)
	absDeviations := make([]float64, len(values))
	for i, v := range values {
		absDeviations[i] = math.Abs(v - median)
	}

	// Sort absolute deviations to find median
	for i := 1; i < len(absDeviations); i++ {
		key := absDeviations[i]
		j := i - 1
		for j >= 0 && absDeviations[j] > key {
			absDeviations[j+1] = absDeviations[j]
			j--
		}
		absDeviations[j+1] = key
	}

	mad := absDeviations[len(absDeviations)/2]

	// Modified Z-score threshold: if value is > 10 * MAD from median, it's an outlier
	// This catches extreme spikes (like 60ms when normal is ~125µs)
	// Use small epsilon to avoid division by zero
	if mad < 100 { // If MAD < 100ns, use 10µs as minimum MAD
		mad = 10000
	}

	deviation := math.Abs(value - median)
	threshold := 10.0 * mad

	return deviation > threshold
}

// Update statistics with new measurements (Welford's algorithm)
func (s *Statistics) Update(batch *Batch) {
	s.mu.Lock()
	defer s.mu.Unlock()

	now := time.Now()

	for _, m := range batch.Measurements {
		// Check if this measurement is an outlier BEFORE adding to measurements
		isOutlier := s.isOutlier(m.PhaseNs)

		// Always store measurement (for CSV download and history)
		s.measurements = append(s.measurements, TimestampedMeasurement{
			Measurement: m,
			ReceivedAt:  now,
		})

		// If outlier, increment counter and skip statistics update
		if isOutlier {
			s.OutliersFiltered++
			continue
		}

		// Update statistics only for non-outlier measurements
		s.SampleCount++

		// Welford's algorithm for running mean and variance
		delta := m.PhaseNs - s.Mean
		s.Mean += delta / float64(s.SampleCount)
		delta2 := m.PhaseNs - s.Mean
		s.M2 += delta * delta2

		// Min/max
		if s.SampleCount == 1 {
			s.Min = m.PhaseNs
			s.Max = m.PhaseNs
		} else {
			if m.PhaseNs < s.Min {
				s.Min = m.PhaseNs
			}
			if m.PhaseNs > s.Max {
				s.Max = m.PhaseNs
			}
		}

		// Direction tracking
		if m.GmFirst {
			s.GmFirstCount++
		} else {
			s.SlaveFirstCount++
		}

		// Keep last values
		s.LastCrystalErr = m.CrystalErrorNs
		s.LastScaleFactor = m.ScaleFactor
	}

	// Prune old measurements (keep last 30 minutes)
	cutoff := now.Add(-30 * time.Minute)
	pruneIndex := 0
	for i, m := range s.measurements {
		if m.ReceivedAt.After(cutoff) {
			pruneIndex = i
			break
		}
	}
	if pruneIndex > 0 {
		s.measurements = s.measurements[pruneIndex:]
	}

	s.BatchesReceived++
	s.LastUpdate = now
	s.BufferDropped = batch.Stats.BufferDropped

	// Broadcast update to WebSocket clients
	select {
	case s.updates <- s.snapshot():
	default:
		// Drop if channel full (no blocking)
	}
}

// filterOutliers returns a filtered copy of measurements with outliers removed
// This is used for chart data generation to avoid outlier spikes in visualizations
func (s *Statistics) filterOutliers(measurements []TimestampedMeasurement) []TimestampedMeasurement {
	if len(measurements) < 10 {
		return measurements // Not enough data to filter
	}

	// Calculate median of all measurements
	values := make([]float64, len(measurements))
	for i, m := range measurements {
		values[i] = m.PhaseNs
	}

	// Sort to find median
	sortedVals := make([]float64, len(values))
	copy(sortedVals, values)
	for i := 1; i < len(sortedVals); i++ {
		key := sortedVals[i]
		j := i - 1
		for j >= 0 && sortedVals[j] > key {
			sortedVals[j+1] = sortedVals[j]
			j--
		}
		sortedVals[j+1] = key
	}
	median := sortedVals[len(sortedVals)/2]

	// Calculate MAD
	absDeviations := make([]float64, len(values))
	for i, v := range values {
		absDeviations[i] = math.Abs(v - median)
	}
	for i := 1; i < len(absDeviations); i++ {
		key := absDeviations[i]
		j := i - 1
		for j >= 0 && absDeviations[j] > key {
			absDeviations[j+1] = absDeviations[j]
			j--
		}
		absDeviations[j+1] = key
	}
	mad := absDeviations[len(absDeviations)/2]

	if mad < 100 {
		mad = 10000 // Minimum MAD of 10µs
	}

	threshold := 10.0 * mad

	// Filter measurements
	filtered := make([]TimestampedMeasurement, 0, len(measurements))
	for _, m := range measurements {
		if math.Abs(m.PhaseNs-median) <= threshold {
			filtered = append(filtered, m)
		}
	}

	return filtered
}

// Calculate statistics for a time window using Pico measurement timestamps
func (s *Statistics) calculateWindow(duration time.Duration) WindowedStats {
	if len(s.measurements) == 0 {
		return WindowedStats{}
	}

	// Filter outliers first
	filtered := s.filterOutliers(s.measurements)
	if len(filtered) == 0 {
		return WindowedStats{}
	}

	// Use Pico timestamps - find measurements within the duration before the latest measurement
	latestTimeUs := filtered[len(filtered)-1].TimestampUs
	cutoffTimeUs := latestTimeUs - uint64(duration.Microseconds())

	var count uint64
	var sum, min, max float64
	first := true

	// First pass: count, sum, min, max
	for _, m := range filtered {
		if m.TimestampUs >= cutoffTimeUs {
			count++
			sum += m.PhaseNs
			if first {
				min = m.PhaseNs
				max = m.PhaseNs
				first = false
			} else {
				if m.PhaseNs < min {
					min = m.PhaseNs
				}
				if m.PhaseNs > max {
					max = m.PhaseNs
				}
			}
		}
	}

	if count == 0 {
		return WindowedStats{}
	}

	mean := sum / float64(count)

	// Second pass: calculate variance
	var m2 float64
	for _, m := range filtered {
		if m.TimestampUs >= cutoffTimeUs {
			delta := m.PhaseNs - mean
			m2 += delta * delta
		}
	}

	stddev := 0.0
	if count > 1 {
		stddev = math.Sqrt(m2 / float64(count-1))
	}

	return WindowedStats{
		Count:  count,
		Mean:   mean,
		StdDev: stddev,
		Min:    min,
		Max:    max,
	}
}

// Calculate MTIE (Maximum Time Interval Error) for various tau values
// MTIE(tau) = max over all windows of size tau of (max - min) within window
func (s *Statistics) calculateMTIE() []MTIEPoint {
	if len(s.measurements) < 2 {
		return nil
	}

	// Filter outliers first
	filtered := s.filterOutliers(s.measurements)
	if len(filtered) < 2 {
		return nil
	}

	// Calculate max duration based on Pico timestamps (microseconds)
	startTimeUs := filtered[0].TimestampUs
	endTimeUs := filtered[len(filtered)-1].TimestampUs
	maxDurationSec := float64(endTimeUs-startTimeUs) / 1000000.0

	if maxDurationSec < 1.0 {
		return nil
	}

	// Use logarithmically spaced tau values
	tauValues := []float64{1, 2, 5, 10, 20, 30, 60, 120, 300, 600, 900, 1200, 1800}
	result := make([]MTIEPoint, 0, len(tauValues))

	for _, tau := range tauValues {
		if tau > maxDurationSec {
			break
		}

		tauUs := uint64(tau * 1000000.0) // Convert tau to microseconds
		maxTIE := 0.0

		// Sliding window over all measurements using Pico timestamps
		for i := 0; i < len(filtered); i++ {
			windowEndUs := filtered[i].TimestampUs + tauUs

			// Find measurements within this window
			windowMin, windowMax := filtered[i].PhaseNs, filtered[i].PhaseNs
			for j := i + 1; j < len(filtered); j++ {
				if filtered[j].TimestampUs > windowEndUs {
					break
				}
				if filtered[j].PhaseNs < windowMin {
					windowMin = filtered[j].PhaseNs
				}
				if filtered[j].PhaseNs > windowMax {
					windowMax = filtered[j].PhaseNs
				}
			}

			tie := windowMax - windowMin
			if tie > maxTIE {
				maxTIE = tie
			}
		}

		result = append(result, MTIEPoint{Tau: tau, MTIE: maxTIE})
	}

	return result
}

// Calculate Allan Deviation for various tau values
// ADEV(tau) = sqrt(0.5 * mean((x[i+1] - x[i])^2)) for non-overlapping windows
func (s *Statistics) calculateAllan() []AllanPoint {
	if len(s.measurements) < 3 {
		return nil
	}

	// Filter outliers first
	filtered := s.filterOutliers(s.measurements)
	if len(filtered) < 3 {
		return nil
	}

	// Tau values to calculate (in seconds)
	tauValues := []float64{1, 2, 5, 10, 20, 30, 60, 120, 300, 600, 900, 1800}
	result := make([]AllanPoint, 0, len(tauValues))

	// Calculate max duration based on Pico timestamps
	startTimeUs := filtered[0].TimestampUs
	endTimeUs := filtered[len(filtered)-1].TimestampUs
	maxDurationSec := float64(endTimeUs-startTimeUs) / 1000000.0

	for _, tau := range tauValues {
		if tau > maxDurationSec {
			break
		}

		tauUs := uint64(tau * 1000000.0) // Convert tau to microseconds

		// Collect non-overlapping window averages using Pico timestamps
		var windowAverages []float64
		i := 0

		for i < len(filtered) {
			windowStartUs := filtered[i].TimestampUs
			windowEndUs := windowStartUs + tauUs

			// Calculate average for this window
			sum := 0.0
			count := 0
			for j := i; j < len(filtered) && filtered[j].TimestampUs < windowEndUs; j++ {
				sum += filtered[j].PhaseNs
				count++
			}

			if count > 0 {
				windowAverages = append(windowAverages, sum/float64(count))
				i += count
			} else {
				break
			}
		}

		// Need at least 2 windows to calculate Allan deviation
		if len(windowAverages) < 2 {
			continue
		}

		// Calculate Allan variance
		sumSquaredDiffs := 0.0
		for i := 0; i < len(windowAverages)-1; i++ {
			diff := windowAverages[i+1] - windowAverages[i]
			sumSquaredDiffs += diff * diff
		}

		allanVar := sumSquaredDiffs / (2.0 * float64(len(windowAverages)-1))
		allanDev := math.Sqrt(allanVar)

		result = append(result, AllanPoint{
			Tau:   tau,
			ADev:  allanDev,
			Count: len(windowAverages),
		})
	}

	return result
}

// Build TIE time series data (all measurements in the last 30 minutes)
func (s *Statistics) buildTIEData() []TIEPoint {
	if len(s.measurements) == 0 {
		return nil
	}

	// Filter outliers first
	filtered := s.filterOutliers(s.measurements)
	if len(filtered) == 0 {
		return nil
	}

	result := make([]TIEPoint, 0, len(filtered))

	// Use Pico timestamp (TimestampUs) for X-axis, not server receive time
	// TimestampUs is in microseconds, convert to seconds
	startTimeUs := filtered[0].TimestampUs

	for i := 0; i < len(filtered); i++ {
		m := filtered[i]
		elapsed := float64(m.TimestampUs-startTimeUs) / 1000000.0 // Convert µs to seconds
		result = append(result, TIEPoint{
			Time:    elapsed,
			PhaseNs: m.PhaseNs,
		})
	}

	return result
}

// Build histogram data (100ns bins, dynamically sized to fit all data)
func (s *Statistics) buildHistogram() *HistogramData {
	if len(s.measurements) == 0 {
		return nil
	}

	// Filter outliers first
	filtered := s.filterOutliers(s.measurements)
	if len(filtered) == 0 {
		return nil
	}

	const binSize = 100.0 // nanoseconds

	// Find min/max to determine range
	minVal := filtered[0].PhaseNs
	maxVal := filtered[0].PhaseNs
	for _, m := range filtered {
		if m.PhaseNs < minVal {
			minVal = m.PhaseNs
		}
		if m.PhaseNs > maxVal {
			maxVal = m.PhaseNs
		}
	}

	// Add 10% margin on each side
	margin := (maxVal - minVal) * 0.1
	minVal -= margin
	maxVal += margin

	// Calculate number of bins needed
	rangeNs := maxVal - minVal
	numBins := int(rangeNs/binSize) + 1

	// Limit to reasonable size
	if numBins > 2000 {
		numBins = 2000
	}
	if numBins < 100 {
		numBins = 100
	}

	bins := make([]int, numBins)

	// Populate histogram
	for _, m := range filtered {
		bin := int((m.PhaseNs - minVal) / binSize)
		if bin >= 0 && bin < numBins {
			bins[bin]++
		}
	}

	return &HistogramData{
		Bins:    bins,
		MinVal:  minVal,
		BinSize: binSize,
	}
}

// Get current snapshot (thread-safe)
func (s *Statistics) Snapshot() *StatsSnapshot {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.snapshot()
}

// Internal snapshot (must hold lock)
func (s *Statistics) snapshot() *StatsSnapshot {
	stddev := 0.0
	if s.SampleCount > 1 {
		stddev = math.Sqrt(s.M2 / float64(s.SampleCount-1))
	}

	return &StatsSnapshot{
		SampleCount:      s.SampleCount,
		Mean:             s.Mean,
		StdDev:           stddev,
		Min:              s.Min,
		Max:              s.Max,
		LastCrystalErr:   s.LastCrystalErr,
		LastScaleFactor:  s.LastScaleFactor,
		BatchesReceived:  s.BatchesReceived,
		LastUpdate:       s.LastUpdate,
		BufferDropped:    s.BufferDropped,
		GmFirstCount:     s.GmFirstCount,
		SlaveFirstCount:  s.SlaveFirstCount,
		OutliersFiltered: s.OutliersFiltered,
		Window5Min:       s.calculateWindow(5 * time.Minute),
		Window15Min:      s.calculateWindow(15 * time.Minute),
		Window30Min:      s.calculateWindow(30 * time.Minute),
		MTIEData:         s.calculateMTIE(),
		AllanData:        s.calculateAllan(),
		TIEData:          s.buildTIEData(),
		Histogram:        s.buildHistogram(),
	}
}

// Subscribe to statistics updates
func (s *Statistics) Subscribe() <-chan *StatsSnapshot {
	return s.updates
}

// Reset clears all statistics and measurements
func (s *Statistics) Reset() {
	s.mu.Lock()
	defer s.mu.Unlock()

	s.SampleCount = 0
	s.Mean = 0
	s.M2 = 0
	s.Min = 0
	s.Max = 0
	s.LastCrystalErr = 0
	s.LastScaleFactor = 0
	s.BatchesReceived = 0
	s.BufferDropped = 0
	s.GmFirstCount = 0
	s.SlaveFirstCount = 0
	s.OutliersFiltered = 0
	s.measurements = make([]TimestampedMeasurement, 0, 2000)
}

// GetRawMeasurements returns all stored measurements for download
func (s *Statistics) GetRawMeasurements() []TimestampedMeasurement {
	s.mu.RLock()
	defer s.mu.RUnlock()

	// Return a copy to avoid race conditions
	measurements := make([]TimestampedMeasurement, len(s.measurements))
	copy(measurements, s.measurements)
	return measurements
}

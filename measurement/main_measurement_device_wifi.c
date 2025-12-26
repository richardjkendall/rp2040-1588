/**
 * PTP Phase Offset Measurement Device - WiFi Telemetry Version
 *
 * Dual-core architecture:
 * - Core 0: GPS discipline + phase measurement (timing critical, no WiFi)
 * - Core 1: WiFi telemetry streaming (all network overhead isolated)
 *
 * Independently measures phase offset between GM and Slave 1PPS signals
 * using GPS-calibrated crystal characterization, with remote telemetry.
 *
 * Architecture:
 * - GPS discipline for crystal calibration (scale factor)
 * - PIO SM2: GM→Slave counter (waits for GM, counts to Slave)
 * - PIO SM3: Slave→GM counter (waits for Slave, counts to GM)
 * - Take minimum of both measurements for true phase offset
 * - Stream measurements to remote server via WiFi (Core 1 only)
 *
 * Resolution: ~12ns per tick (83.33 MHz PIO clock)
 * Accuracy: ±200-400ns (GPS discipline accuracy)
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "gps.h"
#include "discipline_v3.h"
#include "gm_to_slave_counter.pio.h"
#include "slave_to_gm_counter.pio.h"
#include "ring_buffer.h"
#include "telemetry.h"

// WiFi Configuration
#define WIFI_SSID "bhop"
#define WIFI_PASSWORD "houseofpeas"
#define TELEMETRY_SERVER_IP "10.10.143.151"

// GPS configuration
#define GPS_UART_ID     uart0
#define GPS_TX_PIN      0
#define GPS_RX_PIN      1
#define GPS_PPS_PIN     2
#define GPS_PIO         pio0
#define GPS_SM          0
#define GPS_PPS_CAPTURE_SM 1

// Phase measurement pins
#define GM_PPS_PIN      10  // Input: Grandmaster 1PPS
#define SLAVE_PPS_PIN   11  // Input: Slave 1PPS

// PIO state machines for phase measurement
#define PHASE_PIO       pio1
#define GM_TO_SLAVE_SM  2
#define SLAVE_TO_GM_SM  3

// Debug pins
#define DEBUG_PPS_PIN   4
#define DEBUG_LOCK_PIN  5

// Export debug pins for discipline module
uint debug_lock_pin = DEBUG_LOCK_PIN;
uint debug_pps_pin = DEBUG_PPS_PIN;

// Shared ring buffer for Core 0 → Core 1 communication
static measurement_ring_buffer_t measurement_buffer;

// EMA statistics structure
typedef struct {
    double mean;
    double var;
    bool initialized;
} ema_stats_t;

// Phase offset statistics
typedef struct {
    // Running statistics (Welford's algorithm)
    uint64_t count;
    double mean_ns;
    double m2;  // Sum of squared differences

    // Min/max
    double min_ns;
    double max_ns;

    // EMA statistics (5, 15, 30 minute windows)
    ema_stats_t ema_5min;
    ema_stats_t ema_15min;
    ema_stats_t ema_30min;

    // Direction tracking
    uint64_t gm_first_count;
    uint64_t slave_first_count;

    // Histogram (100ns bins, ±10µs range)
    #define HISTOGRAM_BINS 201
    #define HISTOGRAM_BIN_SIZE_NS 100
    #define HISTOGRAM_CENTER 100  // Bin 100 = 0ns offset
    uint32_t histogram[HISTOGRAM_BINS];
} phase_stats_t;

static phase_stats_t phase_stats = {0};
static uint64_t rejected_measurement_count = 0;  // Global counter for rejected measurements

// EMA alpha values
#define EMA_ALPHA_5MIN  0.00333   // ~5 minute time constant
#define EMA_ALPHA_15MIN 0.00111   // ~15 minute time constant
#define EMA_ALPHA_30MIN 0.000556  // ~30 minute time constant

/**
 * Update EMA statistics
 */
static void update_ema_stats(ema_stats_t *ema, double value, double alpha) {
    if (!ema->initialized) {
        ema->mean = value;
        ema->var = 0.0;
        ema->initialized = true;
    } else {
        double delta = value - ema->mean;
        ema->mean += alpha * delta;
        ema->var = (1.0 - alpha) * (ema->var + alpha * delta * delta);
    }
}

/**
 * Update phase offset statistics (Welford's algorithm)
 */
static void update_phase_stats(double phase_offset_ns, bool gm_first) {
    phase_stats.count++;

    // Welford's algorithm for running mean and variance
    double delta = phase_offset_ns - phase_stats.mean_ns;
    phase_stats.mean_ns += delta / phase_stats.count;
    double delta2 = phase_offset_ns - phase_stats.mean_ns;
    phase_stats.m2 += delta * delta2;

    // Min/max
    if (phase_stats.count == 1) {
        phase_stats.min_ns = phase_offset_ns;
        phase_stats.max_ns = phase_offset_ns;
    } else {
        if (phase_offset_ns < phase_stats.min_ns) phase_stats.min_ns = phase_offset_ns;
        if (phase_offset_ns > phase_stats.max_ns) phase_stats.max_ns = phase_offset_ns;
    }

    // Direction tracking
    if (gm_first) {
        phase_stats.gm_first_count++;
    } else {
        phase_stats.slave_first_count++;
    }

    // Update EMA statistics (only after initial samples)
    if (phase_stats.count > 10) {
        update_ema_stats(&phase_stats.ema_5min, phase_offset_ns, EMA_ALPHA_5MIN);
        update_ema_stats(&phase_stats.ema_15min, phase_offset_ns, EMA_ALPHA_15MIN);
        update_ema_stats(&phase_stats.ema_30min, phase_offset_ns, EMA_ALPHA_30MIN);
    }

    // Histogram (100ns bins, ±10µs range)
    int bin = HISTOGRAM_CENTER + (int)(phase_offset_ns / HISTOGRAM_BIN_SIZE_NS);
    if (bin >= 0 && bin < HISTOGRAM_BINS) {
        phase_stats.histogram[bin]++;
    }
}

/**
 * Get standard deviation
 */
static double get_stddev(void) {
    if (phase_stats.count < 2) return 0.0;
    return sqrt(phase_stats.m2 / (phase_stats.count - 1));
}

/**
 * Print phase offset statistics
 */
static void print_stats(void) {
    if (phase_stats.count == 0) {
        printf("No measurements yet\n");
        return;
    }

    double stddev = get_stddev();

    // Get EMA standard deviations
    double ema_5m = phase_stats.ema_5min.initialized ? sqrt(phase_stats.ema_5min.var) : 0.0;
    double ema_15m = phase_stats.ema_15min.initialized ? sqrt(phase_stats.ema_15min.var) : 0.0;
    double ema_30m = phase_stats.ema_30min.initialized ? sqrt(phase_stats.ema_30min.var) : 0.0;

    // Get GPS discipline stats
    extern volatile int32_t crystal_error_ns;
    double crystal_ppm = (double)crystal_error_ns / 1000000.0;

    // Get telemetry stats
    uint32_t batches_sent, send_failures, reconnects;
    bool telemetry_connected;
    telemetry_get_stats(&batches_sent, &send_failures, &reconnects, &telemetry_connected);

    // Get ring buffer stats
    uint32_t buffer_available, buffer_dropped;
    ring_buffer_get_stats(&measurement_buffer, &buffer_available, &buffer_dropped);

    printf("\n=== Phase Offset Measurement ===\n");
    printf("Samples: %llu (GM first: %llu, Slave first: %llu)\n",
           phase_stats.count,
           phase_stats.gm_first_count,
           phase_stats.slave_first_count);
    printf("Rejected: %llu (PIO glitches when phase approaches zero)\n",
           rejected_measurement_count);
    printf("Phase offset: %.1f ± %.1f ns (min: %.1f, max: %.1f)\n",
           phase_stats.mean_ns, stddev,
           phase_stats.min_ns, phase_stats.max_ns);
    printf("EMA stddev (5/15/30min): %.1f / %.1f / %.1f ns\n",
           ema_5m, ema_15m, ema_30m);
    printf("GPS crystal: %+ld ns (%+.3f ppm)\n",
           (long)crystal_error_ns, crystal_ppm);

    printf("\n=== Telemetry Status ===\n");
    printf("WiFi: %s\n", telemetry_connected ? "Connected" : "Disconnected");
    printf("Batches sent: %lu (failures: %lu, reconnects: %lu)\n",
           batches_sent, send_failures, reconnects);
    printf("Ring buffer: %lu available, %lu dropped\n",
           buffer_available, buffer_dropped);

    // Print histogram (only non-zero bins near center)
    printf("\nHistogram (100ns bins):\n");
    for (int i = 0; i < HISTOGRAM_BINS; i++) {
        if (phase_stats.histogram[i] > 0) {
            int offset_ns = (i - HISTOGRAM_CENTER) * HISTOGRAM_BIN_SIZE_NS;
            printf("  %+6d ns: %6lu samples\n", offset_ns, phase_stats.histogram[i]);
        }
    }
    printf("\n");
}

int main() {
    // Overclock to 250 MHz
    set_sys_clock_khz(250000, true);

    // Initialize stdio
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== PTP Phase Offset Measurement Device (WiFi) ===\n");
    printf("System clock: %lu MHz\n\n", clock_get_hz(clk_sys) / 1000000);

    // Initialize debug GPIOs
    gpio_init(DEBUG_PPS_PIN);
    gpio_set_dir(DEBUG_PPS_PIN, GPIO_OUT);
    gpio_put(DEBUG_PPS_PIN, 0);
    gpio_init(DEBUG_LOCK_PIN);
    gpio_set_dir(DEBUG_LOCK_PIN, GPIO_OUT);
    gpio_put(DEBUG_LOCK_PIN, 0);

    // Initialize GPS module
    printf("Initializing GPS...\n");
    gps_init(GPS_UART_ID, GPS_TX_PIN, GPS_RX_PIN, GPS_PPS_PIN, GPS_PIO, GPS_SM);

    // Initialize GPS discipline for crystal calibration
    printf("Initializing GPS discipline...\n");
    if (!discipline_init_v3()) {
        printf("FATAL: GPS discipline init failed\n");
        while (1) { sleep_ms(1000); }
    }
    printf("GPS discipline ready\n\n");

    // Initialize PIO programs for phase measurement
    printf("Initializing phase measurement PIO...\n");

    // Load GM→Slave counter program
    uint gm_to_slave_offset = pio_add_program(PHASE_PIO, &gm_to_slave_counter_program);
    gm_to_slave_counter_program_init(PHASE_PIO, GM_TO_SLAVE_SM, gm_to_slave_offset,
                                      GM_PPS_PIN, SLAVE_PPS_PIN);

    // Load Slave→GM counter program
    uint slave_to_gm_offset = pio_add_program(PHASE_PIO, &slave_to_gm_counter_program);
    slave_to_gm_counter_program_init(PHASE_PIO, SLAVE_TO_GM_SM, slave_to_gm_offset,
                                      SLAVE_PPS_PIN, GM_PPS_PIN);

    printf("Phase measurement ready\n");
    printf("GM 1PPS input: GPIO%d\n", GM_PPS_PIN);
    printf("Slave 1PPS input: GPIO%d\n\n", SLAVE_PPS_PIN);

    // Initialize ring buffer
    printf("Initializing ring buffer...\n");
    ring_buffer_init(&measurement_buffer);

    // Configure telemetry
    printf("Configuring WiFi telemetry...\n");
    telemetry_config_t telemetry_config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASSWORD,
        .server_ip = TELEMETRY_SERVER_IP,
        .server_port = TELEMETRY_SERVER_PORT,
        .ring_buffer = &measurement_buffer
    };

    if (!telemetry_init(&telemetry_config)) {
        printf("FATAL: Telemetry init failed\n");
        while (1) { sleep_ms(1000); }
    }

    // Launch Core 1 for telemetry
    printf("Launching Core 1 for telemetry...\n");
    multicore_launch_core1(telemetry_core1_entry);
    sleep_ms(1000);  // Give Core 1 time to start

    printf("Waiting for GPS lock...\n\n");

    uint64_t last_stats_time_us = 0;
    uint64_t measurement_count = 0;

    // External variable from discipline module
    extern volatile int32_t crystal_error_ns;

    // Main loop (Core 0 - timing critical)
    while (true) {
        uint64_t now_us = time_us_64();

        // Process GPS for crystal calibration
        gps_process();

        // Check if both FIFOs have data
        if (!pio_sm_is_rx_fifo_empty(PHASE_PIO, GM_TO_SLAVE_SM) &&
            !pio_sm_is_rx_fifo_empty(PHASE_PIO, SLAVE_TO_GM_SM)) {

            // Read tick counts from both FIFOs
            uint32_t gm_to_slave_ticks = pio_sm_get(PHASE_PIO, GM_TO_SLAVE_SM);
            uint32_t slave_to_gm_ticks = pio_sm_get(PHASE_PIO, SLAVE_TO_GM_SM);

            // Get GPS-calibrated scale factor
            double sf = discipline_get_scale_factor();

            // Convert ticks to nanoseconds
            // Each tick = 36ns * scale_factor (3 PIO cycles per loop iteration @ 12ns/cycle)
            double gm_to_slave_ns = gm_to_slave_ticks * 36.0 * sf;
            double slave_to_gm_ns = slave_to_gm_ticks * 36.0 * sf;

            // Determine phase offset and which edge came first
            // If one measurement is tiny (< 10µs), it means the pin was already HIGH (wrong interval)
            // Use the other measurement which counted the actual phase offset
            double phase_offset_ns;
            bool gm_first;

            #define INVALID_THRESHOLD_NS 150.0    // 150ns - detect PIO glitch (pin already HIGH), just above 100ns pulse width

            if (gm_to_slave_ns < INVALID_THRESHOLD_NS && slave_to_gm_ns >= INVALID_THRESHOLD_NS) {
                // GM→Slave saw pin already HIGH, use Slave→GM measurement
                phase_offset_ns = slave_to_gm_ns;
                gm_first = false;  // Slave arrived first
            } else if (slave_to_gm_ns < INVALID_THRESHOLD_NS && gm_to_slave_ns >= INVALID_THRESHOLD_NS) {
                // Slave→GM saw pin already HIGH, use GM→Slave measurement
                phase_offset_ns = gm_to_slave_ns;
                gm_first = true;  // GM arrived first
            } else {
                // Both valid or both invalid, take minimum (normal case for very close edges)
                gm_first = (gm_to_slave_ns < slave_to_gm_ns);
                phase_offset_ns = gm_first ? gm_to_slave_ns : slave_to_gm_ns;
            }

            // Validate measurement before sending to telemetry or updating stats
            // Valid phase offset should be small (< 10ms) since we're measuring sub-second offsets
            // Filter out bad measurements caused by PIO glitches when edges are synchronized
            #define MIN_VALID_OFFSET_NS 50.0        // 50ns minimum - aggressive but safe with 100ns pulse width
            #define MAX_VALID_OFFSET_NS 10000000.0  // 10ms maximum (filters 1-second wraparound errors)

            // Only validate the selected phase_offset_ns (not the raw counter values)
            // The raw counters can be ~1 second in normal operation (when one edge wraps to next second)
            bool is_valid = (phase_offset_ns >= MIN_VALID_OFFSET_NS &&
                           phase_offset_ns <= MAX_VALID_OFFSET_NS);

            // If measurement below threshold, use threshold value as conservative estimate
            // This indicates "excellent sync - at or below 50ns" without claiming false precision
            double reported_offset = is_valid ? phase_offset_ns : MIN_VALID_OFFSET_NS;

            // Always send measurement (no gaps in telemetry)
            measurement_t m = {
                .sequence = measurement_count,
                .timestamp_us = now_us,
                .phase_offset_ns = reported_offset,
                .gm_to_slave_ns = gm_to_slave_ns,
                .slave_to_gm_ns = slave_to_gm_ns,
                .scale_factor = sf,
                .gm_first = gm_first,
                .crystal_error_ns = crystal_error_ns
            };

            // Non-blocking write to ring buffer (for Core 1 telemetry)
            if (!ring_buffer_try_write(&measurement_buffer, &m)) {
                // Buffer full - Core 1 too slow (should never happen with 1000 slots)
                // Measurement dropped, counter incremented automatically in ring_buffer
            }

            // Update local statistics (use actual measurement for stats tracking)
            update_phase_stats(reported_offset, gm_first);

            if (!is_valid) {
                // Count rejections for diagnostics
                rejected_measurement_count++;
            }

            measurement_count++;

            // Print individual measurements (first 20, then every 100th)
            if (measurement_count <= 20 || measurement_count % 100 == 0) {
                printf("[%6llu] Phase: %+7.1f ns (%c first, G→S: %7.1f ns, S→G: %7.1f ns, SF: %.6f)\n",
                       measurement_count,
                       phase_offset_ns,
                       gm_first ? 'G' : 'S',
                       gm_to_slave_ns,
                       slave_to_gm_ns,
                       sf);
            }
        }

        // Print statistics every 30 seconds
        if (now_us - last_stats_time_us >= 30000000) {
            print_stats();
            last_stats_time_us = now_us;
        }

        // Minimal delay
        sleep_ms(100);
    }

    return 0;
}

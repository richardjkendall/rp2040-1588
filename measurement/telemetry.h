#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include "ring_buffer.h"

// Telemetry configuration
#define TELEMETRY_BATCH_SIZE 60          // Send every 60 measurements (~1 minute @ 1Hz)
#define TELEMETRY_SERVER_PORT 5001       // Default TCP port
#define TELEMETRY_RECONNECT_DELAY_MS 2000  // Wait 2s between reconnect attempts
#define TELEMETRY_KEEPALIVE_MS 20000     // Send keepalive every 20s if no data

/**
 * Telemetry configuration
 */
typedef struct {
    const char *ssid;
    const char *password;
    const char *server_ip;
    uint16_t server_port;
    measurement_ring_buffer_t *ring_buffer;
} telemetry_config_t;

/**
 * Initialize telemetry subsystem
 * Must be called from Core 0 before launching Core 1
 *
 * @param config Telemetry configuration
 * @return true if configuration accepted, false on error
 */
bool telemetry_init(const telemetry_config_t *config);

/**
 * Core 1 entry point
 * This function never returns - it runs the telemetry loop forever
 *
 * Connects to WiFi, establishes TCP connection, reads from ring buffer,
 * batches measurements, encodes as JSON, and sends to server.
 */
void telemetry_core1_entry(void);

/**
 * Get telemetry statistics
 * Can be called from Core 0 to check Core 1 status
 *
 * @param batches_sent Total batches sent successfully
 * @param send_failures Total send failures
 * @param reconnects Total reconnection attempts
 * @param connected Current connection status
 */
void telemetry_get_stats(uint32_t *batches_sent, uint32_t *send_failures,
                         uint32_t *reconnects, bool *connected);

#endif // TELEMETRY_H

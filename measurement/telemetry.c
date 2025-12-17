#include "telemetry.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include <string.h>
#include <stdio.h>

// Logging levels
#define TELEMETRY_LOG_ERRORS  1  // Always log errors
#define TELEMETRY_LOG_INFO    2  // Log connections/disconnections
#define TELEMETRY_LOG_DEBUG   3  // Log every batch

#define TELEMETRY_LOG_LEVEL TELEMETRY_LOG_INFO  // Default level

#define TEL_LOG_ERROR(...)  printf("[Telemetry] " __VA_ARGS__)
#define TEL_LOG_INFO(...)   if (TELEMETRY_LOG_LEVEL >= 2) printf("[Telemetry] " __VA_ARGS__)
#define TEL_LOG_DEBUG(...)  if (TELEMETRY_LOG_LEVEL >= 3) printf("[Telemetry] " __VA_ARGS__)

// TCP retry configuration
#define MAX_SEND_RETRIES 3
#define SEND_RETRY_DELAY_MS 100

// Configuration (set by telemetry_init)
static telemetry_config_t g_config;

// Statistics
static volatile uint32_t g_batches_sent = 0;
static volatile uint32_t g_send_failures = 0;
static volatile uint32_t g_reconnects = 0;
static volatile bool g_connected = false;

// Batch tracking
static uint32_t g_batch_seq = 0;

bool telemetry_init(const telemetry_config_t *config) {
    if (!config || !config->ring_buffer) {
        return false;
    }

    g_config = *config;
    return true;
}

void telemetry_get_stats(uint32_t *batches_sent, uint32_t *send_failures,
                         uint32_t *reconnects, bool *connected) {
    if (batches_sent) *batches_sent = g_batches_sent;
    if (send_failures) *send_failures = g_send_failures;
    if (reconnects) *reconnects = g_reconnects;
    if (connected) *connected = g_connected;
}

/**
 * Check if WiFi is currently connected
 */
static bool wifi_is_connected(void) {
    return cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;
}

/**
 * Connect to WiFi
 */
static bool wifi_connect(void) {
    // If already connected, don't try to reconnect
    if (wifi_is_connected()) {
        TEL_LOG_DEBUG("WiFi already connected\n");
        return true;
    }

    TEL_LOG_INFO("Connecting to WiFi SSID: %s\n", g_config.ssid);

    if (cyw43_arch_wifi_connect_timeout_ms(g_config.ssid, g_config.password,
                                           CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        TEL_LOG_ERROR("WiFi connection failed\n");
        return false;
    }

    TEL_LOG_INFO("WiFi connected\n");
    return true;
}

/**
 * Build JSON message for a batch of measurements
 *
 * Format:
 * {
 *   "device": "measurement_device",
 *   "batch_seq": 42,
 *   "count": 60,
 *   "measurements": [...],
 *   "stats": {"buffer_available": 850, "buffer_dropped": 0}
 * }
 */
static int build_json_message(const measurement_t *measurements, int count,
                              char *buffer, size_t buffer_size) {
    // Get buffer stats
    uint32_t available, dropped;
    ring_buffer_get_stats(g_config.ring_buffer, &available, &dropped);

    // Start JSON
    int len = snprintf(buffer, buffer_size,
                      "{\"device\":\"measurement_device\","
                      "\"batch_seq\":%u,"
                      "\"count\":%d,"
                      "\"measurements\":[",
                      g_batch_seq++, count);

    // Add measurements
    for (int i = 0; i < count; i++) {
        const measurement_t *m = &measurements[i];

        len += snprintf(buffer + len, buffer_size - len,
                       "%s{\"seq\":%llu,"
                       "\"timestamp_us\":%llu,"
                       "\"phase_ns\":%.1f,"
                       "\"gm_to_slave_ns\":%.1f,"
                       "\"slave_to_gm_ns\":%.1f,"
                       "\"scale_factor\":%.9f,"
                       "\"gm_first\":%s,"
                       "\"crystal_error_ns\":%d}",
                       (i > 0) ? "," : "",
                       m->sequence,
                       m->timestamp_us,
                       m->phase_offset_ns,
                       m->gm_to_slave_ns,
                       m->slave_to_gm_ns,
                       m->scale_factor,
                       m->gm_first ? "true" : "false",
                       m->crystal_error_ns);

        if (len >= buffer_size - 100) {
            // Running out of buffer space
            TEL_LOG_ERROR("Warning: JSON buffer nearly full\n");
            break;
        }
    }

    // Add stats and close JSON
    len += snprintf(buffer + len, buffer_size - len,
                   "],\"stats\":{\"buffer_available\":%u,\"buffer_dropped\":%u}}\n",
                   available, dropped);

    return len;
}

/**
 * Send data over TCP with retry logic for transient errors
 *
 * @param pcb TCP connection
 * @param data Data to send
 * @param len Data length
 * @return true if sent successfully, false on fatal error
 */
static bool tcp_send_with_retry(struct tcp_pcb *pcb, const char *data, int len) {
    for (int retry = 0; retry < MAX_SEND_RETRIES; retry++) {
        err_t err = tcp_write(pcb, data, len, TCP_WRITE_FLAG_COPY);

        if (err == ERR_OK) {
            tcp_output(pcb);  // Flush
            return true;
        }

        // Check if error is transient or fatal
        if (err == ERR_MEM || err == ERR_WOULDBLOCK) {
            // Transient error - retry after delay
            TEL_LOG_DEBUG("Transient error %d, retry %d/%d\n",
                         err, retry + 1, MAX_SEND_RETRIES);
            cyw43_arch_poll();  // Process WiFi stack
            sleep_ms(SEND_RETRY_DELAY_MS);
            continue;
        }

        // Fatal error - don't retry
        TEL_LOG_ERROR("Fatal TCP error: %d\n", err);
        return false;
    }

    // All retries exhausted
    TEL_LOG_ERROR("Send failed after %d retries\n", MAX_SEND_RETRIES);
    return false;
}

/**
 * TCP connection and send loop
 */
static void tcp_client_loop(void) {
    struct tcp_pcb *pcb = NULL;
    ip_addr_t server_addr;

    // Parse server IP
    if (!ipaddr_aton(g_config.server_ip, &server_addr)) {
        TEL_LOG_ERROR("Invalid server IP: %s\n", g_config.server_ip);
        return;
    }

    TEL_LOG_INFO("Connecting to server %s:%u\n",
                g_config.server_ip, g_config.server_port);

    // Create TCP PCB
    pcb = tcp_new();
    if (!pcb) {
        TEL_LOG_ERROR("Failed to create TCP PCB\n");
        return;
    }

    // Connect to server (blocking)
    err_t err = tcp_connect(pcb, &server_addr, g_config.server_port, NULL);
    if (err != ERR_OK) {
        TEL_LOG_ERROR("TCP connect failed: %d\n", err);
        tcp_close(pcb);
        return;
    }

    // Wait for connection (poll with timeout)
    int timeout = 100;  // 10 seconds
    while (pcb->state != ESTABLISHED && timeout > 0) {
        cyw43_arch_poll();
        sleep_ms(100);
        timeout--;
    }

    if (pcb->state != ESTABLISHED) {
        TEL_LOG_ERROR("TCP connection timeout\n");
        tcp_close(pcb);
        return;
    }

    TEL_LOG_INFO("TCP connected\n");
    g_connected = true;

    // Batch buffer
    measurement_t batch[TELEMETRY_BATCH_SIZE];
    int batch_count = 0;

    // JSON buffer (large enough for max batch)
    char json_buffer[16384];  // ~16KB

    uint64_t last_activity = time_us_64();

    // Send loop
    while (g_connected) {
        // Read measurement from ring buffer (timeout 1s)
        measurement_t m;
        if (ring_buffer_read(g_config.ring_buffer, &m, 1000)) {
            // Add to batch
            batch[batch_count++] = m;
            last_activity = time_us_64();

            // Send batch when full
            if (batch_count >= TELEMETRY_BATCH_SIZE) {
                int json_len = build_json_message(batch, batch_count,
                                                  json_buffer, sizeof(json_buffer));

                if (tcp_send_with_retry(pcb, json_buffer, json_len)) {
                    g_batches_sent++;
                    TEL_LOG_DEBUG("Sent batch #%u (%d measurements)\n",
                                 g_batch_seq - 1, batch_count);
                } else {
                    g_send_failures++;
                    g_connected = false;
                    break;
                }

                batch_count = 0;
            }
        } else {
            // Timeout - check for keepalive
            if ((time_us_64() - last_activity) > (TELEMETRY_KEEPALIVE_MS * 1000ULL)) {
                // Send partial batch as keepalive
                if (batch_count > 0) {
                    int json_len = build_json_message(batch, batch_count,
                                                      json_buffer, sizeof(json_buffer));

                    if (tcp_send_with_retry(pcb, json_buffer, json_len)) {
                        g_batches_sent++;
                        TEL_LOG_DEBUG("Sent keepalive batch (%d measurements)\n",
                                     batch_count);
                        batch_count = 0;
                    } else {
                        g_connected = false;
                        break;
                    }
                }
                last_activity = time_us_64();
            }
        }

        // Poll WiFi stack
        cyw43_arch_poll();

        // Check connection status
        if (pcb->state != ESTABLISHED) {
            TEL_LOG_INFO("Connection lost\n");
            g_connected = false;
            break;
        }
    }

    // Cleanup
    tcp_close(pcb);
    g_connected = false;
    TEL_LOG_INFO("TCP connection closed\n");
}

/**
 * Core 1 entry point
 */
void telemetry_core1_entry(void) {
    TEL_LOG_INFO("Core 1 starting...\n");

    // Initialize WiFi (CYW43 driver)
    if (cyw43_arch_init()) {
        TEL_LOG_ERROR("Failed to initialize CYW43\n");
        return;
    }

    cyw43_arch_enable_sta_mode();

    // Main telemetry loop
    while (true) {
        // Connect to WiFi
        if (!wifi_connect()) {
            TEL_LOG_INFO("WiFi connection failed, retrying in %d ms\n",
                        TELEMETRY_RECONNECT_DELAY_MS);
            sleep_ms(TELEMETRY_RECONNECT_DELAY_MS);
            g_reconnects++;
            continue;
        }

        // Connect to TCP server and send data
        tcp_client_loop();

        // TCP connection ended - check if WiFi is still up
        if (wifi_is_connected()) {
            // WiFi is still connected - TCP server issue
            TEL_LOG_INFO("TCP disconnected, WiFi still up. Retrying TCP in %d ms\n",
                        TELEMETRY_RECONNECT_DELAY_MS);
            sleep_ms(TELEMETRY_RECONNECT_DELAY_MS);
            // Don't increment reconnects counter - just retrying TCP
        } else {
            // WiFi is down - need to reconnect WiFi
            TEL_LOG_INFO("WiFi disconnected. Reconnecting in %d ms\n",
                        TELEMETRY_RECONNECT_DELAY_MS);
            sleep_ms(TELEMETRY_RECONNECT_DELAY_MS);
            g_reconnects++;
        }
    }
}

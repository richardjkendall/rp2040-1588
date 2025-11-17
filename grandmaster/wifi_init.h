/**
 * WiFi Initialization Module
 *
 * Handles CYW43 driver initialization, WiFi connection, and static IP configuration.
 */

#ifndef WIFI_INIT_H
#define WIFI_INIT_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/**
 * Initialize WiFi and connect to network with static IP
 *
 * @return true if successful, false otherwise
 */
bool wifi_init_and_connect(void);

/**
 * Get WiFi connection status
 *
 * @return true if connected, false otherwise
 */
bool wifi_is_connected(void);

/**
 * Get assigned IP address as string
 *
 * @param buffer Output buffer for IP address string
 * @param buffer_len Length of output buffer
 */
void wifi_get_ip_address(char *buffer, size_t buffer_len);

/**
 * Poll WiFi/lwIP stack (must be called regularly in main loop)
 *
 * Call this frequently (e.g., every 10-100ms) to handle WiFi and network events
 */
void wifi_poll(void);

#endif // WIFI_INIT_H

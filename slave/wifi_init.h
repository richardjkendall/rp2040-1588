/**
 * WiFi Initialization Module Header
 */

#ifndef WIFI_INIT_H
#define WIFI_INIT_H

#include <stdbool.h>
#include <stddef.h>

/**
 * Initialize WiFi and connect to network
 * Returns: true on success, false on error
 */
bool wifi_init_and_connect(void);

/**
 * Check if WiFi is connected
 * Returns: true if connected, false otherwise
 */
bool wifi_is_connected(void);

/**
 * Get current IP address as string
 */
void wifi_get_ip_address(char *buffer, size_t buffer_len);

/**
 * Poll WiFi and lwIP stack
 * Must be called regularly from main loop
 */
void wifi_poll(void);

#endif // WIFI_INIT_H
